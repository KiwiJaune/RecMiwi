#include "Pilotage.h"
#include "asser.h"
#include "Servomoteur.h"

// ATTENTION /!\ Ces fonctions ne doivent pas être bloquantes

unsigned char RAMBuffer[100];

void CoupureAlimentation()
{
	TRISBbits.TRISB8 = 0;
	LATBbits.LATB8 = 0;
}


// Analyse la trame recue et renvoie vers la bonne fonction de pilotage
// Trame t : Trame ethernet recue
Trame AnalyseTrame(Trame t)
{
	int param1, param2, param3;

	// Les messages ne commencant pas par 0xC2 ne nous sont pas adressés (RecIo)
	if(t.message[0] != 0xC2)
		return;

	switch(t.message[1])
	{
		case 0x10:
			// Bouge servomoteur
			param1 = t.message[2];						// Id servo
			param2 = t.message[3] * 256 + t.message[4];	// Position
			param3 = 19200;

			ServoPos(param3, param1, param2);
			break;

		case 0xF1:

			CoupureAlimentation();
			break;
			
		case 0xE0:
			// Commande réglage servomoteur

			switch(t.message[2])
			{
				case 0x24:
					// Envoi position servomoteur
					param1 = t.message[3];						// ID servo
					param2 = t.message[4];						// Baudrate
					param3 = t.message[5] * 256 + t.message[6];	// Position

					ServoPos(param2, param1, param3);
					
					break;

				case 0x25:
					// Envoi vitesse servomoteur
					param1 = t.message[3];						// ID servo
					param2 = t.message[4];						// Baudrate
					param3 = t.message[5] * 256 + t.message[6];	// Vitesse

					ServoVit(param2, param1, param3);
					
					break;

				case 0x26:
					// Envoi led servomoteur
					param1 = t.message[3];						// ID servo
					param2 = t.message[4];						// Baudrate
					param3 = t.message[5];						// Allume

					ServoLed(param2, param1, param3);
					
					break;

				case 0x27:
					// Envoi ID servomoteur
					param1 = t.message[3];						// ID servo
					param2 = t.message[4];						// Baudrate
					param3 = t.message[5];						// Nouveau ID

					ServoId(param2, param1, param3);
					
					break;

				case 0x28:
					// Envoi baudrate servomoteur
					param1 = t.message[3];						// ID servo
					param2 = t.message[4];						// Baudrate
					param3 = t.message[5];						// Nouveau baudrate

					ServoBauderate(param2, param1, param3);
					break;

				case 0x29:
					// Envoi position min servomoteur
					param1 = t.message[3];						// ID servo
					param2 = t.message[4];						// Baudrate
					param3 = t.message[5] * 256 + t.message[6];	// Position

					ServoMinPos(param2, param1, param3);
					
					break;

				case 0x2A:
					// Envoi position max servomoteur
					param1 = t.message[3];						// ID servo
					param2 = t.message[4];						// Baudrate
					param3 = t.message[5] * 256 + t.message[6];	// Position

					ServoMaxPos(param2, param1, param3);
					
					break;

				case 0x44:

					// Envoi reset
					param1 = t.message[3];						// ID servo

					ServoReset(param1, 9600, 1000000);

					break;
			}
			break;

		case 0xA0 :
		
			ENC_CS_TRIS = 0;
			ENC_CS_IO = 1;
			        
			PHY_CS_TRIS = 0;
			PHY_CS = 0; 

			EnvoiMessageMiwi(&t.message[2], t.nbChar - 2);

			break;
	}

	return t;
}
