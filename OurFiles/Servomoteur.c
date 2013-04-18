#include <p33FJ128MC804.h>
#include "Servomoteur.h"
#include "FonctionsUc.h"
#include "Include/uart2.h"

// Librairie compatible avec les servomoteurs AX12 et CDS


//Contrôle en angle du servomoteur (0 à 1023 soit 0° à 300°)
int ServoPos(int baud,char id, int position)
{
	char taille = 0, instruction = 0, adresse = 0, val = 0,val1 = 0, checksum = 0;
	double baudReel;
	int i;
	
	baudReel = 2000000 / (baud + 1);	
	InitUART2();	//Initialisation de la liaison série 2 (UART2)
	
	if(baudReel > 200000)
	{
		U2MODEbits.BRGH = 1;

		U2BRG = ((FCY/baudReel)/4)-1;
	}
	else
	{
		U2MODEbits.BRGH = 0;

		U2BRG = ((FCY/baudReel)/16)-1;
	}

	taille = 0x05;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x1E; 	
	val = position & 0b0000000011111111;
	val1 = position >> 8;
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val + val1));

	for(i = 0; i < 2; i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(val);
		UART2PutChar(val1);
		UART2PutChar(checksum);
	}
	for(i=0;i<=100;i++);

	return U2BRG;
}

//Paramètre de la vitesse du servomoteur (0 à 1023)
void ServoVit(int baud,char id, unsigned int vitesse)
{
	char taille = 0, instruction = 0, adresse = 0, val = 0,val1 = 0, checksum = 0;
	double baudReel;
	int i;
	
	baudReel = 2000000 / (baud + 1);
	InitUART2();	//Initialisation de la liaison série 2 (UART2)
	
	if(baudReel > 200000)
	{
		U2MODEbits.BRGH = 1;

		U2BRG = ((FCY/baudReel)/4)-1;
	}
	else
	{
		U2MODEbits.BRGH = 0;

		U2BRG = ((FCY/baudReel)/16)-1;
	}

	taille = 0x05;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x20;															
	val = vitesse & 0b0000000011111111;
	val1 = vitesse >> 8;
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val + val1));

	for(i = 0; i < 2; i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(val);
		UART2PutChar(val1);
		UART2PutChar(checksum);
	}

	for(i=0;i<=100;i++);
//		for(j=0;j<=50;j++);
}

//Allumer la led du servomoteur
void ServoLed(int baud,char id, char led)
{
	char taille = 0, instruction = 0, adresse = 0, checksum = 0;
	double baudReel;
	int i;
	
	baudReel = 2000000 / (baud + 1);
	InitUART2();	//Initialisation de la liaison série 2 (UART2)
	
	if(baudReel > 200000)
	{
		U2MODEbits.BRGH = 1;

		U2BRG = ((FCY/baudReel)/4)-1;
	}
	else
	{
		U2MODEbits.BRGH = 0;

		U2BRG = ((FCY/baudReel)/16)-1;
	}

	taille = 0x04;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x19; 															//Allumer ou éteindre une led
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + led));
	
	for(i = 0; i < 2; i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(led);
		UART2PutChar(checksum);
	}
	for(i=0;i<=100;i++);
//		for(j=0;j<=50;j++);
}


//Chercher le Baudrate du servomoteur
void ServoSearchBaudrate(char id, double baudratemin, double baudratemax, char led)
{
	char taille = 0, instruction = 0, adresse = 0, checksum = 0;
	int i;
	double baud;

	for(baud = baudratemin;baud <= baudratemax;baud++)
	{	
		InitUART2();	//Initialisation de la liaison série 2 (UART2)
		U2BRG = ((FCY/baud)/4)-1;			

		taille = 0x04;  															//Nbr de paramètres + 2
		instruction = 0x03;  														//Instruction écriture valeur
		adresse = 0x19; 															//Allumer ou éteindre une led
		checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + led));		
	
		for(i = 0; i < 2; i++)
		{
			UART2PutChar(0xFF);
			UART2PutChar(0xFF);
			UART2PutChar(id);
			UART2PutChar(taille);
			UART2PutChar(instruction);
			UART2PutChar(adresse);
			UART2PutChar(led);
			UART2PutChar(checksum);
		}

		for(i=0;i<=100;i++);
//			for(j=0;j<=50;j++);
	}
}

//Reset du servomoteur
void ServoReset(char id, double baudratemin, double baudratemax)
{
	char taille = 0, instruction = 0, checksum = 0;
	int i;
	double baud;

	for(baud = 1;baud <= 255;baud++)
	{	
		ServoBauderate(baud, id, 1);
	}
}

//Activation du couple du servomoteur (0 ou 1)
void ServoEnableTorque(int baud, char id, unsigned int enableTorque)
{
	char taille = 0, instruction = 0, adresse = 0, checksum = 0;
	int i;	
	double baudReel;

	baudReel = 2000000 / (baud + 1);

	InitUART2();	//Initialisation de la liaison série 2 (UART2)
	if(baudReel > 200000)
	{
		U2MODEbits.BRGH = 1;

		U2BRG = ((FCY/baudReel)/4)-1;
	}
	else
	{
		U2MODEbits.BRGH = 0;

		U2BRG = ((FCY/baudReel)/16)-1;
	}

	taille = 0x05;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x18;															
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + enableTorque));
	
	for(i = 0; i < 2; i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(enableTorque);
		UART2PutChar(checksum);
	}

	for(i=0;i<=100;i++);
//		for(j=0;j<=50;j++);
}


//Changer le bauderate du servomoteur
void ServoBauderate(int baud, char id, char finalBauderate)
{
	char taille = 0, instruction = 0, adresse = 0, val = 0, checksum = 0;
	int i;
	double baudReel;

	baudReel = 2000000 / (baud + 1);
	InitUART2();	//Initialisation de la liaison série 2 (UART2)
	
	if(baudReel > 200000)
	{
		U2MODEbits.BRGH = 1;

		U2BRG = ((FCY/baudReel)/4)-1;
	}
	else
	{
		U2MODEbits.BRGH = 0;

		U2BRG = ((FCY/baudReel)/16)-1;
	}

	taille = 0x04;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x04; 	
	val = finalBauderate;
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val));

	
	for(i = 0; i < 2; i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(val);
		UART2PutChar(checksum);
	}

	for(i=0;i<=100;i++);
//		for(j=0;j<=50;j++);
}

//Changer l'ID du servomoteur
void ServoId(int baud,char id, char newID)
{
	char taille = 0, instruction = 0, adresse = 0, val = 0, checksum = 0;
	int i;
	double baudReel;

	baudReel = 2000000 / (baud + 1);
	InitUART2();	//Initialisation de la liaison série 2 (UART2)
	
	if(baudReel > 200000)
	{
		U2MODEbits.BRGH = 1;

		U2BRG = ((FCY/baudReel)/4)-1;
	}
	else
	{
		U2MODEbits.BRGH = 0;

		U2BRG = ((FCY/baudReel)/16)-1;
	}

	taille = 0x04;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x03; 	
	val = newID;
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val));

	
	for(i = 0; i < 2; i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(val);
		UART2PutChar(checksum);
	}

	for(i=0;i<=100;i++);
//		for(j=0;j<=50;j++);
}

//Régler le maximum de couple du servomoteur en EEPROM (0 à 1023)
void ServoMaxTorque(int baud,char id, int maxTorque)
{
	char taille = 0, instruction = 0, adresse = 0, val = 0,val1 = 0, checksum = 0;
	int i;
	double baudReel;

	baudReel = 2000000 / (baud + 1);
	InitUART2();	//Initialisation de la liaison série 2 (UART2)
	
	if(baudReel > 200000)
	{
		U2MODEbits.BRGH = 1;

		U2BRG = ((FCY/baudReel)/4)-1;
	}
	else
	{
		U2MODEbits.BRGH = 0;

		U2BRG = ((FCY/baudReel)/16)-1;
	}

	taille = 0x05;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x0E; 	
	val = maxTorque & 0b0000000011111111;
	val1 = maxTorque >> 8;
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val + val1));

	
	for(i = 0; i < 2; i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(val);
		UART2PutChar(val1);
		UART2PutChar(checksum);
	}

	for(i=0;i<=100;i++);
//		for(j=0;j<=50;j++);
}

//Régler le minimum de la position du servomoteur (0 à 1023)
void ServoMinPos(int baud,char id, int minPos)
{
	char taille = 0, instruction = 0, adresse = 0, val = 0, val1 = 0, checksum = 0;
	int i;
	double baudReel;

	baudReel = 2000000 / (baud + 1);
	InitUART2();	//Initialisation de la liaison série 2 (UART2)
	
	if(baudReel > 200000)
	{
		U2MODEbits.BRGH = 1;

		U2BRG = ((FCY/baudReel)/4)-1;
	}
	else
	{
		U2MODEbits.BRGH = 0;

		U2BRG = ((FCY/baudReel)/16)-1;
	}
													
	val = minPos & 0b0000000011111111;
	val1 = minPos >> 8;

	taille = 0x05;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x06; 	
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val + val1));

	
	for(i = 0; i < 2; i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(val);
		UART2PutChar(val1);
		UART2PutChar(checksum);
	}

	for(i=0;i<=100;i++);
}

//Régler le maximum de la position du servomoteur (0 à 1023)
void ServoMaxPos(int baud,char id, int maxPos)
{
	char taille = 0, instruction = 0, adresse = 0, val = 0, val1 = 0, checksum = 0;
	int i;
	double baudReel;

	baudReel = 2000000 / (baud + 1);
	InitUART2();	//Initialisation de la liaison série 2 (UART2)
	
	if(baudReel > 200000)
	{
		U2MODEbits.BRGH = 1;

		U2BRG = ((FCY/baudReel)/4)-1;
	}
	else
	{
		U2MODEbits.BRGH = 0;

		U2BRG = ((FCY/baudReel)/16)-1;
	}
													
	val = maxPos & 0b0000000011111111;
	val1 = maxPos >> 8;

	taille = 0x05;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x08; 	
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val + val1));

	
	for(i = 0; i < 2; i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(val);
		UART2PutChar(val1);
		UART2PutChar(checksum);
	}

	for(i=0;i<=100;i++);
}