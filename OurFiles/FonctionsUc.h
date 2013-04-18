
// Quand il est indiqué XUPDATE, cela signifie qu'il y a une mise a jour 
// a effectuer pour le portage du programme dans la carte finale
// (en gros : pinout, vitesse d'horloge, etc.)

#define RECBUFFER 		50			// Taille du buffer reception UART
#define FCY             40000000	// Nombre d'instructions par secondes (IPS)
#define BAUDRATE        9585		// Debit UART (RS232)
#define BRGVAL          ((FCY/BAUDRATE)/4)-1 // Precalcul pour le baudrate generator
#define N				2			// Nombre de moteurs

#define BPA				PORTAbits.RA8 //LATAbits.LATA8
#define BPB				PORTCbits.RC5 //LATCbits.LATC5
#define BPC				PORTBbits.RB0 //LATBbits.LATB0
#define BPD				PORTBbits.RB1 //LATBbits.LATB1
#define LEDA			LATBbits.LATB12
#define LEDB			LATBbits.LATB13	
#define LEDC			LATAbits.LATA10	
#define LEDD			LATAbits.LATA7
#define DEB1			PORTCbits.RC3
#define DEB2			PORTCbits.RC4
#define EV1				LATBbits.LATB10
#define DIR1			LATBbits.LATB14

#define FALSE			0x00
#define TRUE			0x01
#define NEUF			9

//void Tempo1mS(unsigned int nbr);
//void Tempo1uS(unsigned int nbr);
void InitUART2(void);
void InitCLK(void);
//void Initpwm(void); 
//void InitT2(void);
char pwm(unsigned char motor, float value);
//float _abs(float value);
void UART2PutChar( char ch );
