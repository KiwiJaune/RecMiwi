
// Quand il est indiqué XUPDATE, cela signifie qu'il y a une mise a jour 
// a effectuer pour le portage du programme dans la carte finale
// (en gros : pinout, vitesse d'horloge, etc.)

#define RECBUFFER 		50			// Taille du buffer reception UART
#define FCY             40000000	// Nombre d'instructions par secondes (IPS)
#define BAUDRATE        115200		// Debit UART (RS232)
#define BRGVAL          ((FCY/BAUDRATE)/16)-1 // Precalcul pour le baudrate generator
#define N				2			// Nombre de moteurs
#define DEFAULT_KP		1//10//70			// Coefficient proporionnel 
#define DEFAULT_KI		50//20//10			// Coefficient integral (inverse !)
#define DEFAULT_KD		0//90//110			// Coefficient derive
#define CODEUR			500 		// Nombre de pas par tour moteur (sans le ratio x4)
#define REDUCTEUR		1	// Reducteur utilise en sortie d'arbre moteur (=1 si roue codeuse indépendante)
#define DIAMETRE_ROUE 	35*1.071//1.064019			// Diametre de la roue motrice (ou roue codeuse si indépendante) en mm 
#define PI 				3.1416		// Ben pi quoi
#define VOIE			280*1.014//1.0344827586206896551724137931034//1.006	//1.0344827586206896551724137931034			// Distance entre les deux roues en mm
#define COEFF_ROUE		1.0000		// Coeff d'ajustement pour le diametre de la roue
#define COEFF_VOIE		1.0000		// Coeff d'ajustement pour la voie
#define MM_SCALER		COEFF_ROUE*DIAMETRE_ROUE*PI/(4*CODEUR*REDUCTEUR) // Formule de conversion [pas]<==>[mm]
#define MM_INVSCALER	4*CODEUR*REDUCTEUR/(COEFF_ROUE*DIAMETRE_ROUE*PI)
#define DEFAULT_SPEED	500			// Vitesse par défaut en mm/s
#define	MAX_SPEED		500
#define DEFAULT_ACCEL	500			// Acceleration par défaut en mm/s^2
#define ERROR_ALLOWED	1			// En cas de sifflement moteur intempestif (en pas)
#define KP	0
#define KI	1
#define KD	2


#define PWMG 	LATBbits.LATB13
#define DIRG 	LATAbits.LATA10
#define BRAKEG 	LATBbits.LATB12
#define PWMD 	LATBbits.LATB15
#define DIRD 	LATAbits.LATA7
#define BRAKED 	LATBbits.LATB14

#define SW1 	LATAbits.LATA8
#define SW2 	LATBbits.LATB4
#define SW3 	LATAbits.LATA4
#define SW4 	LATAbits.LATA9

#define INT 	LATAbits.LATA3
#define STOP			0x01
#define VITESSE			0x02
#define ACCELERATION	0x03
#define AVANCE			0x04
#define PIVOT			0x05
#define VIRAGE			0x06
#define DISTANCE		0x08
#define ARRIVE			0x10
#define PIVOTG			0x11
#define PIVOTD			0x12
#define RECULE			0x13
#define COUPURE			0x66
#define COEFF_P			0x20
#define COEFF_I			0x21
#define COEFF_D			0x22
#define ROULEAU_AV		0x81
#define ROULEAU_AR		0x82

#define FALSE			0x00
#define TRUE			0x01


#define AVANT 			0			// Convention 
#define ARRIERE 		1
#define GAUCHE 			2			
#define DROITE 			3
#define ON				1
#define OFF				0
#define FREELY			0
#define SMOOTH			1
#define ABRUPT			2

#define BALISE 			4
#define PWMB 	LATCbits.LATC7 	// PW2L1
#define DIRB 	LATCbits.LATC6 // 


void set_pid(float * coeffs);
float pid(unsigned char power, float * targ_pos,float * real_pos);
void Tempo1mS(unsigned int nbr);
void Tempo1uS(unsigned int nbr);
//void InitUART2(void);
void InitPorts(void);
void initCLK(void);
void Initpwm(void); 
void InitQEI(void);
void InitT2(void);
char pwm(unsigned char motor, float value);
float _abs(float value);

