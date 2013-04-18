#include <p33FJ128MC804.h>
#include "asser.h"


/*_FOSCSEL(FNOSC_FRC);			// Internal FRC oscillator
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF  & POSCMD_HS);  
								// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
								// OSC2 Pin Function: OSC2 is Clock Output
								// Primary Oscillator Mode: HS Crystal
_FWDT(FWDTEN_OFF);              // Watchdog Timer Enabled/disabled by user software
								// (LPRC can be disabled by clearing SWDTEN bit in RCON register
_FPOR(FPWRT_PWR1);   			// Turn off the power-up timers.
_FGS(GCP_OFF);            		// Disable Code Protection
*/
float kp[N],ki[N],kd[N];
int revolutions[N];

void set_pid(float *coeffs)
{
	unsigned char i,j=0;

	for(i=0;i<N;i++)
	{
		kp[i] = coeffs[j++];
		ki[i] = coeffs[j++];
		kd[i] = coeffs[j++];
	}
}

// Calcul PID a reiterer a chaque milliseconde
float pid(unsigned char power,float * targ_pos,float * real_pos)
{
	unsigned char i;
	float erreur[N],cor[N];
	static float erreur_old[N]={0},erreur_sum[N]={0};

	// Calcul de la position reelle en mm
	//real_pos[0] = 1.000 * MM_SCALER * (((float)POS1CNT + ((float)revolutions[0])*0x10000)); //roue droite
	//real_pos[1] = 1.000 * MM_SCALER * (((float)POS2CNT + ((float)revolutions[1])*0x10000)); 

	if(power)
	{
		for(i=0;i<N;i++)
		{
			erreur[i] = (targ_pos[i] - real_pos[i]) * MM_INVSCALER; // Calcul de l'erreur en pas codeur
			if(_abs(erreur[i]) < ERROR_ALLOWED) erreur[i] = 0; 		// A utiliser en cas de sifflement du moteur
			if(((erreur_sum[i] + erreur[i]) > -10000) && (erreur_sum[i] + erreur[i]) < 10000) erreur_sum[i] += erreur[i]; // Ecretage de l'integrateur en amont (on peut aussi le placer en aval)
			//cor[i] = erreur[i]*kp[i] + (erreur[i] - erreur_old[i])*kd[i] + erreur_sum[i]/ki[i] ; // Calcul du correcteur PID, a noter: le terme Ki est inverse		
			cor[i] = erreur[i]*kp[i];
			erreur_old[i] = erreur[i]; // Mise a jour necessaire pour le terme derive
		}
		pwm(GAUCHE,cor[0]);
		pwm(DROITE,cor[1]);
	}
	return 0;
}

void Tempo1mS(unsigned int nbr)
{
	unsigned int i,j;

	for(i=0;i<nbr;i++)
		for(j=0;j<8000;j++);
}

void Tempo1uS(unsigned int nbr) // Défaut constant de +0.5us
{
	unsigned int i;

	for(i=0;i<nbr;i++)
	{
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
	}
}

void Initpwm(void)
{
	P1TCONbits.PTEN = 1; 		// PWM Time base is On
	P1TPER = 2000 - 1; 			// 20kHz PWM (2000 counts @40MIPS)
	PWM1CON1bits.PEN1L = 1;		// PWM1L1 pin is enabled for PWM output
	PWM1CON1bits.PEN2L = 1;		// PWM1L2 pin is enabled for PWM output

	P2TCONbits.PTEN = 1; 		// PWM Time base is On
	P2TPER = 2000 - 1; 			// 20kHz PWM (2000 counts @40MIPS)
	PWM2CON1bits.PEN1L = 1;		// PWM1L1 pin is enabled for PWM output


	P1DC1 = 0xFFFF;	// 0x0000 = 100.00% Power
	P1DC2 = 0xFFFF;	// 0xFFFF =   0.00% Power
	P2DC1 = 0xFFFF;
	// 0xFFFF =   0.00% Power
}

void InitQEI(void)
{
	QEI1CONbits.QEIM  = 0b110; // reset on index pulse
	QEI1CONbits.POSRES  = 1; // reset on index pulse
	QEI2CONbits.QEIM  = 0b111;
	QEI1CONbits.SWPAB = 1;
	QEI2CONbits.SWPAB = 0;
	//POS1CNT = 0x8000;
	//POS2CNT = 0x8000;
	POS1CNT = 0x0000;
	POS2CNT = 0x0000;
	DFLT1CONbits.QECK = 0b111;
	DFLT2CONbits.QECK = 0b111;
	IFS3bits.QEI1IF = 0;
	IFS4bits.QEI2IF = 0;
	IEC3bits.QEI1IE = 1;
	IEC4bits.QEI2IE = 1;
}

void initCLK(void)
{
	// Configure Oscillator to operate the device at 40Mhz
	// Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
	 //Fosc= 8M*32/(4*2)=80Mhz for 20Mhz input clock // Version proto
	 //Fosc= 8M*40/(2*2)=80Mhz for  8Mhz input clock // Version finale

	// Quartz 20 MHz
	//PLLFBD = 30; 			// M = 32
	//CLKDIVbits.PLLPRE = 2;	// N1 = 4
	//CLKDIVbits.PLLPOST = 0; // N2 = 2
	
	//Quartz 8 MHz
	PLLFBD = 38;			// M = 40
	CLKDIVbits.PLLPRE = 0;	// N1 = 2
	CLKDIVbits.PLLPOST = 0;	// N2 = 2
	
	OSCTUN = 0;				// Tune FRC oscillator, if FRC is used
	RCONbits.SWDTEN = 0;	// Disable Watch Dog Timer

	// Clock switch to incorporate PLL
	__builtin_write_OSCCONH(0x03);		// Initiate Clock Switch to Primary
										// Oscillator with PLL (NOSC=0b011)
	__builtin_write_OSCCONL(0x01);		// Start clock switching
	while (OSCCONbits.COSC != 0b011);	// Wait for Clock switch to occur	
	while(OSCCONbits.OSWEN) {};			// Wait for PLL to lock
}


/*void InitUART2(void) 
{
	// This is an EXAMPLE, so brutal typing goes into explaining all bit sets
	// The HPC16 board has a DB9 connector wired to UART2, so we will
	// be configuring this port only

	// configure U2MODE
	U2MODEbits.UARTEN = 0;	// Bit15 TX, RX DISABLED, ENABLE at end of func
	//U2MODEbits.notimplemented;	// Bit14
	U2MODEbits.USIDL = 0;	// Bit13 Continue in Idle
	U2MODEbits.IREN = 0;	// Bit12 No IR translation
	U2MODEbits.RTSMD = 0;	// Bit11 Simplex Mode
	//U2MODEbits.notimplemented;	// Bit10
	U2MODEbits.UEN = 0;		// Bits8,9 TX,RX enabled, CTS,RTS not
	U2MODEbits.WAKE = 0;	// Bit7 No Wake up (since we don't sleep here)
	U2MODEbits.LPBACK = 0;	// Bit6 No Loop Back
	U2MODEbits.ABAUD = 0;	// Bit5 No Autobaud (would require sending '55')
	U2MODEbits.URXINV = 0;	// Bit4 IdleState = 1  (for dsPIC)
	U2MODEbits.BRGH = 0;	// Bit3 16 clocks per bit period
	U2MODEbits.PDSEL = 0;	// Bits1,2 8bit, No Parity
	U2MODEbits.STSEL = 0;	// Bit0 One Stop Bit
	
	// Load a value into Baud Rate Generator.  Example is for 9600.
	// See section 19.3.1 of datasheet.
	//  U2BRG = (Fcy/(16*BaudRate))-1
	//  U2BRG = (40M/(16*9600))-1
	//  U2BRG = 260;  // 40Mhz osc, 9600 Baud (FCR)
	U2BRG = BRGVAL;  // 40Mhz osc, 9600 Baud (mode HS,XT PLL)

	// Load all values in for U1STA SFR
	U2STAbits.UTXISEL1 = 0;	//Bit15 Int when Char is transferred (1/2 config!)
	U2STAbits.UTXINV = 0;	//Bit14 N/A, IRDA config
	U2STAbits.UTXISEL0 = 0;	//Bit13 Other half of Bit15
	//U2STAbits.notimplemented = 0;	//Bit12
	U2STAbits.UTXBRK = 0;	//Bit11 Disabled
	U2STAbits.UTXEN = 0;	//Bit10 TX pins controlled by periph
	U2STAbits.UTXBF = 0;	//Bit9 *Read Only Bit*
	U2STAbits.TRMT = 0;	//Bit8 *Read Only bit*
	U2STAbits.URXISEL = 0;	//Bits6,7 Int. on character recieved
	U2STAbits.ADDEN = 0;	//Bit5 Address Detect Disabled
	U2STAbits.RIDLE = 0;	//Bit4 *Read Only Bit*
	U2STAbits.PERR = 0;		//Bit3 *Read Only Bit*
	U2STAbits.FERR = 0;		//Bit2 *Read Only Bit*
	U2STAbits.OERR = 0;		//Bit1 *Read Only Bit*
	U2STAbits.URXDA = 0;	//Bit0 *Read Only Bit*

	IPC7 = 0x4400;	// Mid Range Interrupt Priority level, no urgent reason

	IFS1bits.U2TXIF = 0;	// Clear the Transmit Interrupt Flag
	IEC1bits.U2TXIE = 1;	// Enable Transmit Interrupts
	IFS1bits.U2RXIF = 0;	// Clear the Recieve Interrupt Flag
	IEC1bits.U2RXIE = 1;	// Enable Recieve Interrupts

	U2MODEbits.UARTEN = 1;	// And turn the peripheral on

	U2STAbits.UTXEN = 1;
	// I think I have the thing working now.
}*/

void InitPorts() 
{
	// New
	// Pinout :
	// Ports A
	// 0 | I | RA0  : AN0
	// 1 | I | RA1  : AN1
	// 2 | I | RA2  : OSC
	// 3 | O | RA3  : INT (vers SBC)
	// 4 | I | RA4  : SW3
	// 5 
	// 6 
	// 7 | O | RA7  : DIRD
	// 8 | I | RA8  : SW1
	// 9 | I | RA9  : SW4
	// A | O | RA10 : DIRG
	
	// Ports B&C
	// 0 | I | RP0  | RB0  : AN2
	// 1 | I | RP1  | RB1  : AN3
	// 2 | I | RP2  | RB2  : Ag
	// 3 | I | RP3  | RB3  : Bg
	// 4 | I | RP4  | RB4  : SW2
	// 5 | I | RP5  | RB5  : PGD
	// 6 | I | RP6  | RB6  : PGC
	// 7 | I | RP7  | RB7  : INT0
	// 8 | I | RP8  | RB8  : SCL*
	// 9 | I | RP9  | RB9  : SDA*
	// A | I | RP10 | RB10 : Ad Strapé
	// B | I | RP11 | RB11 : Bd Strapé
	// C | O | RP12 | RB12 : ComBrakeG
	// D | O | RP13 | RB13 : PWMG
	// E | O | RP14 | RB14 : ComBrakeD
	// F | O | RP15 | RB15 : PWMD
	// 0 | I | RP16 | RC0  : DAC1RM*
	// 1 | I | RP17 | RC1  : DAC1LM*
	// 2 | I | RP18 | RC2  : Ig*
	// 3 | I | RP19 | RC3  : Strapé
	// 4 | I | RP20 | RC4  : Strapé
	// 5 | I | RP21 | RC5  : Id
	// 6 | I | RP22 | RC6  : PWM2H1*
	// 7 | I | RP23 | RC7  : PWM2L1*
	// 8 | I | RP24 | RC8  : RX*
	// 9 | I | RP25 | RC9  : TX*

	// * : Non utilisé pour le moment, donc configure en entree

	TRISA 	= 0b1111101101110111;
	//          FEDCBA9876543210
	TRISB 	= 0b0000111111111111;
	//          FEDCBA9876543210
	TRISC 	= 0b1111110100111111;
	//          FEDCBA9876543210

	//RPINR19bits.U2RXR	= 24;	// Rx	<==> RB5
	//RPOR12bits.RP25R		= 0b00101;	// Tx	<==> RB14
	TRISCbits.TRISC9 = 0;

//	RPINR14bits.QEA1R	= 0b00010;	// CHAg 	<==> RP2
//	RPINR14bits.QEB1R	= 0b00011;	// CHBg 	<==> RP3

	RPINR16bits.QEA2R	= 0b10011;	// CHAd 	<==> RP19
	RPINR16bits.QEB2R	= 0b10100;	// CHBd		<==> RP20

	RPINR14bits.QEA1R	= 10;//0b10011;	// CHAd 	<==> RP10
	RPINR14bits.QEB1R	= 11;//0b10100;	// CHBd		<==> RP11
	RPINR15bits.INDX1R	= 18;//0b10100;	// CHBd		<==> RP18
	
	RPINR0bits.INT1R	= 24;// Top capteur RP24
	RPINR1bits.INT2R	= 18;// Top motor RC2 RP18
	
	// Old
	////RPOR0bits.RP1R = 0x05;   			 // UART2 TX @ RP1(RB1), device 22, PIM 41, Expl6 RB12	// XUPDATE
	////TRISBbits.TRISB1 = 0;    			 // UART2 TX, RP1 is device RB1, Expl6 RB12 			// XUPDATE
	
	// Pinout :
	// 0 | I | RB0  : PGD
	// 1 | I | RB1  : PGC
	// 2 | O | RB2  : IR_Reducer (RED)
	// 3 | I | RB3  : IR
	// 4 | I | RB4  : SCK
	// 5 | O | RB5  : SO
	// 6 | I | RB6  : CHA1
	// 7 | I | RB7  : CHB1
	// 8 | I | RB8  : Rx
	// 9 | O | RB9  : DIR1
	// A | I | RB10 : CHA2
	// B | O | RB11 : DIRD
	// C | I | RB12 : CHB2
	// D | O | RB13 : PWM2
	// E | O | RB14 : Tx
	// F | O | RB15 : PWM1
	
	//TRISB 	= 0b0001010111011011;
	//          FEDCBA9876543210
	
	//RPINR14bits.QEA1R	= 0b00110;	// CHA1 	<==> RB6
	//RPINR14bits.QEB1R	= 0b00111;	// CHB1 	<==> RB7
	//RPINR16bits.QEA2R	= 0b01010;	// CHA2 	<==> RB10
	//RPINR16bits.QEB2R	= 0b01100;	// CHB2		<==> RB12
}

void InitT2(void)
{
	T2CONbits.TCKPS = 1;	// 1:8 Prescaler
	PR2 = 5000;			// Time to autoreload (1 ms @40 MIPS)
	IFS0bits.T2IF = 0;		// Interrupt flag cleared
	IEC0bits.T2IE = 1;		// Interrupt enabled
	T2CONbits.TON = 1;		// Timer enabled
}



char pwm(unsigned char motor, float value) // Value = +/- 4000
{
	if(value >  4095) value =  4095;
	if(value < -4095) value = -4095;
	
	if(value >  2000) value =  2000; // config de test, faible puissance
	if(value < -2000) value = -2000;
	

	if(motor==GAUCHE) value = -value;

	switch(motor)
	{
		case AVANT:
		case GAUCHE: if(value > 0)	// Moteur Gauche
				{
					DIRG  = 1;		// Position incremente
					P1DC2 = (unsigned int)(4095 - value);		
				}
				else
				{
					DIRG  = 0;		// Position decremente
					P1DC2 = (unsigned int)(4095 + value);		
				}
				break;
		case ARRIERE:
		case DROITE: if(value > 0)	// Moteur Droit
				{
					DIRD  = 1;		// Position incremente
					P1DC1 = (unsigned int)(4095 - value);		
				}
				else
				{
					DIRD  = 0;		// Position decremente
					P1DC1 = (unsigned int)(4095 + value);		
				}
				break;
		case BALISE: 
				if(value >  500) value =  500; // config de test, faible puissance
				if(value < -500) value = -500;
				if(value > 0)	// Moteur Balise
				{
					DIRB  = 1;		// Position incremente
					P2DC1 = (unsigned int)(4095 - value);		
				}
				else
				{
					DIRB  = 0;		// Position decremente
					P2DC1 = (unsigned int)(4095 + value);		
				}
				break;
		default : return -1;
	}
	return 0;
}


float _abs(float value)
{
	if(value<0) return -value;
	else		return  value;
}

//----------------------------------------------------------------------------



void __attribute__ ((interrupt, no_auto_psv)) _QEI1Interrupt(void) 
{/* moteur GAUCHE*/
	IFS3bits.QEI1IF = 0;
	QEI1CONbits.QEIM = 0;
	if((QEI1CONbits.UPDN==1)	&& POS1CNT < 0x8000)	revolutions[0]++; //overflow
	if((QEI1CONbits.UPDN==0)	&& POS1CNT > 0x8000)	revolutions[0]--; //underflow
	QEI1CONbits.QEIM = QEI2CONbits.QEIM;
}

void __attribute__ ((interrupt, no_auto_psv)) _QEI2Interrupt(void) 
{/* moteur DROIT*/
	IFS4bits.QEI2IF = 0;
	QEI2CONbits.QEIM = 0;
	if((QEI2CONbits.UPDN==1) && POS2CNT < 0x8000)	revolutions[1]++; //overflow
	if((QEI2CONbits.UPDN==0) && POS2CNT > 0x8000)	revolutions[1]--; //underflow
	QEI2CONbits.QEIM = QEI1CONbits.QEIM;
}
