#include <p33FJ128MC804.h>
#include "FonctionsUc.h"


/*void Tempo1mS(unsigned int nbr)
{
	unsigned int i,j;

	for(i=0;i<nbr;i++)
		for(j=0;j<8000;j++);
}*/

/*void Tempo1uS(unsigned int nbr) // Défaut constant de +0.5us
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
}*/

/*void Initpwm(void)
{
	P1TCONbits.PTEN = 1; 		// PWM Time base is On
	P1TPER = 2000 - 1; 			// 20kHz PWM (2000 counts @40MIPS)
	PWM1CON1bits.PEN1L = 1;		// PWM1L1 pin is enabled for PWM output
//	PWM1CON1bits.PEN2L = 1;		// PWM1L2 pin is enabled for PWM output

	P1DC1 = -4000;	// 0    = 100.00% Power
//	P1DC2 = 4000;	// 4000 =   0.00% Power
}*/

void InitCLK(void)
{
	// Configure Oscillator to operate the device at 40Mhz
	// Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
	//Fosc= 8M*32/(4*2)=80Mhz for 20Mhz input clock // Version proto
	//Fosc= 8M*40/(2*2)=80Mhz for  8Mhz input clock // Version finale

//	// Quartz 20 MHz
//	PLLFBD = 30; 			// M = 32
//	CLKDIVbits.PLLPRE = 2;	// N1 = 4
//	CLKDIVbits.PLLPOST = 0; // N2 = 2
	
	//Quartz 8 MHz
	// Multiply by 40 for 160MHz VCO output (8MHz XT oscillator)
	PLLFBD = 38;			// M = 40
	
	// FRC: divide by 2, PLLPOST: divide by 2, PLLPRE: divide by 2
	CLKDIVbits.PLLPRE = 0;	// N1 = 2
	CLKDIVbits.PLLPOST = 0;	// N2 = 2
	
	OSCTUN = 0;				// Tune FRC oscillator, if FRC is used
	RCONbits.SWDTEN = 0;	// Disable Watch Dog Timer

//	// Clock switch to incorporate PLL
//	__builtin_write_OSCCONH(0x03);		// Initiate Clock Switch to Primary
//										// Oscillator with PLL (NOSC=0b011)
//	__builtin_write_OSCCONL(0x01);		// Start clock switching
//	while (OSCCONbits.COSC != 0b011);	// Wait for Clock switch to occur	
//	while(OSCCONbits.OSWEN) {};			// Wait for PLL to lock
}



void InitUART2(void) 
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
	U2MODEbits.BRGH = 1;	// Bit3 4 clocks per bit period
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
}

/*void InitT2(void)
{
	T2CONbits.TCKPS = 1;	// 1:8 Prescaler
	PR2 = 5000;				// Time to autoreload (1 ms @40 MIPS)
	IFS0bits.T2IF = 0;		// Interrupt flag cleared
	IEC0bits.T2IE = 1;		// Interrupt enabled
	T2CONbits.TON = 1;		// Timer enabled
}*/

/*
char pwm(unsigned char motor, float value) // Value = +/- 4000
{
	if(value >  4000) value =  4000;
	if(value < -4000) value = -4000;
	
	if(motor==GAUCHE) value = -value;

	switch(motor)
	{
		case AVANT:
		case GAUCHE: if(value > 0)	// Moteur Gauche
				{
					DIRG  = 1;		// Position incremente
					P1DC2 = (unsigned int)(4000 - value);		
				}
				else
				{
					DIRG  = 0;		// Position decremente
					P1DC2 = (unsigned int)(4000 + value);		
				}
				break;
		case ARRIERE:
		case DROITE: if(value > 0)	// Moteur Droit
				{
					DIRD  = 1;		// Position incremente
					P1DC1 = (unsigned int)(4000 - value);		
				}
				else
				{
					DIRD  = 0;		// Position decremente
					P1DC1 = (unsigned int)(4000 + value);		
				}
				break;
		default : return -1;
	}
	return 0;
}
*/

/*float _abs(float value)
{
	if(value<0) return -value;
	else		return  value;
}*/

/*******************************************************************************
Function: UART2PutChar( char ch )

Precondition:
    UART2Init must be called prior to calling this routine.

Overview:
    This routine writes a character to the transmit FIFO, and then waits for the
    transmit FIFO to be empty.

Input: Byte to be sent.

Output: None.

*******************************************************************************/
void UART2PutChar( char ch )
{
    U2TXREG = ch;
    #if !defined(__PIC32MX__)
        Nop();
    #endif
    while(U2STAbits.TRMT == 0);
}
