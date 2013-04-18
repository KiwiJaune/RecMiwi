/*********************************************************************
 *
 *  Main Application Entry Point and TCP/IP Stack Demo
 *  Module for Microchip TCP/IP Stack
 *   -Demonstrates how to call and use the Microchip TCP/IP stack
 *	 -Reference: AN833
 *
 *********************************************************************
 * FileName:        MainDemo.c
 * Dependencies:    TCPIP.h
 * Processor:       PIC18, PIC24F, PIC24H, dsPIC30F, dsPIC33F, PIC32
 * Compiler:        Microchip C32 v1.05 or higher
 *					Microchip C30 v3.12 or higher
 *					Microchip C18 v3.30 or higher
 *					HI-TECH PICC-18 PRO 9.63PL2 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright (C) 2002-2009 Microchip Technology Inc.  All rights
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy, and
 * distribute:
 * (i)  the Software when embedded on a Microchip microcontroller or
 *      digital signal controller product ("Device") which is
 *      integrated into Licensee's product; or
 * (ii) ONLY the Software driver source files ENC28J60.c, ENC28J60.h,
 *		ENCX24J600.c and ENCX24J600.h ported to a non-Microchip device
 *		used in conjunction with a Microchip ethernet controller for
 *		the sole purpose of interfacing with the ethernet controller.
 *
 * You should refer to the license agreement accompanying this
 * Software for additional information regarding your rights and
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 *
 * Author              Date         Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Nilesh Rajbharti		4/19/01		Original (Rev. 1.0)
 * Nilesh Rajbharti		2/09/02		Cleanup
 * Nilesh Rajbharti		5/22/02		Rev 2.0 (See version.log for detail)
 * Nilesh Rajbharti		7/9/02		Rev 2.1 (See version.log for detail)
 * Nilesh Rajbharti		4/7/03		Rev 2.11.01 (See version log for detail)
 * Howard Schlunder		10/1/04		Beta Rev 0.9 (See version log for detail)
 * Howard Schlunder		10/8/04		Beta Rev 0.9.1 Announce support added
 * Howard Schlunder		11/29/04	Beta Rev 0.9.2 (See version log for detail)
 * Howard Schlunder		2/10/05		Rev 2.5.0
 * Howard Schlunder		1/5/06		Rev 3.00
 * Howard Schlunder		1/18/06		Rev 3.01 ENC28J60 fixes to TCP, 
 *									UDP and ENC28J60 files
 * Howard Schlunder		3/01/06		Rev. 3.16 including 16-bit micro support
 * Howard Schlunder		4/12/06		Rev. 3.50 added LCD for Explorer 16
 * Howard Schlunder		6/19/06		Rev. 3.60 finished dsPIC30F support, added PICDEM.net 2 support
 * Howard Schlunder		8/02/06		Rev. 3.75 added beta DNS, NBNS, and HTTP client (GenericTCPClient.c) services
 * Howard Schlunder		12/28/06	Rev. 4.00RC added SMTP, Telnet, substantially modified TCP layer
 * Howard Schlunder		04/09/07	Rev. 4.02 added TCPPerformanceTest, UDPPerformanceTest, Reboot and fixed some bugs
 * Howard Schlunder		xx/xx/07	Rev. 4.03
 * HSchlunder & EWood	08/27/07	Rev. 4.11
 * HSchlunder & EWood	10/08/07	Rev. 4.13
 * HSchlunder & EWood	11/06/07	Rev. 4.16
 * HSchlunder & EWood	11/08/07	Rev. 4.17
 * HSchlunder & EWood	11/12/07	Rev. 4.18
 * HSchlunder & EWood	02/11/08	Rev. 4.19
 * HSchlunder & EWood   04/26/08    Rev. 4.50 Moved most code to other files for clarity
 * KHesky               07/07/08    Added ZG2100-specific support
 * HSchlunder & EWood   07/24/08    Rev. 4.51
 * Howard Schlunder		11/10/08    Rev. 4.55
 * Howard Schlunder		04/14/09    Rev. 5.00
 * Howard Schlunder		07/10/09    Rev. 5.10
 ********************************************************************/
/*
 * This macro uniquely defines this file as the main entry point.
 * There should only be one such definition in the entire project,
 * and this file must define the AppConfig variable as described below.
 */

#define THIS_IS_STACK_APPLICATION

// Include all headers for any enabled TCPIP Stack functions
#include "TCPIP Stack/TCPIP.h"
#include "TCPIP Stack/UDPPerformanceTest.h"

// Include functions specific to this stack application
#include "Main804.h"

//Include le fichier de gestion de communication udp
#include "OurFiles/UserUdp.h"
#include "OurFiles/Pilotage.h"

//Include le fichier de gestion de communication miwi
//#include "OurFiles/UserMiwi.h"

#include "OurFiles/asser.h"
#include <stdio.h>
#include <uart.h>

/////////////////////////////////I2C//////////////////////////////////////////
//#include "i2cEmem.h" //AZAI2C
///////////////////////////////FIN I2C///////////////////////////////////////


/////////////////////////////////////////////MIWI//////////////////////////////
/************************ HEADERS ****************************************/
#include "WirelessProtocols/Console.h"
#include "ConfigApp.h"
#include "HardwareProfile.h"
#include "WirelessProtocols/MCHP_API.h"
//#include "WirelessProtocols/LCDBlocking.h"

#include "WirelessProtocols/SymbolTime.h"

    #include <p33FJ128MC804.h>    
/************************** VARIABLES ************************************/
#define LIGHT   0x01
#define SWITCH  0x02

/*************************************************************************/
// AdditionalNodeID variable array defines the additional 
// information to identify a device on a PAN. This array
// will be transmitted when initiate the connection between 
// the two devices. This  variable array will be stored in 
// the Connection Entry structure of the partner device. The 
// size of this array is ADDITIONAL_NODE_ID_SIZE, defined in 
// ConfigApp.h.
// In this demo, this variable array is set to be empty.
/*************************************************************************/
#if ADDITIONAL_NODE_ID_SIZE > 0
    BYTE AdditionalNodeID[ADDITIONAL_NODE_ID_SIZE] = {LIGHT};
#endif

/*************************************************************************/
// The variable myChannel defines the channel that the device
// is operate on. This variable will be only effective if energy scan
// (ENABLE_ED_SCAN) is not turned on. Once the energy scan is turned
// on, the operating channel will be one of the channels available with
// least amount of energy (or noise).
/*************************************************************************/

///////////////////////////////FIN MIWI//////////////////////////////////////////////

#define  MAX_CHNUM	 			7		// Highest Analog input number in Channel Scan
#define  SAMP_BUFF_SIZE	 		8		// Size of the input buffer per analog input
#define  NUM_CHS2SCAN			8		// Number of channels enabled for channel scan

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

#define JACK PORTBbits.RB0

//unsigned int  BufferA[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));
//unsigned int  BufferB[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));
//unsigned int ADC_Results[8];
//unsigned int DmaBuffer = 0;
unsigned int adversaire;
//unsigned char flag_balise;

// Declare AppConfig structure and some other supporting stack variables
APP_CONFIG AppConfig;
//BYTE AN0String[8];

// Use UART2 instead of UART1 for stdout (printf functions).  Explorer 16 
// serial port hardware is on PIC UART2 module.
//#if defined(EXPLORER_16)//TODO
	int __C30_UART = 2;
//#endif


// Private helper functions.
// These may or may not be present in all applications.
static void InitAppConfig(void);
static void InitializeBoard(void);
static void ProcessIO(void);

unsigned char ValeurNRJ[100];
extern unsigned char RAMBuffer[100];	//RAM area which will work as EEPROM for Master I2C device

unsigned char *RAMPtr;			//Pointer to RAM memory locations
unsigned int NumeroOctet=0;

unsigned int time_ms=0;
unsigned char time_s=0,ptr,pid_power=1;
char flag,courrier,recbuf[RECBUFFER];
unsigned int first,second;
int angle,distance,Rangle,Rdistance;
unsigned int Rfirst,Rsecond;

int entier16bits(unsigned char poidFort, unsigned char poidFaible);

// C30 and C32 Exception Handlers
// If your code gets here, you either tried to read or write
// a NULL pointer, or your application overflowed the stack
// by having too many local variables or parameters declared.

	void _ISR __attribute__((__no_auto_psv__)) _AddressError(void)
	{
	    Nop();
		Nop();
	}
	void _ISR __attribute__((__no_auto_psv__)) _StackError(void)
	{
	    Nop();
		Nop();
	}

//
// Main application entry point.
//

int main(void)
{
		/////////// Variables pour l'asservissement \\\\\\\\\\\\\

	float cons_pos[N],speed[N],accel[N],speed_max[N],accel_max[N],speed_def[N],accel_def[N],targ_pos[N],decel_point[N],origi_point[N],real_pos[N],rel_pos[N];
	float coeffs[N*3]={DEFAULT_KP,DEFAULT_KI,DEFAULT_KD,DEFAULT_KP,DEFAULT_KI,DEFAULT_KD},rel_pos_curvi;
	unsigned int i,Text[50];
	//unsigned int NombreMesure=0,DebutValeur=0,NombreValeur=0;
	unsigned char flagerror=0,motion[N]={0},sens[N]={0},run[N]={0};
	
	int Rayon,Angle;
	unsigned char Cote;
	float RapportGlobal;
	char jackRetire = 0;

		/////////// Fin variables pour l'asservissement \\\\\\\\\\\\\


	static TICK t = 0;
	static DWORD dwLastIP = 0;
	unsigned int j=0;
	Trame trame;	
	static BYTE mess[2];
	mess[0] = 0x01;
	mess[1] = 0x02;
	trame.message = mess;
	trame.nbChar = 2;

	TRISBbits.TRISB0 = 1;	// Jack en entrée


//miwi
//    BYTE i;
    BYTE TxSynCount = 0;
    BYTE TxSynCount2 = 0;
    BYTE TxNum = 0;
    BYTE RxNum = 0;
    
	int truc = 0;

	char capteurEvitementPrec = 0;
	
	char mesureCapteur;
	
    initCLK(); 		//Initialisation de l'horloge
//	InitUART2();	//Initialisation de la liaison série 2 (UART2)
//	InitPorts(); 	//Initialisation des ports E/S | XUPDATE : Changer le pinout !
//	InitQEI(); 		//Initialisation des entrées en quadrature | XUPDATE : Penser au SWAP : CHA <> CHB 
//	Initpwm();		// Configuration du module PWM 
//	InitT2();		// Configuration du timer 2	

	// Init external interrput
//	IEC1bits.INT1IE = 1;
//	IEC1bits.INT2IE = 1;
	

	StartMiwi();

	// Initialize application specific hardware
	InitializeBoard();
	
	// Initialize stack-related hardware components that may be 
	// required by the UART configuration routines
    TickInit();

	// Initialize Stack and application related NV variables into AppConfig.
	InitAppConfig();
	
	UDPInit();
	// Initialize core stack layers (MAC, ARP, TCP, UDP) and
	// application modules (HTTP, SNMP, etc.)
    StackInit();

	//Initialisation de la liaison série 2 (UART2)
	InitUART2();
	
	// Now that all items are initialized, begin the co-operative
	// multitasking loop.  This infinite loop will continuously 
	// execute all stack-related tasks, as well as your own
	// application's functions.  Custom functions should be added
	// at the end of this loop.
    // Note that this is a "co-operative mult-tasking" mechanism
    // where every task performs its tasks (whether all in one shot
    // or part of it) and returns so that other tasks can do their
    // job.
    // If a task needs very long time to do its job, it must be broken
    // down into smaller pieces so that other tasks can have CPU time.
			
ENC_CS_TRIS = 0;
		ENC_CS_IO = 0;
	
 	UDPPerformanceTask();
	InitUserUdp();

	EnvoiUserUdp(trame);

	TRISBbits.TRISB8 = 0;		// Allimentation ON
	LATBbits.LATB8 = 1;

	TRISAbits.TRISA0 = 1;

	while(1)
    {
		if(!jackRetire && JACK)
		{
			jackRetire = 1;
			trame.nbChar = 2;
			mess[0] = 0xC2;
			mess[1] = 0x52;
			trame.message = mess;

			EnvoiUserUdp(trame);
		}

		mesureCapteur = PORTAbits.RA0;
		if(!capteurEvitementPrec && mesureCapteur)
		{
			trame.nbChar = 2;
			mess[0] = 0xC2;
			mess[1] = 0x78;
			trame.message = mess;

			EnvoiUserUdp(trame);

			capteurEvitementPrec = mesureCapteur;
		}

		if(capteurEvitementPrec && !mesureCapteur)
		{
			trame.nbChar = 2;
			mess[0] = 0xC2;
			mess[1] = 0x79;
			trame.message = mess;

			EnvoiUserUdp(trame);
			capteurEvitementPrec = mesureCapteur;
		}
		
        // Blink LED0 (right most one) every second.
        if(TickGet() - t >= TICK_SECOND/2ul)
        {
            t = TickGet();
//          LED0_IO ^= 1;
//			LED6_IO ^= 1;
        }

		ENC_CS_TRIS = 0;
		ENC_CS_IO = 0;

        // This task performs normal stack task including checking
        // for incoming packet, type of packet and calling
        // appropriate stack entity to process it.
        StackTask();
   
		trame = ReceptionUserUdp();
		if(trame.nbChar != 0)
		{
			AnalyseTrame(trame);
			EnvoiUserUdp(trame);
			// Données reçus en UDP
		}		
					
					
   //EnvoiUserUdp(envoiBalise);
   
        
//		trame = ReceptionUserUdp();
		
//		if(trame.nbChar != 0)
//		{
//			// TODOCHRIS Trame recue, la traiter
//			trame = AnalyseTrame(trame);
//			EnvoiUserUdp(trame);
//		}

        // This tasks invokes each of the core stack application tasks
        StackApplications();
        
		if( MiApp_MessageAvailable() )
		{
			for(i = 0; i < rxMessage.PayloadSize; i++)
			{
				trame.message[i] = rxMessage.Payload[i];
			}
			trame.nbChar = rxMessage.PayloadSize;

			EnvoiUserUdp(trame);

			MiApp_DiscardMessage();
		}

PHY_CS_TRIS = 0;
PHY_CS = 1;  

   
	}
}

// Writes an IP address to the LCD display and the UART as available
void DisplayIPValue(IP_ADDR IPVal)
{
//	printf("%u.%u.%u.%u", IPVal.v[0], IPVal.v[1], IPVal.v[2], IPVal.v[3]);
    BYTE IPDigit[4];
	BYTE i;
#ifdef USE_LCD
	BYTE j;
	BYTE LCDPos=16;
#endif

	for(i = 0; i < sizeof(IP_ADDR); i++)
	{
	    uitoa((WORD)IPVal.v[i], IPDigit);

		#if defined(STACK_USE_UART)
			putsUART(IPDigit);
		#endif

		#ifdef USE_LCD
			for(j = 0; j < strlen((char*)IPDigit); j++)
			{
				LCDText[LCDPos++] = IPDigit[j];
			}
			if(i == sizeof(IP_ADDR)-1)
				break;
			LCDText[LCDPos++] = '.';
		#else
			if(i == sizeof(IP_ADDR)-1)
				break;
		#endif

		#if defined(STACK_USE_UART)
			while(BusyUART());
			WriteUART('.');
		#endif
	}

	#ifdef USE_LCD
		if(LCDPos < 32u)
			LCDText[LCDPos] = 0;
		LCDUpdate();
	#endif
}

// Processes A/D data from the potentiometer
static void ProcessIO(void)
{
    // Convert potentiometer result into ASCII string
//    uitoa((WORD)ADC1BUF0, AN0String);
}


/****************************************************************************
  Function:
    static void InitializeBoard(void)

  Description:
    This routine initializes the hardware.  It is a generic initialization
    routine for many of the Microchip development boards, using definitions
    in HardwareProfile.h to determine specific initialization.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
static void InitializeBoard(void)
{	

	// Crank up the core frequency
	PLLFBD = 38;				// Multiply by 40 for 160MHz VCO output (8MHz XT oscillator)
	CLKDIV = 0x0000;			// FRC: divide by 2, PLLPOST: divide by 2, PLLPRE: divide by 2

	//TRISAbits.TRISA8 = 0;
	//TRISBbits.TRISB9 = 1;
	//TRISAbits.TRISA9 = 0;
	//TRISAbits.TRISA4 = 0;

	TRISCbits.TRISC0 = 1; // Switch compte tour
	TRISAbits.TRISA8 = 1; // Capteur balise
//	TRISCbits.TRISC6 = 0; // 
//	TRISCbits.TRISC7 = 0; // 


//	TRISBbits.TRISB7 = 1;
//	RPINR1bits.INT2R = 0b00111;	// INT2  <==> RB7

//	TRISBbits.TRISB10 = 0;
	RPOR1bits.RP3R = 0b01000;	// SCK1 <==> RB3

//	TRISBbits.TRISB4 = 1;
	RPINR20bits.SDI1R = 0b00100; // SDI1  <==> RP4

//	TRISBbits.TRISB11 = 0;
	RPOR1bits.RP2R = 0b00111;	// SDO1 <==> RB2

#if defined(SPIRAM_CS_TRIS)
	SPIRAMInit();
#endif
#if defined(EEPROM_CS_TRIS)
	XEEInit();
#endif
#if defined(SPIFLASH_CS_TRIS)
	SPIFlashInit();
#endif

	//Configuration des ports pour la liaison UART2
	RPINR19bits.U2RXR	= 24;	// Rx	<==> RP24-RC8
	TRISCbits.TRISC9 	= 0;
	RPOR12bits.RP25R	= 0b00101;	// Tx	<==> RP25-RC9

	//Initialisation du sens de communication pour les AX12
	LATCbits.LATC2 = 1;	// 1 J'envoie et 0 je réceptionne
}

/*********************************************************************
 * Function:        void InitAppConfig(void)
 *
 * PreCondition:    MPFSInit() is already called.
 *
 * Input:           None
 *
 * Output:          Write/Read non-volatile config variables.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/
// MAC Address Serialization using a MPLAB PM3 Programmer and 
// Serialized Quick Turn Programming (SQTP). 
// The advantage of using SQTP for programming the MAC Address is it
// allows you to auto-increment the MAC address without recompiling 
// the code for each unit.  To use SQTP, the MAC address must be fixed
// at a specific location in program memory.  Uncomment these two pragmas
// that locate the MAC address at 0x1FFF0.  Syntax below is for MPLAB C 
// Compiler for PIC18 MCUs. Syntax will vary for other compilers.
//#pragma romdata MACROM=0x1FFF0
static ROM BYTE SerializedMACAddress[6] = {MY_DEFAULT_MAC_BYTE1, MY_DEFAULT_MAC_BYTE2, MY_DEFAULT_MAC_BYTE3, MY_DEFAULT_MAC_BYTE4, MY_DEFAULT_MAC_BYTE5, MY_DEFAULT_MAC_BYTE6};
//#pragma romdata

static void InitAppConfig(void)
{
	AppConfig.Flags.bIsDHCPEnabled = TRUE;
	AppConfig.Flags.bInConfigMode = TRUE;
	memcpypgm2ram((void*)&AppConfig.MyMACAddr, (ROM void*)SerializedMACAddress, sizeof(AppConfig.MyMACAddr));
//	{
//		_prog_addressT MACAddressAddress;
//		MACAddressAddress.next = 0x157F8;
//		_memcpy_p2d24((char*)&AppConfig.MyMACAddr, MACAddressAddress, sizeof(AppConfig.MyMACAddr));
//	}
	AppConfig.MyIPAddr.Val = MY_DEFAULT_IP_ADDR_BYTE1 | MY_DEFAULT_IP_ADDR_BYTE2<<8ul | MY_DEFAULT_IP_ADDR_BYTE3<<16ul | MY_DEFAULT_IP_ADDR_BYTE4<<24ul;
	AppConfig.DefaultIPAddr.Val = AppConfig.MyIPAddr.Val;
	AppConfig.MyMask.Val = MY_DEFAULT_MASK_BYTE1 | MY_DEFAULT_MASK_BYTE2<<8ul | MY_DEFAULT_MASK_BYTE3<<16ul | MY_DEFAULT_MASK_BYTE4<<24ul;
	AppConfig.DefaultMask.Val = AppConfig.MyMask.Val;
	AppConfig.MyGateway.Val = MY_DEFAULT_GATE_BYTE1 | MY_DEFAULT_GATE_BYTE2<<8ul | MY_DEFAULT_GATE_BYTE3<<16ul | MY_DEFAULT_GATE_BYTE4<<24ul;
	AppConfig.PrimaryDNSServer.Val = MY_DEFAULT_PRIMARY_DNS_BYTE1 | MY_DEFAULT_PRIMARY_DNS_BYTE2<<8ul  | MY_DEFAULT_PRIMARY_DNS_BYTE3<<16ul  | MY_DEFAULT_PRIMARY_DNS_BYTE4<<24ul;
	AppConfig.SecondaryDNSServer.Val = MY_DEFAULT_SECONDARY_DNS_BYTE1 | MY_DEFAULT_SECONDARY_DNS_BYTE2<<8ul  | MY_DEFAULT_SECONDARY_DNS_BYTE3<<16ul  | MY_DEFAULT_SECONDARY_DNS_BYTE4<<24ul;


	// SNMP Community String configuration
	#if defined(STACK_USE_SNMP_SERVER)
	{
		BYTE i;
		static ROM char * ROM cReadCommunities[] = SNMP_READ_COMMUNITIES;
		static ROM char * ROM cWriteCommunities[] = SNMP_WRITE_COMMUNITIES;
		ROM char * strCommunity;
		
		for(i = 0; i < SNMP_MAX_COMMUNITY_SUPPORT; i++)
		{
			// Get a pointer to the next community string
			strCommunity = cReadCommunities[i];
			if(i >= sizeof(cReadCommunities)/sizeof(cReadCommunities[0]))
				strCommunity = "";

			// Ensure we don't buffer overflow.  If your code gets stuck here, 
			// it means your SNMP_COMMUNITY_MAX_LEN definition in TCPIPConfig.h 
			// is either too small or one of your community string lengths 
			// (SNMP_READ_COMMUNITIES) are too large.  Fix either.
			if(strlenpgm(strCommunity) >= sizeof(AppConfig.readCommunity[0]))
				while(1);
			
			// Copy string into AppConfig
			strcpypgm2ram((char*)AppConfig.readCommunity[i], strCommunity);

			// Get a pointer to the next community string
			strCommunity = cWriteCommunities[i];
			if(i >= sizeof(cWriteCommunities)/sizeof(cWriteCommunities[0]))
				strCommunity = "";

			// Ensure we don't buffer overflow.  If your code gets stuck here, 
			// it means your SNMP_COMMUNITY_MAX_LEN definition in TCPIPConfig.h 
			// is either too small or one of your community string lengths 
			// (SNMP_WRITE_COMMUNITIES) are too large.  Fix either.
			if(strlenpgm(strCommunity) >= sizeof(AppConfig.writeCommunity[0]))
				while(1);

			// Copy string into AppConfig
			strcpypgm2ram((char*)AppConfig.writeCommunity[i], strCommunity);
		}
	}
	#endif

	// Load the default NetBIOS Host Name
	memcpypgm2ram(AppConfig.NetBIOSName, (ROM void*)MY_DEFAULT_HOST_NAME, 16);
	FormatNetBIOSName(AppConfig.NetBIOSName);

	#if defined(ZG_CS_TRIS)
		// Load the default SSID Name
		if (sizeof(MY_DEFAULT_SSID_NAME) > sizeof(AppConfig.MySSID))
		{
		    ZGSYS_DRIVER_ASSERT(5, (ROM char *)"AppConfig.MySSID[] too small.\n");
		}
		memcpypgm2ram(AppConfig.MySSID, (ROM void*)MY_DEFAULT_SSID_NAME, sizeof(MY_DEFAULT_SSID_NAME));
	#endif

	#if defined(EEPROM_CS_TRIS)
	{
		BYTE c;
		
	    // When a record is saved, first byte is written as 0x60 to indicate
	    // that a valid record was saved.  Note that older stack versions
		// used 0x57.  This change has been made to so old EEPROM contents
		// will get overwritten.  The AppConfig() structure has been changed,
		// resulting in parameter misalignment if still using old EEPROM
		// contents.
		XEEReadArray(0x0000, &c, 1);
	    if(c == 0x60u)
		    XEEReadArray(0x0001, (BYTE*)&AppConfig, sizeof(AppConfig));
	    else
	        SaveAppConfig();
	}
	#elif defined(SPIFLASH_CS_TRIS)
	{
		BYTE c;
		
		SPIFlashReadArray(0x0000, &c, 1);
		if(c == 0x60u)
			SPIFlashReadArray(0x0001, (BYTE*)&AppConfig, sizeof(AppConfig));
		else
			SaveAppConfig();
	}
	#endif
}

#if defined(EEPROM_CS_TRIS) || defined(SPIFLASH_CS_TRIS)
void SaveAppConfig(void)
{
	// Ensure adequate space has been reserved in non-volatile storage to 
	// store the entire AppConfig structure.  If you get stuck in this while(1) 
	// trap, it means you have a design time misconfiguration in TCPIPConfig.h.
	// You must increase MPFS_RESERVE_BLOCK to allocate more space.
	#if defined(STACK_USE_MPFS) || defined(STACK_USE_MPFS2)
		if(sizeof(AppConfig) > MPFS_RESERVE_BLOCK)
			while(1);
	#endif

	#if defined(EEPROM_CS_TRIS)
	    XEEBeginWrite(0x0000);
	    XEEWrite(0x60);
	    XEEWriteArray((BYTE*)&AppConfig, sizeof(AppConfig));
    #else
	    SPIFlashBeginWrite(0x0000);
	    SPIFlashWrite(0x60);
	    SPIFlashWriteArray((BYTE*)&AppConfig, sizeof(AppConfig));
    #endif
}
#endif

int entier16bits(unsigned char poidFort, unsigned char poidFaible)
{
	int poidFortInt;
	poidFortInt = (int)poidFort;
	int retour = poidFort << 8;

	retour = (int)( retour | poidFaible);
	return retour;
}


void __attribute__ ((interrupt, no_auto_psv)) _U2RXInterrupt(void) 
{
	IFS1bits.U2RXIF = 0;
	if(ptr > RECBUFFER - 2) ptr = RECBUFFER - 2;
	recbuf[ptr] = U2RXREG & 0x00FF;
	if(recbuf[ptr++]=='?')
	{
		recbuf[--ptr]=0;
		courrier=ptr;
		ptr=0;
	}
}
void __attribute__ ((interrupt, no_auto_psv)) _U2TXInterrupt(void) 
{
	IFS1bits.U2TXIF = 0;
}
/*
void __attribute__ ((interrupt, no_auto_psv)) _INT1Interrupt(void) // Top capteur
{
	IFS1bits.INT1IF = 0;
//	if(INTCON2bits.INT1EP)
//	{
//		first = POS1CNT;
//	}
//	else
//	{
//		second = POS1CNT;
//		if(distance < abs((int)first - (int)second));
//		{
//			distance = abs((int)first - (int)second);
//			if(first + second < 2000)	angle = 360;
//			else						angle = 0;
//			if(first < second) 			angle += ((int)first + (int)second)/2;
//			else 						angle += ((int)first + (int)second - 2000)/2;
//		}
//
//	}
	
	if(INTCON2bits.INT1EP) // On voit
	{
		if(first==3000) first = POS1CNT;
		
	}
	else				// On voit plus
	{
		second = POS1CNT;
	}
	INTCON2bits.INT1EP = !INTCON2bits.INT1EP;
}
*/
void __attribute__ ((interrupt, no_auto_psv)) _INT2Interrupt(void) // Top motor
{
	IFS1bits.INT2IF = 0;
	POS1CNT=0;
	/*Rangle = angle;
	Rdistance = distance;
	angle = 0;
	distance = 0;*/
//	Rfirst  = first;
//	Rsecond = second;
//	first = 3000;
//	second = 3000;
//	flag_balise = 1;
}

void __attribute__ ((interrupt, no_auto_psv)) _T2Interrupt(void) 
{
	IFS0bits.T2IF = 0;
	/*static unsigned int codeurold, codeur, vitesse;
	static int real_vitesse, err_vitesse, cons_vitesse=16;
	codeur = POS1CNT;
	real_vitesse = abs((int)codeur - (int)codeurold);
	err_vitesse = cons_vitesse - real_vitesse;

	pwm(BALISE,(float)err_vitesse * 10 );
	codeurold = codeur;*/
	if(pid_power)	pwm(BALISE,110);
	else			pwm(BALISE,0);
	flag=0;
}

void __attribute__((interrupt, no_auto_psv)) _DMA5Interrupt(void)
{
/*	if(DmaBuffer == 0)
	{
//		ADC_Results[0] = BufferA[0][0];
//		ADC_Results[1] = BufferA[1][0];
//		ADC_Results[2] = BufferA[2][0];
//		ADC_Results[3] = BufferA[3][0];
//		ADC_Results[4] = BufferA[6][0];
//		ADC_Results[5] = BufferA[7][0];
	}
	else
	{
//		ADC_Results[0] = BufferB[0][0];
//		ADC_Results[1] = BufferB[1][0];
//		ADC_Results[2] = BufferB[2][0];
//		ADC_Results[3] = BufferB[3][0];
//		ADC_Results[4] = BufferB[6][0];
//		ADC_Results[5] = BufferB[7][0];
	}

	DmaBuffer ^= 1;
*/
	IFS3bits.DMA5IF = 0;		// Clear the DMA0 Interrupt Flag
}