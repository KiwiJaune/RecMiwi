/************************************************
*
* UserMiwi.c
* Fichier permettant de gérer une communication Miwi
* Couche intermédiaire entre le main et le fichier MiWiPRO.c
*
*
*****************************************************/

#include "UserMiwi.h"

BYTE myChannel = 24;

StartMiwi()
{
int i;
//////////////////////////////////////////MIWI////////////////////////////////////
	BoardInit();

	MiApp_ProtocolInit(FALSE);

	MiApp_SetChannel(myChannel);

    MiApp_ConnectionMode(ENABLE_ALL_CONN);

    /*******************************************************************/
    // Function MiApp_EstablishConnection try to establish a new 
    // connection with peer device. 
    // The first parameter is the index to the active scan result, 
    //      which is acquired by discovery process (active scan). If 
    //      the value of the index is 0xFF, try to establish a 
    //      connection with any peer.
    // The second parameter is the mode to establish connection, 
    //      either direct or indirect. Direct mode means connection 
    //      within the radio range; indirect mode means connection 
    //      may or may not in the radio range. 
    /*******************************************************************/
    i = MiApp_EstablishConnection(0xFF, CONN_MODE_DIRECT);
    /*******************************************************************/
    // Display current opertion on LCD of demo board, if applicable
    /*******************************************************************/
    if( i != 0xFF )
    {
        //LCDDisplay((char *)" Connected Peer  on Channel %d", myChannel, TRUE);
    }
    else
    {
        MiApp_StartConnection(START_CONN_DIRECT, 10, 0);
    }
    DumpConnection(0xFF);
//////////////////////////////////FIN MIWI///////////////////////////////////////
}

EnvoiMessageMiwi(char * message,unsigned int nbCara)
{
unsigned int i;
	MiApp_FlushTx();
    for(i = 0; i < nbCara; i++)
    {
        MiApp_WriteData(message[i]);
    }
//    TxSynCount++;
                    

            // Function MiApp_BroadcastPacket is used to broadcast a message
            //    The only parameter is the boolean to indicate if we need to
            //       secure the frame

     MiApp_BroadcastPacket(FALSE); 
}