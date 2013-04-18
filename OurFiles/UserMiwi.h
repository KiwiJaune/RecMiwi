/************************************************
*
* UserUdp.h
* Fichier permettant de gérer une communication UDP
* Couche intermédiaire entre le main et le fichier udp.c
*
*
*****************************************************/

#ifndef __USER_MIWI__
#define __USER_MIWI__


#include "ConfigApp.h"
#if defined(PROTOCOL_P2P)
    #include "WirelessProtocols/P2P/P2P.h"
#elif defined(PROTOCOL_MIWI)
    #include "WirelessProtocols/MiWi/MiWi.h"
#elif defined(PROTOCOL_MIWI_PRO)
    #include "WirelessProtocols/MiWiPRO/MiWiPRO.h"
#endif

#include "WirelessProtocols/MCHP_API.h"

void StartMiwi(void);
void EnvoiMessageMiwi(char * message,unsigned int nbCara);

#endif