#ifndef __AX12_H__
#define __AX12_H__


//Bauderates AX12
#define BAUD1	9585
#define BAUD2
#define BAUD3
#define BAUD4
#define BAUD5
#define BAUD6
#define BAUD7
#define BAUD8
#define BAUD9	910000

//Contr�le en angle de L'AX12 (0� � 300�)
int AX12Pos(int baud,char id, int position);

//Param�tre de la vitesse de l'AX12
void AX12Vit(int baud,char id, unsigned int vitesse);

//Allumer la led de l'AX12
void AX12Led(int baud,char id, char led);

//Chercher le Baudrate d'un AX12
void AX12searchBaudrate(char id, double baudratemin, double baudratemax, char led);

//Reset d'un AX12
void AX12reset(char id, double baudratemin, double baudratemax);

//Activation du couple de l'AX12 (0 ou 1)
void AX12enableTorque(int baud,char id, unsigned int enableTorque);

//Changer le bauderate de L'AX12 
void AX12bauderate(int baud,char id, char finalBauderate);

//Changer l'ID de L'AX12 
void AX12id(int baud,char id, char newID);

//R�gler le maximum de couple en EEPROM (0 � 1023)
void AX12maxTorque(int baud,char id, int maxTorque);

//R�gler le minimum de la position du servomoteur (0 � 1023)		SI le mini et le maxi sont � 0 on passe en rotation libre
void AX12MinPos(int baud,char id, int minPos);

//R�gler le maximum de la position du servomoteur (0 � 1023)
void AX12MaxPos(int baud,char id, int maxPos);

#endif // __AX12_H__
