#include "mbed.h"
#include "glibr.h"

#define SR 0   // 0 = CAN | 1 = serie

// Adress
#define data_adress_sensor  0x00        // Specific to each sensor (between 0x00 and 0xFD)
#define data_adress_general 0xFF        // will talk to every sensor
#define adress_color_sensor 0x4B0       // CAN ID same for every sensor (only for CAN)

// Request commands
#define send_RGB       0x00
#define send_RED       0x01
#define send_GREEN     0x02
#define send_BLUE      0x03
#define send_PROXIMITY 0x04
#define send_COLOR     0x05

// Setup commands
#define setup_LED 0x08
#define setup_PROXIMITY_THRESHOLD 0x09

// Masks
#define HIGH 0xFF00
#define LOW  0x00FF

// Buffer
#define CAN_MAX     256
#define SR_MAX     1024

Serial USB_link(USBTX, USBRX);     // USB initialization
RawSerial tx(PA_9, NC);
RawSerial rx(NC, PA_10);
glibr capt1(D4,D5);// I²C initialization : D4 = SDA ; D5 = SCL
CAN can(PA_11, PA_12);      

// Buffer CAN
CANMessage canBuffer[CAN_MAX];
int canRempli = 0;
int canVide = 0;
int canPerdu = 0;

//Buffer SR
uint8_t srBuffer[SR_MAX];
int srRempli = 0;
int srVide = 0;
int srPerdu = 0;

// traitement tramme SR
int etatSR = 0;
uint8_t checksumSR1 = 0,checksumSR2 = 0,calc_checksumSR1=0,calc_checksumSR2=0;
uint8_t lenSR = 0, idSR,cmdSR, dataSR=0;

// tramme retour
char messageSR[15], messageCAN[8];
int lenRetour = 0;

// capteur et traitement
uint16_t r,g,b ; // RGB values in 2 bytes
uint8_t a ;      // proximity value in 1 byte
char proximity_tresh = 250, color;
char state;

void srRead();
/* Fonction récéptionnant la tramme serie
et la stockant dans un buffer en attendant traitement */

void srTraitement();
/* Fonction decryptant la tramme serie recue dans le buffer */

void envoiSR(char commande, char data);
/* Fonction envoyant une tramme serie au format herkulex */

void canRead();
/* Fonction receptionnant les messaages CAN dans un Buffer */

void canTraitement(const CANMessage &msg);
/* Fonction traitant le message CAN reçu et repondant si besoin */


void srRead()
{
    srBuffer[srRempli++] = rx.getc();       //stockage nouvel octet dans le buffer
    if (srRempli==SR_MAX) srRempli = 0;     // on recommence au debut du tableau si le max est atteint


    if (srRempli == srVide) { // buffer plein on perd un caractère (le premier recu)
        srVide++;   // le message commence donc un octet plus tard
        if (srVide == SR_MAX) srVide = 0;   // on recommence au debut du tableau si le max est atteint

        srPerdu++;  // mise en memoire : un message perdu
    }
}

void srTraitement()
{
    uint8_t c = srBuffer[srVide++];     // c prends la valeur d'un octet de la tramme
    if (srVide == SR_MAX) srVide = 0;   // on recommence au debut du tableau si le max est atteint

    switch (etatSR) {
        case 0: // Verification premier octet header (FF)
            calc_checksumSR1 = c;
            if (c==0xFF) {
                etatSR = 1;
            }
            break;
        case 1: // Verification dexième octet header (FF)
            calc_checksumSR1 += c;    //update checksum
            if (c==0xFF) {
                etatSR = 2;
            } else {
                etatSR = 0;
            }
            break;
        case 2: // traitement octet Packet Size
            calc_checksumSR1 += c;
            lenSR=c;
            if (lenSR<7) etatSR =0; //impossible
            else etatSR = 3;
            break;
        case 3: // traitement octet ID
            calc_checksumSR1 += c;
            idSR = c;
            if (idSR!= data_adress_sensor) etatSR =0; //le capteur n'est pas concerné
            else etatSR = 4;
            break;
        case 4: // traitement octet CMD
            calc_checksumSR1 += c;
            cmdSR = c;
            etatSR = 5;
            break;
        case 5: // traitement octet checkSum1
            checksumSR1 = c;
            etatSR = 6;
            break;
        case 6: // traitement octet checkSum2
            checksumSR2 = c;
            if (lenSR>7) {
                etatSR =7;// si le message comporte des datas
            } else {
                dataSR = 0x00;
                etatSR=8;
            }
            break;
        case 7: // octet data (un seul octet dans notre cas)
            calc_checksumSR1 += c;
            dataSR=c;
            etatSR =8;
            break;
        case 8: // verification des checksum et envoi
            calc_checksumSR1 &=0xFE ;
            calc_checksumSR2 = (~calc_checksumSR1 & 0xFE);
            etatSR = 0;
            if ((checksumSR1 == calc_checksumSR1) && (checksumSR2 == calc_checksumSR2)) {  // Verification validité de la tramme
                envoiSR(cmdSR,dataSR);// dataSR ne sera utilise que dans les cas de setup 
            }
            break;
    }

}

void envoiSR(char commande, char data)
{
    int j;
    messageSR[4]=commande+0x40; // CMD (doc)

    switch (commande) {
        case send_RGB:
            messageSR[7] = (char)((r & HIGH)>>8); // data
            messageSR[8] = (char) (r & LOW);
            messageSR[9] = (char)((g & HIGH)>>8);
            messageSR[10]= (char) (g & LOW);
            messageSR[11]= (char)a ;
            messageSR[2]=12;    // Packet size
            break;

        case send_RED:
            messageSR[7]= (char)((r & HIGH)>>8);
            messageSR[8]= (char) (r & LOW);
            messageSR[2]=9;
            break;

        case send_GREEN:
            messageSR[7]= (char)((g & HIGH)>>8);
            messageSR[8]= (char) (g & LOW);
            messageSR[2]=9;
            break;

        case send_BLUE:
            messageSR[7]= (char)((b & HIGH)>>8);
            messageSR[8]= (char) (b & LOW);
            messageSR[2]=9;
            break;

        case send_PROXIMITY:
            messageSR[7] = a ;
            messageSR[2]=8;
            break;

        case send_COLOR:
            messageSR[7]=color;
            messageSR[2]=8;
            break;

        case setup_LED:
            //LED.write(data/258.0);
            messageSR[2]=7;
            break;

        case setup_PROXIMITY_THRESHOLD :
            proximity_tresh = data;
            messageSR[2]=7;
            break;
    }
    messageSR[5]=0x00;    // checksum1
    //calcul des checksums
    for(j=0; j<messageSR[2]; j++) {
        if ((j!=5)&&(j!=6)) messageSR[5] += messageSR[j];
    }
    messageSR[5] &= 0xFE;   // checksum1
    messageSR[6] = (~messageSR[5] & 0xFE);//checksum2

    // envoi
    for (j=0; j<messageSR[2]; j++) {
        while (!tx.writeable());      // attente bluetooth libre
        tx.putc(messageSR[j]);        // ecriture octet par octet
    }
}


void canRead()
{
    can.read(canBuffer[canRempli++]);
    if (canRempli==CAN_MAX) {
        canRempli = 0;
    }
    if (canRempli == canVide) { // buffer plein on perd un message
        canVide++;
        if (canVide == CAN_MAX) canVide = 0;
        canPerdu++;
    }
}

void envoiCAN(const CANMessage &msg)
{

    state = msg.data[1];
     messageCAN[1]= state +0x40;// CMD return
    switch (state) {
        case send_RGB:
            messageCAN[2]= (char)((r & HIGH)>>8);
            messageCAN[3]= (char) (r & LOW);
            messageCAN[4]= (char)((g & HIGH)>>8);
            messageCAN[5]= (char) (g & LOW);
            messageCAN[6]= (char)a ;
            lenRetour=7;
            break;

        case send_RED:
            messageCAN[2]= (char)((r & HIGH)>>8);
            messageCAN[3]= (char) (r & LOW);
            lenRetour=4;
            break;

        case send_GREEN:
            messageCAN[2]= (char)((g & HIGH)>>8);
            messageCAN[3]= (char) (g & LOW);
            lenRetour=4;
            break;

        case send_BLUE:
            messageCAN[2]= (char)((b & HIGH)>>8);
            messageCAN[3]= (char) (b & LOW);
            lenRetour=4;
            break;

        case send_PROXIMITY:
            messageCAN[2] = a ;
            lenRetour=3;
            break;

        case send_COLOR:
            messageCAN[2]=color;
            lenRetour=3;
            break;

        case setup_LED:
            //LED.write(msg.data[2]/258.0);
            lenRetour=0;
            break;

        case setup_PROXIMITY_THRESHOLD :
            proximity_tresh = msg.data[2];
            lenRetour=0;
            break;
    }
    
    // envoi si besoin
    if (lenRetour>0) { 
        can.write(CANMessage(adress_color_sensor,messageCAN,lenRetour));
    }
}


int main()
{
    can.frequency(1000000);
    //rx.baud(115200);
    //tx.baud(115200);

    if(SR==0) { // liaison CAN selectionné
        can.attach(canRead);
        // le premier octet est toujours pareil
        messageCAN[0] = data_adress_sensor;
    

    } else if (SR==1) { // liaison Serie selectionnée
        rx.attach(&srRead); 
        // octets toujours pareil :
        messageSR[0]=0xff; // Start of packet
        messageSR[1]=0xff;
        messageSR[3]= data_adress_sensor; // pID
    }

    while(1) {

        /* acquisition capteurs */
        capt1.readRedLight(r);
        capt1.readGreenLight(g);
        capt1.readBlueLight(b);
        capt1.readProximity(a);

        /* calcul couleur */
        if (a<proximity_tresh) {
            color = 0 ;  // 0 Rien
        } else if (r > g ) {
            color = 1 ;  // 1 rouge
        } else {
            color = 2 ;  // 2 vert
        }
        
        /* liaison CAN */
        if (canRempli != canVide) { // si le buffer CAN n'est pas vide
            canVide++;
            if (canVide == CAN_MAX) canVide = 0;
            envoiCAN(canBuffer[canRempli-1]);
        }

        /* liaison serie */
        if (srRempli != srVide) { // si le buffer serie n'est pas vide
            srTraitement();          // traitement de la tramme sr
        }
    }
}
