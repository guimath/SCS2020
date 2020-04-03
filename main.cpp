#include "mbed.h"
#include "glibr.h"

// Adress
#define adress_color_sensor 0x4B0       // same for every sensor
#define data_adress_general 0xFE        // will talk to every sensor
#define data_adress_sensor  0x00        // Specific to each sensor (between 0x00 and 0xFD)    

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

Serial USB_link(USBTX, USBRX);     // USB initialization

PwmOut LED(D9);                   // LED initialization

glibr capt1(D4,D5);                // I²C initialization : D4 = SDA ; D5 = SCL

CAN can1(PA_11, PA_12);            // CAN initialization : PA_11 = RD ; PA_12 = TD


CANMessage msg;

RawSerial tx(PA_9, NC);
RawSerial rx(NC, PA_10);

bool initialization(void)
{
    //  USB initialization
    USB_link.baud(115200);
    USB_link.printf("Debut prog\r\n");

    // CAN initialization
    can1.frequency(1000000);
    rx.baud(115200);
    
    // LED initialization
    LED.period_ms(10);
    LED.write(0);

    // Sensor initalization
    if( (capt1.ginit()) && (capt1.enableLightSensor(true)) && (capt1.enableProximitySensor(true)) ) {
        return true;
    } else {
        return false;
    }
}

int main (void)
{
    char c;
    uint16_t r,g,b ; // RGB values in 2 bytes
    uint8_t a ;      // proximity value in 1 byte
    char proximity_tresh = 250, color, state, message[8];           
    message[0] = data_adress_sensor; // (PID)
    
    if (initialization())  USB_link.printf("Initialization complete \r\n");
    else USB_link.printf("Error during initialization\r\n");

    while(1) {
        //reading of every interesting values
        capt1.readRedLight(r);
        capt1.readGreenLight(g);
        capt1.readBlueLight(b);
        capt1.readProximity(a);

        //calculation of color
        if (a<proximity_tresh) {
            color = 0 ;  // 0 means no object being measured
        } else if (r > g ) {
            color = 1 ;  // 1 means red
        } else {
            color = 2 ;  // 2 means green
        }
        
        // serial test :
        if(rx.readable()){
            c=rx.getc();
            USB_link.printf("%c",c);
        }
            
        //display of red, green and proximty variables
        //USB_link.printf("r: %hu g : %hu  prox : %hu \r\n ",r,g,a);

        //diplay of color value
        //USB_link.printf("color : %hu \r\n", color);
        
        // reading of the CAN bus
        if (can1.read(msg)) {
            
            //verification of the ID ( 0x4B0 ) and "soft" ID
            if ((msg.id==adress_color_sensor)&((msg.data[0]==data_adress_general)|(msg.data[0]==data_adress_sensor))) {
                
                state=msg.data[1];
                message[1]=state+0x40; // command
                switch (state) {
                    case send_RGB:  
                        message[2]= (char)((r & HIGH)>>8);  // data
                        message[3]= (char) (r & LOW);
                        message[4]= (char)((g & HIGH)>>8);
                        message[5]= (char) (g & LOW);                       
                        message[6]= (char)a ;
                        can1.write(CANMessage(adress_color_sensor,message,7)); 
                        break;

                    case send_RED:    
                        message[2]= (char)((r & HIGH)>>8);
                        message[3]= (char) (r & LOW);
                        can1.write(CANMessage(adress_color_sensor,message,4));
                        break;

                    case send_GREEN:    
                        message[2]= (char)((g & HIGH)>>8);
                        message[3]= (char) (g & LOW);
                        can1.write(CANMessage(adress_color_sensor,message,4));
                        break;

                    case send_BLUE:    
                        message[2]= (char)((b & HIGH)>>8);
                        message[3]= (char) (b & LOW);
                        can1.write(CANMessage(adress_color_sensor,message,4));
                        break;

                    case send_PROXIMITY:    
                        message[2] = a ;
                        can1.write(CANMessage(adress_color_sensor,message,3));
                        break;

                    case send_COLOR:    
                        message[2]=color;
                        can1.write(CANMessage(adress_color_sensor,message,3));
                        break;

                    case setup_LED:  
                        LED.write(msg.data[2]/258.0);
                        break;
                        
                    case setup_PROXIMITY_THRESHOLD :
                        proximity_tresh = msg.data[2];
                        break;
                }
            }
        }
    }
}