
#include "mbed.h"
CAN can1(p30,p29); //IVT can p9-Rx,p10-Tx
CANMessage receive;
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
Serial pc(USBTX,USBRX);

void SET_MODE_STOP()
{
    char can_msg[8];
    can_msg[0] = 0x34;
    can_msg[1] = 0x00;
    can_msg[2] = 0x01;
    for(int i=3; i<8; i++) {
        can_msg[i]=0x00;
    }

    can1.write(CANMessage(0x411, can_msg, 8));
}

void SET_CONFIG_I()
{

    char U1_msg[8];
    U1_msg[0] = 0x20;  // for current
    U1_msg[1] = 0x02;
    U1_msg[2] = 0x00;
    U1_msg[3] = 0x3C;
    for(int i=4; i<8; i++) {
        U1_msg[i]=0x00;
    }
    can1.write(CANMessage(0x411, U1_msg,8));  //for current measurement configuration
    
}

void SET_CONFIG_CC()
{

    char U1_msg[8];
    U1_msg[0] = 0x26;  // for current
    U1_msg[1] = 0x02;
    U1_msg[2] = 0x00;
    U1_msg[3] = 0x1E;
    for(int i=4; i<8; i++) {
        U1_msg[i]=0x00;
    }
    can1.write(CANMessage(0x411, U1_msg,8));  //for current measurement configuration
    
}

void SET_MODE_START()
{
    char can_msg[8];
    can_msg[0] = 0x34;
    can_msg[1] = 0x01;
    can_msg[2] = 0x01;
    for(int i=3; i<8; i++) {
        can_msg[i]=0x00;
    }
    can1.write(CANMessage(0x411, can_msg, 8));
}

void change_can_freq(){
   char can_msg[8];
    can_msg[0] = 0x3A;
    can_msg[1] = 0x08;
    can_msg[2] = 0x00;
    for(int i=3; i<8; i++) {
        can_msg[i]=0x00;
    }
    can1.write(CANMessage(0x411, can_msg, 8));
    wait_ms(1000);    
}


void ivt_current_start(){

    SET_MODE_STOP();
    wait_ms(100);
    SET_CONFIG_I();
    wait_ms(100);
    SET_CONFIG_CC();
    wait_ms(100);
    //change_can_freq();
    //can1.frequency(250000);
    //wait_ms(1000);
    SET_MODE_START();
 
 }
int main(){

    int curr_data=5;
    can1.frequency(500000);
    
    //can1.frequency(500000);
    pc.baud(115200); 
    ivt_current_start(); 
    while (1) {
        led1 = 1;
        led2 = 1;
        while(!can1.read(receive)) led2 = !led2;
            if (receive.id == 0x521){
              led4 = 1;
            curr_data = receive.data[5] + receive.data[4]*256;
            printf("%d",curr_data);
            printf("\n");  
                
                }
            if (receive.id==0x527) {  //1319 for CC
            led3 = 1;
            curr_data = receive.data[5] + receive.data[4]*256;
            printf("%d",curr_data);
            printf("\n"); 
             }
             
     }
}