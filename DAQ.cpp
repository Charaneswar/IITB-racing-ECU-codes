#include "mbed.h"
#include "ds1307.h"
#include "SDFileSystem.h"

SDFileSystem sd(p5, p6, p7, p8, "sd"); // the pinout on the mbed Cool Components workshop board
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
Serial pc(USBTX,USBRX);
DS1307 rtc(p28,p27);

int motor_counter = 0;
int sensor_counter = 0;
int sensor_flag=0;

//CAN
CAN mc(p30,p29);    //MC RX

CAN sensor(p9,p10);    //Sensor RX

CANMessage msg;
DigitalOut xbeeReset(p18);
Serial xbee(p13, p14);
 FILE *fp ;


Timer t1,t2;

char filename_s[30];

void SET_MODE_STOP()
{
    char can_msg[8];
    can_msg[0] = 0x34;
    can_msg[1] = 0x00;
    can_msg[2] = 0x01;
    for(int i=3; i<8; i++) {
        can_msg[i]=0x00;
    }

    mc.write(CANMessage(0x411, can_msg, 8));
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
   mc.write(CANMessage(0x411, U1_msg,8));  //for current measurement configuration
    
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
    mc.write(CANMessage(0x411, can_msg, 8));
}


//void ivt_current_start(){
//
//    SET_MODE_STOP();
//    wait_ms(100);
//    SET_CONFIG_I();
//    wait_ms(100);
//    SET_MODE_START();
// }


void xbee_reset(void)
{
    xbeeReset =0;
    wait(0.1);
    xbeeReset=1;
    pc.printf("reset done");

}
void mc_func(void)
{
  led2=!led2;
  if(sensor_flag == 0) {
        t1.start();
        sensor_flag=1;
    }
    mc.read(msg);
 

   if(motor_counter%8 == 0) {
    xbee.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d, \r\n",msg.id,msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5],msg.data[6],msg.data[7]);
  }

//    fprintf(fp,"%d,%f,%d,%d,%d,%d,%d,%d,%d,%d \r\n",msg.id,t1.read(),msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5],msg.data[6],msg.data[7]);
   
   motor_counter=motor_counter+1;
}

void sensor_func(void)
{
     led4=!led4;
    if(sensor_flag == 0) {
        t1.start();
        sensor_flag=1;
    }
    sensor.read(msg);
  if(sensor_counter%8 == 0) {
    xbee.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d, \r\n",msg.id,msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5],msg.data[6],msg.data[7]);
   }
//    fprintf(fp,"%d,%f,%d,%d,%d,%d,%d,%d,%d,%d \r\n",msg.id,t1.read(),msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5],msg.data[6],msg.data[7]);
    sensor_counter=sensor_counter+1;

}
void ivt_current_start(){

    SET_MODE_STOP();
    wait_ms(100);
    SET_CONFIG_I();
    wait_ms(100);
    SET_MODE_START();
 }


int main()
{
    mc.frequency(500000);
   sensor.frequency(1000000);
    pc.baud(57600);
    xbee.baud(57600);
     Ticker t2;
    xbeeReset =0;   //first time resetting the xbee
    wait(0.1);
    xbeeReset=1;

    t2.attach(&xbee_reset, 100);
    led1 = 0;
    led2 = 0;
    led3 = 0;
    led4 = 0;

    int sec = 0;
    int mins = 0;
    int hours = 0;
    int day = 0;
    int date = 0;
    int month = 0;
    int year = 0;
    
  ivt_current_start(); 
   rtc.gettime(&sec,&mins,&hours,&day,&date,&month,&year);
    
    
    sprintf(filename_s,"/sd/mydir/%d-%d-%d----%d-%d-%d.csv",date,month,year,hours,mins,sec);

    mkdir("/sd/mydir", 0777);

//    fp = fopen(filename_s, "a");
    led1=1;
   


    sensor.attach(&sensor_func, CAN::RxIrq);
    mc.attach(&mc_func, CAN::RxIrq);
    
  
    
//   fclose(fp);
    

}
