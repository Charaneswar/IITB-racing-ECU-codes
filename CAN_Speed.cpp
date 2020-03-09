#include "mbed.h"
#define apps_id 104
#define current_id 1313
#define voltage_id 1314
#define rpm_id 120
#define yaw_id 112

//------------------Declaration of Flag Variables-------------
int speed_request_flag = 1;
int enable_receive;
double rpm_right_receive;
int  disable_send_flag=0;
int enable_request_flag =0;
int   enable_sent = 0;

//==================motor initial setup variables=============
const char BTB_request[8]= {0x3D,0xE2,0,0,0,0,0,0};
const char disable_send[8]= {0x51,0x04,0,0,0,0,0,0};
const char enable_request[8]= {0x3D,0xE8,0,0,0,0,0,0};
const char enable_send[8]= {0x51,0,0,0,0,0,0,0};
const char acc_ramp_send[8]= {0x35,0,0,0,0,0,0,0};
const char dec_ramp_send[8]= {0xED,0,0,0,0,0,0,0};
//================Send function variables==================
CANMessage receive;
Ticker mc;

int torque_left=0;
int torque_right=0;
int rpm_right_recieve=0;
int rpm_left_receive=0;

char speed_send[8]= {0x31,0,0,0,0,0,0,0};
const char speed_request[8]= {0x3D,0x30,0x1E,0,0,0,0,0}; //30 ms cyclic





char voltage_send[8] = {0,0,0,0,0,0,0,0};
int voltage_send_temp =0;
double energy;



int attach_flag=0;

//===============================================APPS & BPS & STEER=======================

//===========================Working range of sensors============
//-------------calibration
const uint16_t APPS1_max = 850;
const uint16_t APPS1_min = 330;
const uint16_t APPS2_max = 830;
const uint16_t APPS2_min = 310;
const uint16_t BPS_max = 630;
const uint16_t BPS_min = 40;
const uint16_t STEER_left = 130;
const uint16_t STEER_right = 968;
const uint16_t STEER_mid =  650;
const uint16_t BPS_actuated = 170;
//----------------------------
const uint16_t BPS_regen = 170;

//===============================Allowed  range of sensor values======

const uint16_t APPS1_gmax = 1000;
const uint16_t APPS1_gmin = 50;
const uint16_t APPS2_gmax = 1000;
const uint16_t APPS2_gmin = 50;
const uint16_t BPS_gmax = 1000;
const uint16_t BPS_gmin = 10;
const uint16_t STEER_gmax = 1000;
const uint16_t STEER_gmin = 10;

const uint16_t APPS1range = APPS1_max - APPS1_min;
const uint16_t APPS2range = APPS2_max - APPS2_min;
const uint16_t STEERrange_left = abs(STEER_left-STEER_mid);
const uint16_t STEERrange_right = abs(STEER_right-STEER_mid);

float APPS1_normalised;
float APPS2_normalised;
float APPS_avg;
float STEER_normalised;
float STEER_normalised_left;//With respect to steer mid
float STEER_normalised_right;// with respected steer rejected

//====================Input sensor values=================
uint16_t APPS1_in;
uint16_t APPS2_in;
uint16_t BPS_in;
uint16_t STEER_in;
uint16_t CURRENT_in;
uint32_t VOLTAGE_in;
uint32_t time_apps = 0;
uint32_t time_apps_last=0;


//===============================Plausibility and SCS checking variables====================

uint32_t time_out = 100; // maximum idle time allowed in ms
int apps_plaus_flag= 0;
int bps_plaus_flag= 0;
int bps_plaus_flag2 = 0;
int power;

Timer t1,t2,t3,t4;//t1 for APPS plausibility and t2 for BPS plausibility

//=======================================================Ediff variables==================================

int torque_ediff = 0;
uint16_t STEER_rejected = 15;//for incoming value without normalization
const float STEER_rejected_left = STEER_rejected *256/STEERrange_left;
const float STEER_rejected_right =STEER_rejected*256/STEERrange_right;
float kp = 0.12;//constant of proportionality for steering and torque
float ediff_max = 0 ;//allowed maximum ediff to be added above avg torque
float torque_left_temp=0;
float torque_right_temp=0;



//==============================================Power block variables================================

int Torquemax = 2000; // units in Num
const int fixed_Torquemax = 20;
int torque_threshold = 0.05*Torquemax;
int torque_avg = 0;
const int power_max = 75000;

//=====================================interrupt variables========================

uint16_t APPS1_read;
uint16_t APPS2_read;
uint16_t BPS_read;
uint16_t STEER_read;
uint16_t time_read;
uint16_t CURRENT_read;
uint32_t VOLTAGE_read;

uint32_t energy_prev;
double energy_time;
double energy_time_prev;



//============================LV Variables======================================
InterruptIn ignition_Sig(p20);
InterruptIn gr_90_Sig(p18);  // greater than 90% signal
InterruptIn shutdown_switch(p17);

DigitalOut open_FRG(p26);
DigitalOut rtds_Sig(p22); // ready to drive sound
DigitalOut brake_light_Sig(p21);
DigitalOut ECU_error(p25);
DigitalOut ams_volt_error(p24);
DigitalOut ams_temp_error(p23);

int press_brakes_to_start = 0;
bool flag_rtds = 0;
bool lvready = 0;//data is sent to motor only when lvready is true




CAN MC_can(p30, p29); //MC CAN
CAN SN_can(p9, p10); //Sensor CAN
Serial pc(USBTX,USBRX);

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);


//============================== send function ====================================================


void send ()   //This is  torque mode send fucntion which is called by 'TICKER' every 10ms
{
    

    if(speed_request_flag == 1)
        if(MC_can.write(CANMessage(201,&speed_request[0],8))){
            speed_request_flag = 0;
        }

    speed_send[2]=torque_right>>8;
    speed_send[1]=torque_right & 255;
    MC_can.write(CANMessage(201,&speed_send[0],8));
}  



//==========================================ISR for sensor==========================================
void sensor_read()
{
    if(SN_can.read(receive) && receive.id == apps_id) {

        APPS1_read  = (receive.data[0] << 2) | (receive.data[4] & 3);
        APPS2_read = (receive.data[1] << 2) | ((receive.data[4] >>2) & 3);
        BPS_read   = (receive.data[2] << 2) | ((receive.data[4] >>4) & 3);
        STEER_read  = (receive.data[3] << 2) | ((receive.data[4] >>6) & 3);
        time_read = (receive.data[7] << 16) | (receive.data[6] << 8) | receive.data[5];
    }

}

//==========================================ISR for motor===========================================
void mc_read()
{

    if (receive.id == 181) {//reads right mc
        if(receive.data[0] == 0xE8 && receive.data[1] == 1 ) { //checking enable
            enable_receive=1;
        }
        if(receive.data[0] == 0x30) {//reading speed
            rpm_right_receive = ((receive.data[3]<<16)+(receive.data[2]<<8)+receive.data[1] )*7000/32676;
        }



    }
}


//===================================motor initial setup==============================================

void initial_setup()
{

    if( disable_send_flag==0) {
        if( MC_can.write(CANMessage(201,&disable_send[0],3))) {
            disable_send_flag=1;
        }
    }

    if(enable_request_flag ==0) {
        if(MC_can.write(CANMessage(201,&enable_request[0],3))) {
            enable_request_flag =1;
        }
    }

    if(enable_receive==1) {
        if(MC_can.write(CANMessage(201,&enable_send[0],3))) {
            enable_sent=1;
        }

    }
}



//==================================================Main code====================================
    int main()
    {
  //      ECU_error = 1;
//        MC_error = 1;
        lvready=0;
        rtds_Sig = 0;

        pc.baud(9600);
        MC_can.frequency(500000);
        SN_can.frequency(1000000);
        led1 = 0;
        led2 = 0;
        led3 = 0;
        led4 = 0;

    
        pc.baud(9600);

        SN_can.attach(&sensor_read, CAN::RxIrq);
        MC_can.attach(&mc_read, CAN::RxIrq);

        while(1) {

            if(enable_sent==0) {
                initial_setup();
            }

            if(enable_sent==1 && attach_flag==0) {
                mc.attach(&send,0.010);
                attach_flag=1;
            }


            //=============reading sensor values============


            APPS1_in = APPS1_read;
            APPS2_in = APPS2_read;
            BPS_in = BPS_read;
            STEER_in = STEER_read;
            time_apps =time_read;

            if (APPS1_in <APPS1_gmin || APPS1_in>APPS1_gmax || APPS2_in<APPS2_gmin || APPS2_in>APPS2_gmax  ) {//generating signal outside operating range
                torque_left = 0;
                torque_right = 0;
            } else {// Arranging varibles within operating range

                if(APPS1_in < APPS1_min) {

                    APPS1_in = APPS1_min;
                } else if(APPS1_in > APPS1_max) {

                    APPS1_in = APPS1_max;
                }
                if(APPS2_in < APPS2_min) {
                    APPS2_in = APPS2_min;

                } else if(APPS2_in > APPS2_max) {

                    APPS2_in = APPS2_max;
                }

                //conversion to percentage values

                APPS1_normalised = (APPS1_in - APPS1_min)*100.0/APPS1range;
                APPS2_normalised = (APPS2_in - APPS2_min)*100.0/APPS2range;

                APPS_avg = 1.0*(APPS1_normalised+APPS2_normalised)/2.0; // APPS_avg is normalised

//====================================  led indications    ============================================================

                if(ignition_Sig.read()==1) {
                    led1 = 1;
                } else if(ignition_Sig.read()==0) {
                    led1 = 0;
                }

                if(gr_90_Sig.read()==1) {
                    led2 = 1;
                } else if(gr_90_Sig.read()==0) {
                    led2 = 0;
                }
//====================================  starting sequence      ====================================


                if(!flag_rtds) {
                    if(press_brakes_to_start==1 && BPS_in > BPS_actuated) {  // checking if ignition is on and brake is pressed, and ready is off

                        rtds_Sig = 1;

                        wait(2);
                        rtds_Sig = 0;
                        lvready=1;
                        led3 = 1;
                        flag_rtds=1;
                    }
                }



                torque_avg = 1.0*(Torquemax*APPS_avg)/100.0;

                torque_left_temp = torque_avg;
                torque_right_temp = torque_avg;


                torque_left = torque_left_temp;
                torque_right = torque_right_temp;
                pc.printf("torque_avg= %d , rpm_sent=%d, rpm_Recieve = %d \n ",torque_avg,torque_right*7000/32676,rpm_right_receive);

            }
        }
        }