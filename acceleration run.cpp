#include "mbed.h"
#define apps_id 104
#define current_id 1313
#define voltage_id 1314
#define rpm_id 120
#define yaw_id 112
//================Send function variables==================
CANMessage receive;
Ticker mc;
int mode_right=0;
int mode_left=0;
int torque_left=0;
int torque_right=0;
int rpm_right=0;
int rpm_left=0;
int mc_left_send_temp=0;
int mc_right_send_temp=0;
int counter_mc_left=0;
int counter_mc_right=0;
char mc_left_send[8]= {0,16,0,103,157,92,160,14};
char mc_right_send[8]= {0,16,0,155,162,92,160,14};
char voltage_send[8] ={0,0,0,0,0,0,0,0};
int voltage_send_temp =0;
double energy;
const int initialization_mode = 240;
const int torque_mode = 16;
const int torque_offset = 256;
//============================================ 1st data packet variables======================================

int counter_1_right=0;
int counter_1_left=0;
float torque_right_receive=0;
float torque_left_receive=0;
float rpm_right_receive=0;
float rpm_left_receive=0;
int Voltage_right_receive = 0;
int Voltage_left_receive = 0;
float power_right_receive=0;
float power_left_receive=0;
float efficiency_right_receive=0;
float efficiency_left_receive=0;

//==============================================2nd data packet variables===================================

int counter_2_right=0;
int counter_2_left=0;
int mode_right_receive=0;
int mode_left_receive=0;
int mc_status_right_receive=0;
int mc_status_left_receive=0;
int max_power_right_receive=0;
int max_power_left_receive=0;
int max_cornering_right_receive=0;
int max_cornering_left_receive=0;
int ign_lock_position_right_receive=0;
int ign_lock_position_left_receive=0;
float disconnect_hardware_right_receive=0;
float disconnect_hardware_left_receive=0;

//===================================3rd data packet variables===============================

int counter_3_right=0;
int counter_3_left=0;
int temp1_right_receive =0;
int temp2_right_receive =0;
int temp3_right_receive =0;
int temp1_left_receive =0;
int temp2_left_receive =0;
int temp3_left_receive =0;

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

int Torquemax = 20;
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
InterruptIn ignition_Sig(p17);
InterruptIn gr_90_Sig(p18);  // greater than 90% signal

DigitalOut rtds_Sig(p21); // ready to drive sound
DigitalOut brake_light_Sig(p22); 
DigitalOut ECU_error(p23);
DigitalOut MC_error(p24);

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


//============================== send function for constant torque mode ====================================================


void send ()   //This is  torque mode send fucntion which is called by 'TICKER' every 10ms
{

    if(mode_left==0) { //standby or error mode
        mc_left_send[1] = initialization_mode + counter_mc_left;// This corresponds to initialization mode(mode==15) as it takes binary value in range 11110000 to 11111111
    } else { //cosidering left motor takes negative torque-anticlockwise and range is 255 to 0 -->255 cooresponds min value and 0 corresponds to max value
        mc_left_send[1] = torque_mode + counter_mc_left ;//This corresponds to torque mode(mode==1) as it takes binary value in range 00010000 to 00011111
        mc_left_send_temp = torque_offset - torque_left;//converting torque vaue according to offset
        mc_left_send[3] = (mc_left_send_temp &256)>>8 ;//determing 9th bit
        mc_left_send[2] = mc_left_send_temp &255;//determining bit 1 to bit 8
    }

    if(mode_right==0) { //standby or error mode
        mc_right_send[1] = initialization_mode + counter_mc_right;// This corresponds to initialization mode(mode==15) as it takes binary value in range 11110000 to 11111111
    } else { //cosidering right motor takes negative torque-clockwise and range is 256 to 511
        mc_right_send[1] = torque_mode + counter_mc_right;//This corresponds to torque mode(mode==1) as it takes binary value in range 00010000 to 00011111
        mc_right_send_temp = torque_offset + torque_right;//converting torque vaue according to offset
        mc_right_send[3] = (mc_right_send_temp &256)>>8 ;//determing 9th bit
        mc_right_send[2] = mc_right_send_temp &255;//determining bit 1 to bit 8
    }

    
        voltage_send_temp =((Voltage_left_receive+Voltage_right_receive)/2); 
        voltage_send[1]= voltage_send_temp>>8;
         voltage_send[0]=voltage_send_temp&255;
         voltage_send[2] = press_brakes_to_start;
         voltage_send[3] = energy;

    if(SN_can.write(CANMessage(122,&voltage_send[0],8)));

    
    if(lvready &&  (torque_right > torque_threshold) && (torque_left > torque_threshold)) {
        if(MC_can.write(CANMessage(129, &mc_left_send[0], 8))) {  //Writing data to the CAN bus and incrementing the counter value
            counter_mc_left = (counter_mc_left + 1)%16;//ensures that counter takes value from 0 to 15
        } 
        if(MC_can.write(CANMessage(128, &mc_right_send[0], 8))) {  //Writing data to the CAN bus and incrementing the counter value
            counter_mc_right = (counter_mc_right + 1)%16;//ensures that counter takes value from 0 to 15
        }
    }

}
//torque mode send function ends here

//============================================                      =======================================

void ignition_Sig_rise()
{
    wait(0.010);
    if(ignition_Sig.read()==1 && gr_90_Sig.read() == 1) {
        press_brakes_to_start = 1;
        led4 = 1;
    } else {
//    if(ignition_Sig.read()==0 || gr_90_Sig.read() == 0){
        press_brakes_to_start = 0;
        led4 = 0;
    }
}

void shut_down()
{
    wait(2);
    if(ignition_Sig.read()==0 || gr_90_Sig.read() == 0) {
        lvready = 0;
        led3 = 0;
        flag_rtds = 0;
        press_brakes_to_start = 0;
        led4 = 0;
    }
}
void sensor_read()
{
    if(SN_can.read(receive) && receive.id == apps_id){
       
        APPS1_read  = (receive.data[0] << 2) | (receive.data[4] & 3);
        APPS2_read = (receive.data[1] << 2) | ((receive.data[4] >>2) & 3);     
        BPS_read   = (receive.data[2] << 2) | ((receive.data[4] >>4) & 3);
        STEER_read  = (receive.data[3] << 2) | ((receive.data[4] >>6) & 3);
        time_read = (receive.data[7] << 16) | (receive.data[6] << 8) | receive.data[5];
    }

}

void mc_read(){
    if(MC_can.read(receive)){
        if(receive.id==256) {//1st data packet right motor
    
    //              counter_1_right = receive.data[1] & 15;
            rpm_right = ((((224) & receive.data[2])>>5)+(receive.data[3]<<3)+ ((receive.data[4] & (3))<<11))*3 -12288;
            Voltage_right_receive = (((receive.data[4] & 252)>>2) + ((receive.data[5] & 15)<<6));
     // torque_right_receive = ((((31) & receive.data[2])<<4)+((receive.data[1]& (240))>>4))+ (-256);
    //              power_right_receive = ((receive.data[5] & 240)>>4) + ((receive.data[6] & 63)<<4);
    //              efficiency_right_receive = receive.data[7] & 127 ;
        } else if(receive.id==272) {//1st data packet left motor
    //              counter_1_left = receive.data[1] & 15;
            rpm_left = ((((224) & receive.data[2])>>5)+(receive.data[3]<<3)+ ((receive.data[4] & (3))<<11))*3 -12288;
            Voltage_left_receive = (((receive.data[4] & 252)>>2) + ((receive.data[5] & 15)<<6));
    // torque_left_receive = ((((31) & receive.data[2])<<4)+((receive.data[1]& (240))>>4))+ (-256);
    //              power_left_receive = ((receive.data[5] & 240)>>4) + ((receive.data[6] & 63)<<4);
    //              efficiency_left_receive = receive.data[7] & 127 ;
        } else if(receive.id==257) {//2nd data packet right motor
    //                counter_2_right = receive.data[1] & 15;
            mode_right_receive = (receive.data[1] & 240)>>4;
     mc_status_right_receive = ( receive.data[3]<<8 ) + receive.data[2];
    //                max_power_right_receive = ((( receive.data[4]<<2 ) + ( receive.data[5] & 3))-128)*0.25;
    //                max_cornering_right_receive = ((((receive.data[5] & 252)>>2) + ((receive.data[6] & 15)<<6))-128)*0.25;
    //                ign_lock_position_right_receive = (receive.data[6]& 8)>>3;
    //                disconnect_hardware_right_receive = (receive.data[6]& 4)>>2;
            if(mode_right_receive ==14) {
                mode_right=0;
            } else (mode_right=1);
        } else if(receive.id==273) {//2nd data packet left motor
    //                counter_2_left = receive.data[1] & 15;
            mode_left_receive = (receive.data[1] & 240)>>4;
  mc_status_left_receive = ( receive.data[3]<<8 ) + receive.data[2];
    //                max_power_left_receive = ((( receive.data[4]<<2 ) + ( receive.data[5] & 3))-128)*0.25;
    //                max_cornering_left_receive = ((((receive.data[5] & 252)>>2) + ((receive.data[6] & 15)<<6))-128)*0.25;
    //                ign_lock_position_left_receive = (receive.data[6]& 8)>>3;
    //                disconnect_hardware_left_receive = (receive.data[6]& 4)>>2;
            if(mode_left_receive ==14) {
                mode_left=0;
            } else (mode_left=1);
        }  

        else if (receive.id == current_id){
            CURRENT_read =((receive.data[2]<<24) + (receive.data[3]<<16) + (receive.data[4]<<8) + receive.data[5])/1000;
            if(CURRENT_read>200)
            {
              CURRENT_read = 0;  
            } 
        else if (receive.id == voltage_id){
            VOLTAGE_read =((receive.data[2]<<24) + (receive.data[3]<<16) + (receive.data[4]<<8) + receive.data[5])/1000;
            if(VOLTAGE_read>450)
            {
              VOLTAGE_read = 0;  
            } 
        }

        }
    }
}
    





    


//==================================================Main code====================================
int main()
{
    ECU_error = 1;
    MC_error = 1;
    lvready=0;
    rtds_Sig = 0;
   
    pc.baud(9600);
    MC_can.frequency(500000);
    SN_can.frequency(1000000);
    led1 = 0;
    led2 = 0;
    led3 = 0;
    led4 = 0;
    
    mc.attach(&send,0.010);
    pc.baud(9600);

    SN_can.attach(&sensor_read, CAN::RxIrq);
    MC_can.attach(&mc_read, CAN::RxIrq);
    ignition_Sig.rise(&ignition_Sig_rise);
   gr_90_Sig.fall(&shut_down);
//    ignition_Sig.fall(&shut_down);

    t4.start();

    while(1) {


        //=============reading sensor values ==================


        APPS1_in = APPS1_read;
        APPS2_in = APPS2_read;
        BPS_in = BPS_read;
        STEER_in = STEER_read;
        time_apps =time_read;
        CURRENT_in = CURRENT_read;
        VOLTAGE_in = VOLTAGE_read;
        energy_time = t4.read();



       energy = (energy_prev + (VOLTAGE_in*CURRENT_in* (energy_time - energy_time_prev)))/1000000;

       energy_time_prev = energy_time ;

       time_apps_last = time_apps ;
        
   // ==========================================  (in a header file)==================  
        
      if(time_apps - time_apps_last > time_out){//delay of message
           APPS1_in = APPS1_min;
           APPS2_in = APPS2_min;
           STEER_in = STEER_mid;
       }

        if (APPS1_in <APPS1_gmin || APPS1_in>APPS1_gmax || APPS2_in<APPS2_gmin || APPS2_in>APPS2_gmax || BPS_in < BPS_gmin || BPS_in>BPS_gmax ) {//generating signal outside operating range
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
            if(BPS_in < BPS_min) {
                BPS_in = BPS_min;
            } else if(BPS_in > BPS_max) {
                BPS_in = BPS_max;
            }

            //conversion to percentage values

            APPS1_normalised = (APPS1_in - APPS1_min)*100.0/APPS1range;
            APPS2_normalised = (APPS2_in - APPS2_min)*100.0/APPS2range;

            APPS_avg = 1.0*(APPS1_normalised+APPS2_normalised)/2.0; // APPS_avg is normalised    

//====================================   brake light (in a header file)=================================================
            if(BPS_in>BPS_actuated)
                {brake_light_Sig=0;}
            else
                {brake_light_Sig=1;}
//====================================  led indications   (in a header file) ============================================================

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
//====================================  starting sequence     (in a header file) ====================================

  
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
            

//==================================Power Block determines max torque (in a header file)======================

           if( fixed_Torquemax* rpm_left_receive*2*3.14/60 > power_max || fixed_Torquemax* rpm_left_receive*2*3.14/60 > power_max ){
                if(rpm_left_receive > rpm_right_receive){
                   Torquemax = power_max*60 /(2*3.14*rpm_left_receive);

               }else if(rpm_right_receive > rpm_left_receive){
                   Torquemax = power_max*60 /(2*3.14*rpm_right_receive);
               }
            }else{
               Torquemax = fixed_Torquemax;
            }

            torque_threshold= 0.05*Torquemax;
            ediff_max = 0.12*Torquemax;

            torque_avg = 1.0*(Torquemax*APPS_avg)/100.0;

            torque_left_temp = torque_avg;
            torque_right_temp = torque_avg;


////==========================APPS AND BPS PLAUSIBILITY (in a header file)=============================

            if(apps_plaus_flag== 1 && abs(APPS1_normalised-APPS2_normalised)<10){//APPS Plausibility
                t1.stop();
                t1.reset();
                apps_plaus_flag= 0;
            }else if (abs(APPS1_normalised-APPS2_normalised)>10 ){
               if(apps_plaus_flag== 0){
                   t1.start();
                }
               apps_plaus_flag= 1;
               if(t1.read() > 0.1) {
                   torque_left_temp = 0;
                   torque_right_temp = 0;
                }
            }

            if(Voltage_left_receive>Voltage_right_receive) {
                power=CURRENT_in*Voltage_left_receive;
            } else {
                power=CURRENT_in*Voltage_right_receive;
            }

            if( APPS_avg<5 && power<5000 && bps_plaus_flag==1  ) { //BPS Plausibility with hard braking actuated
                t2.stop();
                t2.reset();
                torque_right_temp=0;
                torque_left_temp=0;
                bps_plaus_flag= 0;
                bps_plaus_flag2=0;

            }
            if(bps_plaus_flag2 ==1)
            {
                torque_right_temp=0;
                torque_left_temp=0;
            }
            if((APPS_avg>25 || power>5000) && BPS_in > BPS_actuated) { //BPS Plausibility with hard braking actuated
                if(bps_plaus_flag== 0) {
                    t2.start();
                  
                }
                bps_plaus_flag= 1;
                if(t2.read() > 0.5) {//making torque zero
                    torque_left_temp = 0;
                    torque_right_temp = 0;

                    bps_plaus_flag2=1;
                }
            }

            if (APPS_avg<5) {
                torque_left_temp = 0;
                torque_right_temp = 0;
            }


            torque_left = torque_left_temp;
            torque_right = torque_right_temp;


        }
    }
}
