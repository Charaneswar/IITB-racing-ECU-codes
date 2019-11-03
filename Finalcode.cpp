#include "mbed.h"
#define apps_id 104
#define current_id 1313
#define voltage_id 1314
#define rpm_id 120
#define yaw_id 112
//======================code_combining_variables===========
char selection[25];
char flag_selection[25];
int calibration_flag=1;
char code_selection[25];
int code_flag=1;
char *token_s;
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
char voltage_send[8] = {0,0,0,0,0,0,0,0};
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
uint16_t APPS1_max = 850;
uint16_t APPS1_min = 330;
uint16_t APPS2_max = 830;
uint16_t APPS2_min = 310;
const uint16_t BPS_max = 630;
const uint16_t BPS_min = 120;
uint16_t STEER_left = 80;
uint16_t STEER_right = 915;
uint16_t STEER_mid =  570;
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

uint16_t APPS1range = APPS1_max - APPS1_min;
uint16_t APPS2range = APPS2_max - APPS2_min;
uint16_t STEERrange_left = abs(STEER_left-STEER_mid);
uint16_t STEERrange_right = abs(STEER_right-STEER_mid);



float APPS1_normalised;
float APPS2_normalised;
float APPS_avg;
float STEER_normalised;
float STEER_normalised_left;//With respect to steer mid
float STEER_normalised_right;// with respected steer rejected
float STEER_displacement =0;

//====================Input sensor values=================
uint16_t APPS1_in=0;
uint16_t APPS2_in=0;
uint16_t BPS_in=0;
uint16_t STEER_in=0;
uint16_t CURRENT_in;
uint32_t VOLTAGE_in;
uint32_t RPM_in;
double YAW_in;

uint32_t time_apps = 0;
uint32_t time_apps_last=0;
uint32_t rpm_right_in;
uint32_t rpm_left_in;

//===============================Plausibility and SCS checking variables====================

uint32_t time_out = 100; // maximum idle time allowed in ms
int apps_plaus_flag= 0;
int bps_plaus_flag= 0;
int bps_plaus_flag2 = 0;
int power;

Timer t1,t2,t3,t4,t5;//t1 for APPS plausibility and t2 for BPS plausibility

//=======================================================Ediff variables==================================

uint16_t STEER_rejected = 30;//for incoming value without normalization
float STEER_rejected_left = STEER_rejected *256/STEERrange_left;
float STEER_rejected_right =STEER_rejected*256/STEERrange_right;
const float kp = 2;
const float ki= 1.8;
double error_current;
double error_prev;
char yaw;

int flag_ediff = 0;
int t;
int tprev;

const double imax=0.0301;
const double imin=0;
double integral;
double integral_prev;
const double tyre_radius =0.22;
double velocity;

int torque_ediff =0;
//float ediff_max =  0.12*Torquemax;//allowed maximum absolute voltage to be added above avg torque
int ediff_max =4;

float torque_left_temp=0;
float torque_right_temp=0;

LocalFileSystem local("local");
LocalFileSystem local1("local1");
LocalFileSystem local2("local2");
LocalFileSystem local3("local3");
LocalFileSystem local4("local4");
LocalFileSystem local5("local5");
LocalFileSystem local6("local6");
LocalFileSystem local7("local7");
LocalFileSystem local8("local8");
LocalFileSystem local9("local9");
LocalFileSystem local10("local0");

char yaw_rate[600];
char  yaw_rate_temp[25];
double rpm_time=0;
double rpm_time_last=0;
int rpm_flag=0;
int rpm_counter_left;
int rpm_counter_left_last;
int rpm_counter_right;
int rpm_counter_right_last;
const int rpm_teeth =12;
//==============================================Power block variables================================

int Torquemax = 30;
double Torquemax_temp=0;
int fixed_Torquemax = 30;
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

int ceil_steer =0;
int ceil_velocity =0;
int floor_steer =0;
int floor_velocity =0;


double node1=0;

double node2=0;

double node3=0;
double node4=0;

double yaw_rate_1;

double yaw_rate_2;

double yaw_rate_3;

double yaw_rate_4;


double N1,N2,N3,N4;

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
    wait(0.010);
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
    if(SN_can.read(receive) && receive.id == apps_id) {

        APPS1_read  = (receive.data[0] << 2) | (receive.data[4] & 3);
        APPS2_read = (receive.data[1] << 2) | ((receive.data[4] >>2) & 3);
        BPS_read   = (receive.data[2] << 2) | ((receive.data[4] >>4) & 3);
        STEER_read  = (receive.data[3] << 2) | ((receive.data[4] >>6) & 3);
        time_read = (receive.data[7] << 16) | (receive.data[6] << 8) | receive.data[5];
    }


    else if(SN_can.read(receive) && receive.id == rpm_id) {

        rpm_left_in = receive.data[0]*255 + receive.data[1];

        rpm_right_in = receive.data[2]*255 + receive.data[3];

        rpm_time = t4.read();
        if(rpm_flag <2 ) {
            rpm_flag = rpm_flag+1;
        }

    } else if(SN_can.read(receive) && receive.id == yaw_id) {

        YAW_in = (receive.data[1]<<8 + receive.data[0])*0.005  -163.8375;


    }
}

void mc_read()
{
    if(MC_can.read(receive)) {
        if(receive.id==256) {//1st data packet right motor

            //              counter_1_right = receive.data[1] & 15;
            rpm_right_receive = ((((224) & receive.data[2])>>5)+(receive.data[3]<<3)+ ((receive.data[4] & (3))<<11))*3 -12288;
            Voltage_right_receive = (((receive.data[4] & 252)>>2) + ((receive.data[5] & 15)<<6));
            // torque_right_receive = ((((31) & receive.data[2])<<4)+((receive.data[1]& (240))>>4))+ (-256);
            //              power_right_receive = ((receive.data[5] & 240)>>4) + ((receive.data[6] & 63)<<4);
            //              efficiency_right_receive = receive.data[7] & 127 ;
        } else if(receive.id==272) {//1st data packet left motor
            //              counter_1_left = receive.data[1] & 15;
            rpm_left_receive = ((((224) & receive.data[2])>>5)+(receive.data[3]<<3)+ ((receive.data[4] & (3))<<11))*3 -12288;
            Voltage_left_receive = (((receive.data[4] & 252)>>2) + ((receive.data[5] & 15)<<6));
            // torque_left_receive = ((((31) & receive.data[2])<<4)+((receive.data[1]& (240))>>4))+ (-256);
            //              power_left_receive = ((receive.data[5] & 240)>>4) + ((receive.data[6] & 63)<<4);
            //              efficiency_left_receive = receive.data[7] & 127 ;
        } else if(receive.id==257) {//2nd data packet right motor
            //                counter_2_right = receive.data[1] & 15;
            mode_right_receive = (receive.data[1] & 240)>>4;
            // mc_status_right_receive = ( receive.data[3]<<8 ) + receive.data[2];
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
            // mc_status_left_receive = ( receive.data[3]<<8 ) + receive.data[2];
            //                max_power_left_receive = ((( receive.data[4]<<2 ) + ( receive.data[5] & 3))-128)*0.25;
            //                max_cornering_left_receive = ((((receive.data[5] & 252)>>2) + ((receive.data[6] & 15)<<6))-128)*0.25;
            //                ign_lock_position_left_receive = (receive.data[6]& 8)>>3;
            //                disconnect_hardware_left_receive = (receive.data[6]& 4)>>2;
            if(mode_left_receive ==14) {
                mode_left=0;
            } else (mode_left=1);
        }

        else if (receive.id == current_id) {
            CURRENT_read =((receive.data[2]<<24) + (receive.data[3]<<16) + (receive.data[4]<<8) + receive.data[5])/1000;
            if(CURRENT_read>200) {
                CURRENT_read = 0;
            } else if (receive.id == voltage_id) {
                VOLTAGE_read =((receive.data[2]<<24) + (receive.data[3]<<16) + (receive.data[4]<<8) + receive.data[5])/1000;
                if(VOLTAGE_read>450) {
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
    brake_light_Sig=0;

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



    FILE *fp = fopen("/local/flag.txt", "r");
    fscanf(fp,"%s",flag_selection );
    token_s = strtok(flag_selection,",;");
    calibration_flag= atoi(token_s);
    Torquemax_temp =calibration_flag/10;

    calibration_flag = calibration_flag - (10*floor(Torquemax_temp));
    code_flag = floor(Torquemax_temp);

    fclose(fp);

///=======================calibration============================

    if(calibration_flag ==1) {

        FILE *fp17 = fopen("/local1/apps1_max.txt", "r");
        fscanf(fp17,"%s",code_selection );
        token_s = strtok(code_selection,",;");
        APPS1_max= atoi(token_s);
        fclose(fp17);

        FILE *fp18 = fopen("/local2/apps1_min.txt", "r");
        fscanf(fp18,"%s",code_selection );
        token_s = strtok(code_selection,",;");
        APPS1_min= atoi(token_s);
        fclose(fp18);

        FILE *fp19 = fopen("/local3/apps2_max.txt", "r");
        fscanf(fp19,"%s",code_selection );
        token_s = strtok(code_selection,",;");
        APPS2_max= atoi(token_s);
        fclose(fp19);

        FILE *fp20 = fopen("/local4/apps2_min.txt", "r");
        fscanf(fp20,"%s",code_selection );
        token_s = strtok(code_selection,",;");
        APPS2_min= atoi(token_s);
        fclose(fp20);

        FILE *fp21 = fopen("/local5/steer_left.txt", "r");
        fscanf(fp21,"%s",code_selection );
        token_s = strtok(code_selection,",;");
        STEER_left= atoi(token_s);
        fclose(fp21);


        FILE *fp22 = fopen("/local6/steer_right.txt", "r");
        fscanf(fp21,"%s",code_selection );
        token_s = strtok(code_selection,",;");
        STEER_right= atoi(token_s);
        fclose(fp21);


        FILE *fp23 = fopen("/local7/steer_mid.txt", "r");
        fscanf(fp23,"%s",code_selection );
        token_s = strtok(code_selection,",;");
        STEER_mid= atoi(token_s);
        fclose(fp23);



        FILE *fp26 = fopen("/local8/torque.txt", "r");
        fscanf(fp26,"%s",code_selection );
        token_s = strtok(code_selection,",;");
        Torquemax= atoi(token_s);

        Torquemax_temp =Torquemax/10;
        ediff_max = Torquemax - (10*floor(Torquemax_temp));
        Torquemax = floor(Torquemax_temp);
        fixed_Torquemax =Torquemax;
        fclose(fp26);






    }
    if (calibration_flag ==0) {

//============apps1 max=======
        while(!(pc.readable())) {
            APPS1_in = APPS1_read;
            pc.printf("%d \n",APPS1_in );
            pc.printf("apps1_max \n");
        } // read input from pc
        pc.gets(selection,25);
        token_s = strtok(selection,",;");
        APPS1_max= atoi(token_s);
        FILE *fp1 = fopen("/local1/apps1_max.txt", "w");
        fprintf(fp1,"%d",APPS1_max);
        fclose(fp1);



//==============apps1 min=========================


        while(!(pc.readable())) {
            APPS1_in = APPS1_read;
            pc.printf("%d \n",APPS1_in );
            pc.printf("apps1_min \n");
        } // read input from pc
        pc.gets(selection,25);
        token_s = strtok(selection,",;");
        APPS1_min= atoi(token_s);
        FILE *fp2 = fopen("/local2/apps1_min.txt", "w");
        fprintf(fp2,"%d",APPS1_min);
        fclose(fp2);

//==============apps2 max===================================


        while(!(pc.readable())) {
            APPS2_in = APPS2_read;
            pc.printf("%d \n",APPS2_in );
            pc.printf("apps2_max \n");
        } // read input from pc
        pc.gets(selection,25);
        token_s = strtok(selection,",;");
        APPS2_max = atoi(token_s);
        FILE *fp3 = fopen("/local3/apps2_max.txt", "w");
        fprintf(fp3,"%d",APPS2_max);
        fclose(fp3);

//=============apps2 min================================

        while(!(pc.readable())) {
            APPS2_in = APPS2_read;
            pc.printf("%d \n",APPS2_in );
            pc.printf("apps2_min \n");
        } // read input from pc
        pc.gets(selection,25);
        token_s = strtok(selection,",;");
        APPS2_min = atoi(token_s);
        FILE *fp4 = fopen("/local4/apps2_min.txt", "w");
        fprintf(fp4,"%d",APPS2_min);
        fclose(fp4);

//=========steer left=================================
        while(!(pc.readable())) {
            STEER_in = STEER_read;
            pc.printf("%d \n",STEER_in );
            pc.printf("steer left \n");
        } // read input from pc
        pc.gets(selection,25);
        token_s = strtok(selection,",;");
        STEER_left = atoi(token_s);
        FILE *fp5 = fopen("/local5/steer_left.txt", "w");
        fprintf(fp5,"%d",STEER_left);
        fclose(fp5);


//========steer right==================================

        while(!(pc.readable())) {
            STEER_in = STEER_read;
            pc.printf("%d \n",STEER_in );
            pc.printf("Steer right \n");
        } // read input from pc
        pc.gets(selection,25);
        token_s = strtok(selection,",;");
        STEER_right = atoi(token_s);
        FILE *fp6 = fopen("/local6/steer_right.txt", "w");
        fprintf(fp6,"%d",STEER_right);
        fclose(fp6);

//=========steer mid======================================

        while(!(pc.readable())) {
            STEER_in = STEER_read;
            pc.printf("%d \n",STEER_in );
            pc.printf("steer mid \n");
        } // read input from pc
        pc.gets(selection,25);
        token_s = strtok(selection,",;");
        STEER_mid = atoi(token_s);
        FILE *fp7 = fopen("/local7/steer_mid.txt", "w");
        fprintf(fp7,"%d",STEER_mid);
        fclose(fp7);


//================torque============
        pc.printf("torque \n");
        while(!(pc.readable())) ;
        pc.gets(selection,25);
        token_s = strtok(selection,",;");
        fixed_Torquemax = atoi(token_s);
        Torquemax = atoi(token_s);
        FILE *fp10 = fopen("/local8/torque.txt", "w");
        fprintf(fp10,"%d",Torquemax);
        fclose(fp10);
        Torquemax_temp = Torquemax/10;
        ediff_max = Torquemax - (10*(floor(Torquemax_temp)));
        Torquemax = floor(Torquemax_temp);
        fixed_Torquemax =Torquemax;



        calibration_flag =1;
        calibration_flag=(code_flag*10)+calibration_flag;
        FILE *fp13 = fopen("/local/flag.txt", "w");
        fprintf(fp13,"%d",calibration_flag);
        fclose(fp13);

    }

//=========================================
    if(calibration_flag ==2) {

        //================torque============

        while(!(pc.readable())) {
            pc.printf("torque \n");
            pc.printf(" calibration done already \n");
        }
        pc.gets(selection,25);
        token_s = strtok(selection,",;");
        Torquemax = atoi(token_s);
        FILE *fp14 = fopen("/local8/torque.txt", "w");
        fprintf(fp14,"%d",Torquemax);
        fclose(fp14);
        Torquemax_temp = Torquemax/10;
        ediff_max = Torquemax - (10*(floor(Torquemax_temp)));
        Torquemax = floor(Torquemax_temp);
        fixed_Torquemax =Torquemax;



        calibration_flag =1;
        calibration_flag=(code_flag*10)+calibration_flag;
        FILE *fp34 = fopen("/local/flag.txt", "w");
        fprintf(fp34,"%d",calibration_flag);
        fclose(fp34);

    }





/////===========================close loop ediff====================================





    if(code_flag ==3) {
        STEER_rejected_left = STEER_rejected *256/STEERrange_left;
        STEER_rejected_right =STEER_rejected*256/STEERrange_right;
        APPS1range = APPS1_max - APPS1_min;
        APPS2range = APPS2_max - APPS2_min;
        STEERrange_left = abs(STEER_left-STEER_mid);
        STEERrange_right = abs(STEER_right-STEER_mid);
//        ECU_error = 1;
//        MC_error = 1;
//        lvready=0;
//        rtds_Sig = 0;
//        brake_light_Sig=0;
//
//        pc.baud(9600);
//        MC_can.frequency(500000);
//        SN_can.frequency(1000000);
//        led1 = 0;
//        led2 = 0;
//        led3 = 0;
//        led4 = 0;
//
//        mc.attach(&send,0.010);
//        pc.baud(9600);
//
//        SN_can.attach(&sensor_read, CAN::RxIrq);
//        MC_can.attach(&mc_read, CAN::RxIrq);
        ignition_Sig.rise(&ignition_Sig_rise);
        gr_90_Sig.fall(&shut_down);
//    ignition_Sig.fall(&shut_down);
        t4.start();
        t5.start();


//==============reading yaw rate value from look up table=======================

        char *token;




//    fclose(fp);

        while(1) {


            //=============reading sensor values============

            pc.printf("\n code3 \n");
            APPS1_in = APPS1_read;
            APPS2_in = APPS2_read;
            BPS_in = BPS_read;
            STEER_in = STEER_read;
            time_apps =time_read;
            CURRENT_in = CURRENT_read;
            VOLTAGE_in = VOLTAGE_read;
            energy_time = t5.read();
            rpm_counter_right = rpm_right_in;
            rpm_counter_left = rpm_left_in;

//            if(time_apps - time_apps_last > time_out) { //delay of message
//                APPS1_in = APPS1_min;
//                APPS2_in = APPS2_min;
//                STEER_in = STEER_mid;
//            }
            energy = (energy_prev + (VOLTAGE_in*CURRENT_in* (energy_time - energy_time_prev)))/1000000;

            energy_time_prev = energy_time ;
   pc.printf("APPS1=%d \t apps2=%d \t APPS1_min=%d \t APPS2_min=%d \t APPS1_max=%d \t APPS2_max=%d \t APPS1range=%d \t APPS2range=%d \t torquemax=%d \n ",APPS1_in,APPS2_in,APPS1_min,APPS2_min,APPS1_max,APPS2_max,APPS1range,APPS2range,Torquemax);



            time_apps_last = time_apps ;
//if(false){
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
              

//====================================   brake light =================================================
                if(BPS_in>BPS_actuated) {
                    brake_light_Sig=0;
                } else {
                    brake_light_Sig=1;
                }
//=====================================  led indications    ============================================================

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


//==================================Power Block determines max torque======================

                if( fixed_Torquemax* rpm_left_receive*2*3.14/60 > power_max || fixed_Torquemax* rpm_left_receive*2*3.14/60 > power_max ) {
                    if(rpm_left_receive > rpm_right_receive) {
                        Torquemax = power_max*60 /(2*3.14*rpm_left_receive);

                    } else if(rpm_right_receive > rpm_left_receive) {
                        Torquemax = power_max*60 /(2*3.14*rpm_right_receive);
                    }
                } else {
                    Torquemax = fixed_Torquemax;
                }

                torque_threshold= 0.05*Torquemax;
                ediff_max = 0.12*Torquemax;

                torque_avg = 1.0*(Torquemax*APPS_avg)/100.0;

                torque_left_temp = torque_avg;
                torque_right_temp = torque_avg;


//=================================================Steering=========================

                if (STEER_in >STEER_gmax || STEER_in < STEER_gmin) {
                    torque_ediff=0;
                } else {
                    //adjusting steer values to stay in range
                    if(STEER_in < STEER_left) {
                        STEER_in = STEER_left;

                    } else if(STEER_in > STEER_right) {
                        STEER_in = STEER_right;
                    }

//================================ ediff  ========================================
                    STEER_displacement = (( -0.00006708 * STEER_in*STEER_in) + (  -0.03057*STEER_in)+   92.45)-44 ;
                    if(rpm_flag ==2) {
                        if(rpm_counter_left > rpm_counter_left_last && rpm_counter_right > rpm_counter_right_last) {
                            rpm_left = rpm_counter_left - rpm_counter_left_last;
                            rpm_right = rpm_counter_right - rpm_counter_right_last;


                        }
                        if(rpm_counter_left < rpm_counter_left_last && rpm_counter_right > rpm_counter_right_last) {
                            rpm_right = rpm_counter_right - rpm_counter_right_last;
                            rpm_left = 10000 - rpm_counter_left + rpm_counter_left_last;


                        }
                        if(rpm_counter_left > rpm_counter_left_last && rpm_counter_right < rpm_counter_right_last) {
                            rpm_right = 10000 - rpm_counter_right + rpm_counter_right_last;
                            rpm_left = rpm_counter_left - rpm_counter_left_last;
                        }
                        if(rpm_counter_left < rpm_counter_left_last && rpm_counter_right < rpm_counter_right_last) {
                            rpm_right = 10000 - rpm_counter_right + rpm_counter_right_last;
                            rpm_left = 10000 - rpm_counter_left + rpm_counter_left_last;

                        }

                        RPM_in = 1.0*(rpm_left +rpm_right)*60/(1.0*(rpm_time - rpm_time_last)*2*rpm_teeth);

                    }


                    if(rpm_flag < 2) {
                        RPM_in = 0;

                    }
                    rpm_time_last = rpm_time;
                    rpm_counter_left_last = rpm_counter_left;
                    rpm_counter_right_last = rpm_counter_right;


                    velocity =  tyre_radius* RPM_in*0.10472;

                    ceil_steer = ceil(abs(STEER_displacement));
                    floor_steer =floor(abs(STEER_displacement));
                    ceil_velocity =ceil(velocity );
                    floor_velocity =floor(velocity );
                    if(ceil_steer %2 != 0) {
                        ceil_steer = ceil_steer + 1;

                    }
                    if(floor_steer %2 != 0) {
                        floor_steer = floor_steer -1;
                    }
                    if(ceil_steer == floor_steer) {
                        floor_steer = floor_steer + 2;
                    }


                    node1 = floor_velocity + (floor_steer*25/2) -1;
                    node2 = ceil_velocity + (floor_steer*25/2) -1;
                    node3 = ceil_velocity + (ceil_steer*25/2) -1;
                    node4 = floor_velocity + (ceil_steer*25/2) -1;

                    N1 = (abs(STEER_displacement) - ceil_steer)*(velocity - ceil_velocity);
                    N2 = -1*(abs(STEER_displacement) - ceil_steer)*(velocity - floor_velocity);
                    N3 = (abs(STEER_displacement) - floor_steer)*(velocity - floor_velocity);
                    N4 = -1*(abs(STEER_displacement) - floor_steer)*(velocity - ceil_velocity);

                    FILE *fp30 = fopen("/local9/yaw.txt", "r");

                    for(int j=0; j<600; j++) {

                        fscanf(fp30,"%s",yaw_rate_temp );

                        if(j == node1) {
                            token = strtok(yaw_rate_temp,",;");
                            yaw_rate_1 = atoi(token);
                            yaw_rate_1 = yaw_rate_1 /100000000;
                        } else if(j == node2) {
                            token = strtok(yaw_rate_temp,",;");
                            yaw_rate_2 = atoi(token);
                            yaw_rate_2 = yaw_rate_2 /100000000;
                        } else if(j == node3) {
                            token = strtok(yaw_rate_temp,",;");
                            yaw_rate_3 = atoi(token);
                            yaw_rate_3 = yaw_rate_3 /100000000;
                        } else if(j == node4) {
                            token = strtok(yaw_rate_temp,",;");
                            yaw_rate_4 = atoi(token);
                            yaw_rate_4 = yaw_rate_4 /100000000;
                        }

                    }
                    fclose(fp30);


                    yaw =( (yaw_rate_1)*N1 + (yaw_rate_2)*N2 + (yaw_rate_3*N3) +( yaw_rate_4*N4))/2 ;


                    if(STEER_displacement < 0 ) {
                        yaw = -1*yaw;
                    }


                    error_current = YAW_in - yaw;





//======================================PI implementation ====================================================

                    if(flag_ediff == 1) {
                        if (error_prev !=0 && error_current !=0) {
                            if(error_prev/abs(error_prev)!= error_current/abs(error_current)) {
                                integral=0;
                                t=t3.read();
                                tprev=t;
                                integral_prev = integral;
                            }
                        } else if(error_current == 0) {
                            integral=0;
                            t=t3.read();
                            tprev=t;
                            integral_prev = integral;

                        } else {
                            t=t3.read();
                            integral = integral_prev + error_current*(t-tprev);
                            tprev = t;
                            integral_prev = integral;
                        }
                    }

                    if(APPS_avg >5 && flag_ediff ==0) {
                        t3.start();
                        tprev=t3.read();
                        error_prev = error_current;
                        integral_prev = 0;
                        flag_ediff = 1;
                    }

                    if(integral>imin && integral<imax) {
                        torque_ediff=kp*error_current+ ki*integral;
                    } else if (integral<imin) {
                        torque_ediff=kp*error_current ;
                    } else {
                        torque_ediff=kp*error_current + ki*imax;
                    }



                    torque_left_temp = torque_avg + torque_ediff;
                    torque_right_temp = torque_avg - torque_ediff;

//===========================setting torque value========================================

                    if(abs(torque_ediff)>ediff_max) {//checking ediff max condition
                        torque_ediff = ediff_max*torque_ediff/abs(torque_ediff);
                    }

                    if (torque_left_temp < 0) {
                        torque_left_temp = torque_threshold;
                        torque_right_temp  = 2*torque_avg-torque_threshold;

                    } else if (torque_right_temp < 0) {
                        torque_right_temp = torque_threshold;
                        torque_left_temp  = 2*torque_avg-torque_threshold;

                    } else if (torque_left_temp > Torquemax) {
                        torque_left_temp = Torquemax;
                        torque_right_temp = 2*torque_avg-Torquemax;

                    } else if (torque_right_temp > Torquemax) {
                        torque_right_temp = Torquemax;
                        torque_left_temp = 2*torque_avg-Torquemax;
                    }
                }



////=============APPS AND BPS PLAUSIBILITY=============================

                if(apps_plaus_flag== 1 && abs(APPS1_normalised-APPS2_normalised)<10) { //APPS Plausibility
                    t1.stop();
                    t1.reset();
                    apps_plaus_flag= 0;
                } else if (abs(APPS1_normalised-APPS2_normalised)>10 ) {
                    if(apps_plaus_flag== 0) {
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
                if(bps_plaus_flag2 ==1) {
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
                pc.printf("ap_f=%d \t, bp_flag =%d \n",apps_plaus_flag,bps_plaus_flag);
                pc.printf("m_r = %d \t m_l = %d \t V_r = %d \t V_l = %d \t rpm_r = %.1f \t rpm_l= %.1f \t T_r = %.1f \t T_l = %.1f \t bps = %.1d \t apps = %.1f \t torqueleft_temp = %.1f \t torqueright_temp = %.1f \n", mode_right_receive, mode_left_receive, Voltage_right_receive, Voltage_left_receive, rpm_right_receive, rpm_left_receive, torque_right_receive, torque_left_receive,BPS_in,APPS_avg,torque_left_temp,torque_right_temp);
                pc.printf("s_l = %d \t s_r = %d \t current = %d \n",  mc_status_left_receive,  mc_status_right_receive, CURRENT_in);



            }
        }
    }

///////////////////////////////////==============================================acceleration_run=========================================



    if(code_flag == 1) {
        STEER_rejected_left = STEER_rejected *256/STEERrange_left;
        STEER_rejected_right =STEER_rejected*256/STEERrange_right;
        APPS1range = APPS1_max - APPS1_min;
        APPS2range = APPS2_max - APPS2_min;
        STEERrange_left = abs(STEER_left-STEER_mid);
        STEERrange_right = abs(STEER_right-STEER_mid);
        //      ECU_error = 1;
//        MC_error = 1;
//        lvready=0;
//        rtds_Sig = 0;
//
//        pc.baud(9600);
//        MC_can.frequency(500000);
//        SN_can.frequency(1000000);
//        led1 = 0;
//        led2 = 0;
//        led3 = 0;
//        led4 = 0;
//
//        mc.attach(&send,0.010);
//        pc.baud(9600);
//
//        SN_can.attach(&sensor_read, CAN::RxIrq);
//        MC_can.attach(&mc_read, CAN::RxIrq);
        ignition_Sig.rise(&ignition_Sig_rise);
        gr_90_Sig.fall(&shut_down);
//    ignition_Sig.fall(&shut_down);

        t4.start();

        while(1) {


            //=============reading sensor values============

            pc.printf("\n code1 \n");
            APPS1_in = APPS1_read;
            APPS2_in = APPS2_read;
            BPS_in = BPS_read;
            STEER_in = STEER_read;
            time_apps =time_read;
            CURRENT_in = CURRENT_read;
            VOLTAGE_in = VOLTAGE_read;
            energy_time = t4.read();







//            if(time_apps - time_apps_last > time_out) { //delay of message
//                APPS1_in = APPS1_min;
//                APPS2_in = APPS2_min;
//                STEER_in = STEER_mid;
//            }

            pc.printf("APPS1=%d \t apps2=%d \t APPS1_min=%d \t APPS2_min=%d \t APPS1_max=%d \t APPS2_max=%d \t APPS1range=%d \t APPS2range=%d \t torquemax=%d \n ",APPS1_in,APPS2_in,APPS1_min,APPS2_min,APPS1_max,APPS2_max,APPS1range,APPS2range,Torquemax);


            energy = (energy_prev + (VOLTAGE_in*CURRENT_in* (energy_time - energy_time_prev)))/1000000;

            energy_time_prev = energy_time ;

            time_apps_last = time_apps ;


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

//====================================   brake light =================================================
                if(BPS_in>BPS_actuated) {
                    brake_light_Sig=0;
                } else {
                    brake_light_Sig=1;
                }
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


//==================================Power Block determines max torque======================

                if( fixed_Torquemax* rpm_left_receive*2*3.14/60 > power_max || fixed_Torquemax* rpm_left_receive*2*3.14/60 > power_max ) {
                    if(rpm_left_receive > rpm_right_receive) {
                        Torquemax = power_max*60 /(2*3.14*rpm_left_receive);

                    } else if(rpm_right_receive > rpm_left_receive) {
                        Torquemax = power_max*60 /(2*3.14*rpm_right_receive);
                    }
                } else {
                    Torquemax = fixed_Torquemax;
                }

                torque_threshold= 0.05*Torquemax;


                torque_avg = 1.0*(Torquemax*APPS_avg)/100.0;

                torque_left_temp = torque_avg;
                torque_right_temp = torque_avg;


////=============APPS AND BPS PLAUSIBILITY=============================

                if(apps_plaus_flag== 1 && abs(APPS1_normalised-APPS2_normalised)<10) { //APPS Plausibility
                    t1.stop();
                    t1.reset();
                    apps_plaus_flag= 0;
                } else if (abs(APPS1_normalised-APPS2_normalised)>10 ) {
                    if(apps_plaus_flag== 0) {
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
                if(bps_plaus_flag2 ==1) {
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
                pc.printf("ap_f=%d \t, bp_flag =%d \n",apps_plaus_flag,bps_plaus_flag);
                pc.printf("m_r = %d \t m_l = %d \t V_r = %d \t V_l = %d \t rpm_r = %.1f \t rpm_l= %.1f \t T_r = %.1f \t T_l = %.1f \t bps = %.1d \t apps = %.1f \t torqueleft_temp = %.1f \t torqueright_temp = %.1f \n", mode_right_receive, mode_left_receive, Voltage_right_receive, Voltage_left_receive, rpm_right_receive, rpm_left_receive, torque_right_receive, torque_left_receive,BPS_in,APPS_avg,torque_left_temp,torque_right_temp);
                pc.printf("s_l = %d \t s_r = %d \t current = %d \n",  mc_status_left_receive,  mc_status_right_receive, CURRENT_in);


            }

        }

    }


///////////======================================open loop ediff=====================================================================
    if(code_flag == 2) {
        STEER_rejected_left = STEER_rejected *256/STEERrange_left;
        STEER_rejected_right =STEER_rejected*256/STEERrange_right;
        APPS1range = APPS1_max - APPS1_min;
        APPS2range = APPS2_max - APPS2_min;
        STEERrange_left = abs(STEER_left-STEER_mid);
        STEERrange_right = abs(STEER_right-STEER_mid);
//       ECU_error = 1;
//        MC_error = 1;
//        lvready=0;
//        rtds_Sig = 0;
//
//        pc.baud(9600);
//        MC_can.frequency(500000);
//        SN_can.frequency(1000000);
//        led1 = 0;
//        led2 = 0;
//        led3 = 0;
//        led4 = 0;
//        brake_light_Sig=1;
//
//        mc.attach(&send,0.010);
//        pc.baud(9600);
//
//        SN_can.attach(&sensor_read, CAN::RxIrq);
//        MC_can.attach(&mc_read, CAN::RxIrq);
        ignition_Sig.rise(&ignition_Sig_rise);
        gr_90_Sig.fall(&shut_down);
        ignition_Sig.fall(&shut_down);
        t4.start();


        while(1) {


            //=============reading sensor values============
            pc.printf("\n code2 \n");

            APPS1_in = APPS1_read;
            APPS2_in = APPS2_read;
            BPS_in = BPS_read;
            STEER_in = STEER_read;
            time_apps =time_read;
            CURRENT_in = CURRENT_read;
            VOLTAGE_in = VOLTAGE_read;
            energy_time = t4.read();

//            if(time_apps - time_apps_last > time_out) { //delay of message
//                APPS1_in = APPS1_min;
//                APPS2_in = APPS2_min;
//                STEER_in = STEER_mid;
//            }
            energy = (energy_prev + (VOLTAGE_in*CURRENT_in* (energy_time - energy_time_prev)))/1000000;

            energy_time_prev = energy_time ;


            time_apps_last = time_apps ;

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

//====================================   brake light =================================================
                if(BPS_in>BPS_actuated) {
                    brake_light_Sig=0;
                } else {
                    brake_light_Sig=1;
                }
//=====================================  led indications    ============================================================

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


//==================================Power Block determines max torque======================

                if( fixed_Torquemax* rpm_left_receive*2*3.14/60 > power_max || fixed_Torquemax* rpm_left_receive*2*3.14/60 > power_max ) {
                    if(rpm_left_receive > rpm_right_receive) {
                        Torquemax = power_max*60 /(2*3.14*rpm_left_receive);

                    } else if(rpm_right_receive > rpm_left_receive) {
                        Torquemax = power_max*60 /(2*3.14*rpm_right_receive);
                    }
                } else {
                    Torquemax = fixed_Torquemax;
                }

                torque_threshold= 0.05*Torquemax;
                ediff_max = 0.12*Torquemax;

                torque_avg = 1.0*(Torquemax*APPS_avg)/100.0;

                torque_left_temp = torque_avg;
                torque_right_temp = torque_avg;



//=================================================Steering=========================

                if (STEER_in >STEER_gmax || STEER_in < STEER_gmin) {
                    torque_ediff=0;
                } else {
                    //adjusting steer values to stay in range
                    if(STEER_in < STEER_left) {
                        STEER_in = STEER_left;

                    } else if(STEER_in > STEER_right) {
                        STEER_in = STEER_right;
                    }

//================================Basic ediff estabilishment ========================================

                    STEER_normalised_left = (STEER_in - STEER_mid)*256.0/STEERrange_right;
                    STEER_normalised_right = (STEER_in - STEER_mid)*256.0/STEERrange_left;
                    if((abs(STEER_in) < STEER_rejected)) {
                        torque_ediff=0;//as steer stays in allowed mid steer range or the steer value is out of allowed range
                    } else {
                        if(STEER_in > STEER_mid) {//turning right
                            STEER_normalised = STEER_normalised_right - STEER_rejected_right;
                            torque_ediff = kp*STEER_normalised*ediff_max;

                        } else if(STEER_in < STEER_mid) { //turning left
                            STEER_normalised = STEER_normalised_left + STEER_rejected_left;
                            torque_ediff =-1* kp*STEER_normalised*ediff_max;
                        }

                        if(abs(torque_ediff)>ediff_max) {//checking ediff max condition
                            torque_ediff = ediff_max*torque_ediff/abs(torque_ediff);
                        }
                    }



                    torque_left_temp = torque_avg + torque_ediff;
                    torque_right_temp = torque_avg - torque_ediff;

//===========================setting torque value========================================


                    if (torque_left_temp < torque_threshold) {
                        torque_left_temp = torque_threshold;
                        torque_right_temp  = 2*torque_avg-torque_threshold;

                    } else if (torque_right_temp < torque_threshold) {
                        torque_right_temp = torque_threshold;
                        torque_left_temp  = 2*torque_avg-torque_threshold;

                    } else if (torque_left_temp > Torquemax) {
                        torque_left_temp = Torquemax;
                        torque_right_temp = 2*torque_avg-Torquemax;

                    } else if (torque_right_temp > Torquemax) {
                        torque_right_temp = Torquemax;
                        torque_left_temp = 2*torque_avg-Torquemax;
                    }
                }



////=============APPS AND BPS PLAUSIBILITY=============================

                if(apps_plaus_flag== 1 && abs(APPS1_normalised-APPS2_normalised)<10) { //APPS Plausibility
                    t1.stop();
                    t1.reset();
                    apps_plaus_flag= 0;
                } else if (abs(APPS1_normalised-APPS2_normalised)>10 ) {
                    if(apps_plaus_flag== 0) {
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
                if(bps_plaus_flag2 ==1) {
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
                pc.printf("ap_f=%d \t, bp_flag =%d \n",apps_plaus_flag,bps_plaus_flag);
                pc.printf("m_r = %d \t m_l = %d \t V_r = %d \t V_l = %d \t rpm_r = %.1f \t rpm_l= %.1f \t T_r = %.1f \t T_l = %.1f \t bps = %.1d \t apps = %.1f \t torqueleft_temp = %.1f \t torqueright_temp = %.1f \n", mode_right_receive, mode_left_receive, Voltage_right_receive, Voltage_left_receive, rpm_right_receive, rpm_left_receive, torque_right_receive, torque_left_receive,BPS_in,APPS_avg,torque_left_temp,torque_right_temp);
                pc.printf("s_l = %d \t s_r = %d \t current = %d \n",  mc_status_left_receive,  mc_status_right_receive, CURRENT_in);
            }
        }
    }
}
