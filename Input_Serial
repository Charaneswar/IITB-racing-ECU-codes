#include "mbed.h"
Serial pc(USBTX, USBRX); // tx, rx
double signal[3],torque_1,torque_2,d_1,d_2,d_3,d_avg,torque_right,torque_left,i;
const double signal_max_1=600,signal_min_1=300,signal_max_2=700,signal_min_2=400,signal_max_3=400,signal_min_3=200,torque_max_APPS=300,torque_max_steer=100,off_min=230,off_max=270;
char command1[3],command2[3],command3[3];
char* token;
int main()
{
    pc.printf("Enter the value of APPS Signal 1\n");
    pc.printf("Enter the value of APPS Signal 2\n");
    pc.printf("Enter the value of Steer Signal 3\n");
    while(1)
   {
    pc.scanf("%s",command1);
    pc.scanf("%s",command2);
    pc.scanf("%s",command3);
    token = strtok(command1,",;");
    signal[0]= atoi(token);
    token = strtok(command2,",;");
    signal[1]=atoi(token);
    token = strtok(command3,",;");
    signal[2]= atoi(token);
    d_1=((signal[0]-signal_min_1)/(signal_max_1-signal_min_1))*100;
    
    d_2=((signal[1]-signal_min_2)/(signal_max_2-signal_min_2))*100;
    
    d_avg=(d_1+d_2)/2;
    
    if(abs(d_1-d_2)<=10)
    {
    torque_1=(torque_max_APPS*d_avg)/100;
    }
    else
    {
    torque_1=0;    
    }
    d_3=((signal[2]-signal_min_3)/(signal_max_3-signal_min_3))*100;
    
    if(signal_min_3 - 30 < signal[2] && signal[2] < signal_max_3 + 30)
    
    {
        torque_2=(torque_max_steer*d_3)/100;
    }
    else
    {
        torque_2=0;   
    }
    
    if(off_min<=signal[2]<=off_max)
{
    torque_right=torque_1;
    torque_left=torque_1;
}
    else if(signal[2]<=off_min)
{
    torque_right=torque_1+torque_2;
    torque_left=torque_1-torque_2;
}
    else if(signal[2]>=off_max)
{
    torque_right=torque_1-torque_2;
    torque_left=torque_1+torque_2;
}
    pc.printf("The value of norm_1 is %f\n",d_1);    
}
}
