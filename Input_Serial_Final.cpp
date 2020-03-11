#include "mbed.h"
Serial pc(USBTX, USBRX); // tx, rx
Timer timer,t2;
LocalFileSystem local("local");
char yaw_rate_temp[25];
double yaw_rate_1[600];
char* token1;
int brake,flag_1=0,i=0,j=10,k=0,flag_3=0;
double v,sum,steer_dis,angular_motor,signal_APPS1,signal_APPS2,signal_steer,torque_1,torque_2,normalised_APPS1,normalised_APPS2,normalised_avg,normalised_steer,torque_right,torque_left,rpm_1,rpm_2,yaw_in,yaw_out,yaw_error;
const double APPS1_glbmax=630,APPS1_lclmax=600,APPS1_glbmin=270,APPS1_lclmin=300,APPS2_glbmax=730,APPS2_lclmax=700,APPS2_glbmin=370,APPS2_lclmin=400,steer_glbmax=430,steer_lclmax=400,steer_lclmin=200,steer_glbmin=170,torque_max_APPS=300,torque_max_steer=100,off_min=230,off_max=270;
char x[6],y[6],command1[4],command2[4],command3[4],command4[2],command5[8];
char* token;
void Rx_interrupt()
{
  //    while(1)
//   {
    pc.printf("Enter the value of Rpm1:\n");
    pc.scanf("%s",x);
    pc.printf("Enter the value of Rpm2:\n");
    pc.scanf("%s",y);
    pc.printf("Enter the value of APPS Signal 1:\n");
    pc.scanf("%s",command1);
    while(!(pc.readable()));
    pc.printf("Enter the value of APPS Signal 2:\n");
    pc.scanf("%s",command2);
    while(!(pc.readable())) ;
    pc.printf("Enter the value of steer Signal 3:\n");
    pc.scanf("%s",command3);
    while(!(pc.readable()));
    pc.printf("Brakes are pressed or not?\n");
    pc.scanf("%s",command4);
    while(!(pc.readable()));
    pc.printf("Enter the input yaw value:\n");
    pc.scanf("%s",command5);
    token = strtok(command1,",;");
    signal_APPS1= atoi(token);
    token = strtok(command2,",;");
    signal_APPS2=atoi(token);
    token = strtok(command3,",;");
    signal_steer= atoi(token);
    token = strtok(command4,",;");
    brake = atoi(token);
    token=strtok(command5,",;");
    yaw_in=atoi(token);
    yaw_in=yaw_in/100000;
    token=strtok(x,",;");
    rpm_1=atoi(token);
    token=strtok(y,",;");
    rpm_2=atoi(token);
    angular_motor = (rpm_1+rpm_2)/2;//Angular Velocity of Motor
    v = (angular_motor)/1.73;
    for(i=0;i<60;i++)
    {
       if( j <= v && v < j+10)
       {
         break;
       }
       j = j+10;    
    }
    j=5;
    steer_dis = (float(signal_steer)/float(1024))*50;
    for(k=0;k<10;k++)
    {
        if( j <= steer_dis && steer_dis < j+5 )
        {
            break;
        }
        j = j+5;
    }
    sum = yaw_rate_1[10*i+k]+yaw_rate_1[10*i+k+1]+yaw_rate_1[10*(i+1)+k]+yaw_rate_1[10*(i+1)+k+1];
    yaw_out = sum/4 ;
    if(signal_APPS1 <= APPS1_lclmax && signal_APPS1 >= APPS1_lclmin)
    {
        normalised_APPS1=((signal_APPS1 - APPS1_lclmin)/(APPS1_lclmax - APPS1_lclmin))*100;
    }
     else if (signal_APPS1 < APPS1_lclmin && signal_APPS1 >= APPS1_glbmin) 
     {
        signal_APPS1= APPS1_lclmin;
        normalised_APPS1=((signal_APPS1-APPS1_lclmin)/(APPS1_lclmax-APPS1_lclmin))*100;
    }
     else if (signal_APPS1 <= APPS1_glbmax && signal_APPS1 > APPS1_lclmax)
    {
        signal_APPS1= APPS1_lclmax;
        normalised_APPS1=((signal_APPS1-APPS1_lclmin)/(APPS1_lclmax - APPS1_lclmin))*100;
    } 
    else 
    {
        normalised_APPS1=0;
    }
    if(signal_APPS2<=APPS2_lclmax && signal_APPS2>=APPS2_lclmin) 
    {
        normalised_APPS2=((signal_APPS2 - APPS2_lclmin)/(APPS2_lclmax - APPS2_lclmin))*100;
    }
     else if (signal_APPS2<APPS2_lclmin &&signal_APPS2>=APPS2_glbmin) 
    {
        signal_APPS2= APPS2_lclmin;
        normalised_APPS2=((signal_APPS2-APPS2_lclmin)/(APPS2_lclmax-APPS2_lclmin))*100;
    }
     else if (signal_APPS2 <= APPS2_glbmax && signal_APPS2 > APPS2_lclmax) 
    {
        signal_APPS2 = APPS2_lclmax;
        normalised_APPS2=((signal_APPS2-APPS2_lclmin)/(APPS2_lclmax - APPS2_lclmin))*100;
    } 
    else 
    {
        normalised_APPS2=0;
    }
        normalised_avg=(normalised_APPS1+normalised_APPS2)/2;

    if(abs(normalised_APPS1-normalised_APPS2)>10) 
    {
        if(flag_3==0)
        {
        t2.start();
        }
        flag_3=1;
        if(t2.read()<100)
        {
        torque_1=(torque_max_APPS*normalised_avg)/100;
        }
        else
        {
         torque_1=0;   
        }
        }
    else if(abs(normalised_APPS1-normalised_APPS2)<=10)
        {
        t2.stop();
        t2.reset();
        flag_3=0;
        torque_1=(torque_max_APPS*normalised_avg)/100;   
        }
     if(signal_steer<=steer_lclmax && signal_steer>=steer_lclmin)
    {
        normalised_steer=((signal_steer-steer_lclmin)/(steer_lclmax-steer_lclmin))*100;
    } 
    else if(signal_steer < steer_lclmin && signal_steer>=steer_glbmin) 
    {
        signal_steer = steer_lclmin;
        normalised_steer=((signal_steer-steer_lclmin)/(steer_lclmax -steer_lclmin))*100;
    }
    else if(signal_steer<=steer_glbmax && signal_steer>steer_lclmax) 
    {
        signal_steer= steer_lclmax;
        normalised_steer=((signal_steer-steer_lclmin)/(steer_lclmax-steer_lclmin))*100;
    }
     else
    {
        normalised_steer=0;
    }
    
    if(steer_lclmin - 30 < signal_steer && signal_steer < steer_lclmax + 30)

    {
        torque_2=(torque_max_steer*normalised_steer)/100;
    }
     else 
    {
        torque_2=0;
    }

    if(off_min<=signal_steer<=off_max)
    {
        torque_right=torque_1;
        torque_left=torque_1;
    }
    else if(signal_steer<=off_min)
    {
        torque_right=torque_1+torque_2;
        torque_left=torque_1-torque_2;
    }
    else if(signal_steer>=off_max) 
    {
        torque_right=torque_1-torque_2;
        torque_left=torque_1+torque_2;
    }
    if(normalised_avg <= 5 && timer.read()>20)
    {
    timer.stop();
    timer.reset();
    torque_1=0;
    torque_2=0;
    flag_1=0;    
    }
    if(normalised_avg > 5 && timer.read()>20)
    {
    torque_1=0;
    torque_2=0;
    }
    
    if(normalised_avg > 25 && brake==1)
    {
        if(flag_1==0)
        {
        timer.start();
        }
        flag_1=1;    
        if(timer.read()>20)
        {
            torque_1=0;
            torque_2=0;
        } 
     }
    else if((normalised_avg <= 25 || brake==0)&&flag_1==1)
     {
        if(timer.read()<20)
        {
            timer.stop();
            timer.reset();
            flag_1=0;
        }    
     }
     if(normalised_avg < 5)
     {
        torque_1=0;
        torque_2=0;    
     }
    yaw_error = yaw_in - yaw_out;
    pc.printf("yaw_in=%f\n",yaw_in);
    pc.printf("yaw_out = %f\n",yaw_out);
    pc.printf("The value of norm_avg is %f\n",normalised_avg);
    pc.printf("The preceeding velocity index is %d\n",i); 
    pc.printf("The succeeding velocity index is %d\n",i+1); 
    pc.printf("The preceeding steering index is %d\n",k);
    pc.printf("The succeeding steering index is %d\n",k+1);
    pc.printf("yaw_error = %f\n",yaw_error);
    pc.printf("The yaw_out value is %f\n",yaw_rate_1[599]);
}

int main()
{
pc.attach(&Rx_interrupt, Serial::RxIrq);
pc.baud(9600);
    FILE *fp = fopen("/local/yaw.txt", "r");

        for(int j=0; j<600; j++) {
    
            fscanf(fp,"%s",yaw_rate_temp );
            token1 = strtok(yaw_rate_temp,",;");
            yaw_rate_1[j] = atoi(token1);
          yaw_rate_1[j] = yaw_rate_1[j] /1000000000;
}
fclose(fp);
}//}
