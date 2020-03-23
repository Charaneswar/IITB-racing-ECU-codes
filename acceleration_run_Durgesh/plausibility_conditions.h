
void plausibility_conditions(int APPS1_normalised,int APPS2_normalised){
        if(abs(APPS1_normalised - APPS2_normalised)>10) 
    {
        if(flag_APPS_Check==0)
        {
        t2.start();
        }
        
        flag_APPS_Check = 1;
        
        if(t2.read()<100)
        {
      torque_avg = (Torquemax*normalised_avg)/100;
        }
        else
        {
         torque_avg = 0;   
        }
        }
    else if(abs(APPS1_normalised - APPS2_normalised)<10)
        {
        t2.stop();
        t2.reset();
        flag_APPS_Check = 0;
        torque_avg =(Torquemax*APPS_avg)/100;   
        }
    else if(abs(normalised_APPS1-normalised_APPS2)==0)
        {
            torque_avg = 0;
        }
        }
        
        }