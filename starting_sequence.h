
void starting_sequence(int *flag_rtds,int press_brakes_to_start, int BPS_in, int * rtds_Sig, int *lvready,int *led3, int *flag_rtds, int torque_avg, int rpm_right_receive){                
               int BPS_actuated = 170;
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
                pc.printf("torque_avg= %d , rpm_sent=%d, rpm_Recieve = %d \n ",torque_avg,torque_avg*7000/32676,rpm_right_receive);
                }