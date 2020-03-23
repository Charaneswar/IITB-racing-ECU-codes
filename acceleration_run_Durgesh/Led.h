
int* led_indications(int ignition_sig,int gr_90_sig)
    {
      int led[2];
                if(ignition_sig == 1) {
                    led[0] = 1;
                } else if(ignition_sig==0) {
                    led[0] = 0;
                }

                if(gr_90_sig == 1) {
                    led[1] = 1;
                } else if(gr_90_sig == 0) {
                    led[1] = 0;
                }
        return led;
    
    }