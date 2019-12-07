#include "mbed.h"

LocalFileSystem local("local");
char yaw_rate_temp[25];
double yaw_rate_1[600];
char* token;
Serial pc(USBTX,USBRX);

int main()
{    pc.baud(9600);
    FILE *fp = fopen("/local/yaw.txt", "r");

        for(int j=0; j<600; j++) {
    
            fscanf(fp,"%s",yaw_rate_temp );
            token = strtok(yaw_rate_temp,",;");
            yaw_rate_1[j] = atoi(token);
//            yaw_rate_1[j] = yaw_rate_1[j] /100000000;

        }
    double* a= &yaw_rate_1[0]  ;
    fclose(fp);
            for(int k=0; k<600; k++) {
           
     printf("%lf \n", *a);
          a++;

               }

}
