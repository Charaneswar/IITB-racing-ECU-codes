#include "mbed.h"

LocalFileSystem local("local");
char yaw_rate[600][25];
char  c[25];

int main()
{
    FILE *fp = fopen("/local/yaw.txt", "r");

        for(int j=0; j<600; j++) {
    
        fscanf(fp,"%s",c );
           for(int i=0;i<25;i++)
           {
               yaw_rate[j][i]=c[i];

               }

        }

    fclose(fp);
            for(int k=0; k<600; k++) {
                   
               printf(yaw_rate[k]);
                printf("\n");
               }

}
