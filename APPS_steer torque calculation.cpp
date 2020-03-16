#include "mbed.h"
Serial pc(USBTX, USBRX); // tx, rx
///////////APPS inuts
float apps1_analog_input;
float apps2_analog_input;
float apps_input;

//// APPS sensor values max and min
float const apps1_max = 800, apps2_max = 900;
float const apps1_min = 100, apps2_min = 200;
float const apps1_global_max = 850, apps2_global_max = 950;
float const apps1_global_min = 50, apps2_global_min = 150;

float const torque_max_for_motor = 80;

float apps1_normalise;
float apps2_normalise;
float apps_normalised_difference;
Timer apps_plausibility_timer;

float torque = 0; //this is the torque value that I get from the apps

////////////Steering Sensor input
float steering_sensor_input;

//steering sensor input values for {{{{{
float const steering_right_max = 800;
float const steering_left_max = 200;
float const steering_right_global_max = 900;
float const steering_left_global_max = 100;
float const steering_right_min = 510;
float const steering_left_min = 490;
int flag = 1;
int steer_flag=0;
//float const zero_steering = 500;
//}}}}}

float const turning_torque_multiplier_constant = 0.5; // range from 0 to 1

float torque_left = 0;
float torque_right = 0;

//pedal pressing will send same torque value...

//braking
float brake_sensor_input;
float const hard_brake_value = 500;
int brake_flag = 0;
Timer t;

///////////////////

float const max_battery_power_allowed = 80000;
float battery_voltage;
float battery_current;
float w_left, w_right;
float battery_to_motors_efficiency;
float battery_power;
float motor_powers_sum;
//angular velocity of both the wheels
//float power_left, power_right;`

/////////////////////////////////////////////define functions//////////////////////////////////////////////////////////
void get_inputs()
{
    pc.printf("Enter apps_sensor_1 value:  ");
    while (!pc.readable());
    pc.scanf("%f",&apps1_analog_input);
    pc.printf("%f\n",apps1_analog_input);

    pc.printf("Enter apps_sensor_2 value:  ");
    while (!pc.readable());
    pc.scanf("%f",&apps2_analog_input);
    pc.printf("%f\n",apps2_analog_input);

    pc.printf("Enter steering_sensor value:  ");
    while (!pc.readable());
    pc.scanf("%f",&steering_sensor_input);
    pc.printf("%f\n",steering_sensor_input);

    pc.printf("Enter brake_sensor value:  ");
    while (!pc.readable());
    pc.scanf("%f",&brake_sensor_input);
    pc.printf("%f\n",brake_sensor_input);
}



void print_all_values()
{
    //print torque of both the wheels and other required values
    pc.printf("flag: ");
    pc.printf("%d \n",flag);
    pc.printf("steer flag: ");
    pc.printf("%d \n",steer_flag);
    pc.printf("brake flag: ");
    pc.printf("%d \n",brake_flag);
    pc.printf("t.read() : ");
    pc.printf("%f \n",t.read());
    pc.printf("apps final value: ");
    pc.printf("%f \n",apps_input);
    pc.printf("left torque: ");
    pc.printf("%f \n",torque_left);
    pc.printf("right torque: ");
    pc.printf("%f \n",torque_right);
    pc.printf("\n");
}

void set_torque_in_range()
{
    //if torque value is out of range (0 to torque_max_for_motor) then setting the values to the extremum (0 or torque_max_for_motor)
    if(torque_left > torque_max_for_motor) torque_left = torque_max_for_motor;
    if(torque_right > torque_max_for_motor) torque_right = torque_max_for_motor;
    if(torque_left < 0) torque_left = 0;
    if(torque_right < 0) torque_right = 0;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main()
{
    while(1) {
        //take inputs
        get_inputs();

        //normalise the value of apps1 and apps2
        apps1_normalise = (apps1_analog_input-apps1_min)/(apps1_max-apps1_min);
        apps2_normalise = (apps2_analog_input-apps2_min)/(apps2_max-apps2_min);
        apps_normalised_difference = apps1_normalise-apps2_normalise;
        if(apps_normalised_difference<0) {
            apps_normalised_difference = -1*apps_normalised_difference;
        }

        //perform all operations
        if(apps1_analog_input>apps1_global_max || apps2_analog_input>apps2_global_max || apps1_analog_input<apps1_global_min || apps2_analog_input<apps2_global_min
                || steering_sensor_input < steering_left_global_max || steering_sensor_input > steering_right_global_max) {
            torque_left = 0;
            torque_right = 0;
            flag = 2;
        } else {
            flag = 3;
            if(apps1_analog_input > apps1_max) {
                apps1_analog_input = apps1_max;
            }
            if(apps2_analog_input > apps2_max) {
                apps2_analog_input = apps2_max;
            }
            if(apps1_analog_input < apps1_min) {
                apps1_analog_input = apps1_min;
            }
            if(apps2_analog_input < apps2_min) {
                apps2_analog_input = apps2_min;
            }



            apps_input = ( apps1_normalise + apps2_normalise ) / 2;
            //APPS inputs taken and converted to range 0 to 1... then their avg value is taken

            torque = apps_input*torque_max_for_motor;
            //torque value normalised in the range for our motors

            if(steering_sensor_input > steering_right_min) {
                //turning right
                steer_flag = 1;
                if (steering_sensor_input > steering_right_max) {
                    steering_sensor_input = steering_right_max;
                }
                steering_sensor_input = (steering_sensor_input - steering_right_min)/(steering_right_max - steering_right_min);
                torque_left = torque*(1 + steering_sensor_input*turning_torque_multiplier_constant);
                torque_right = torque*(1 - steering_sensor_input*turning_torque_multiplier_constant);

            }

            else if(steering_sensor_input < steering_left_min) {
                //turning left
                steer_flag = -1;
                if (steering_sensor_input < steering_left_max) {
                    steering_sensor_input = steering_left_max;
                }
                steering_sensor_input = (steering_sensor_input - steering_left_min)/(steering_left_max - steering_left_min);
                torque_left = torque*(1 - steering_sensor_input*turning_torque_multiplier_constant);
                torque_right = torque*(1 + steering_sensor_input*turning_torque_multiplier_constant);

            }

            else {
                steer_flag = 101;
                //no steering
                torque_left = torque;
                torque_right = torque;
            }

            //if torque value is out of range (0 to torque_max_for_motor) then setting the values to the extremum (0 or torque_max_for_motor)
            set_torque_in_range();

            if(brake_sensor_input>hard_brake_value && apps_input>0.25) {
                if (brake_flag == 0) {
                    t.start();
                    brake_flag = 1;
                } else if(t.read() > 0.5) {
                    torque_left = 0;
                    torque_right = 0;
                    brake_flag = 1;
                }

            } else if(brake_sensor_input<hard_brake_value && apps_input<0.05) {
                t.reset();
                t.stop();
                brake_flag = 0;
            } else if(brake_flag == 1) {
                torque_left = 0;
                torque_right = 0;
            }
            //else if(t.read()>0.5 && brake_flag==1 && (brake_sensor_input>hard_brake_value || apps_input>0.25) ) {
            //    torque_left = 0;
            //    torque_right = 0;
            //}


        }

        //check if battery power > max allowed.... if yes.... then adjust accordingly
        motor_powers_sum = torque_right*w_right + torque_left*w_left;
        battery_power = motor_powers_sum/battery_to_motors_efficiency;
        if(battery_power > max_battery_power_allowed) {
            float torque_adjustment_factor = max_battery_power_allowed/battery_power;
            torque_left = torque_adjustment_factor*torque_left;
            torque_right = torque_adjustment_factor*torque_right;
        }

        if(apps1_analog_input == apps2_analog_input) {
            torque_left = 0;
            torque_right = 0;
        }

        if(apps_normalised_difference>0.1) {
            if(apps_plausibility_timer.read() == 0) {
                apps_plausibility_timer.start();
            }

            if(apps_plausibility_timer.read() > 0.1) {
                torque_left = 0;
                torque_right = 0;
            } else {
                apps_plausibility_timer.reset();
            }
        }


        //print torque of both the wheels and other required values
        print_all_values();


    }

}


