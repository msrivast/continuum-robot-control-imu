// #include <manual_adjustment.cpp>
// #include <manual_adjustment_with_IMU.cpp>
// #if 0


/*
11-23-22: Distal(single) section hose control using IMU feedback.
        - Only controlling theta in one tendon plane. ASSUMES that the robot can achieve 0 to theta max by retracting and extending one tendon.
            The initial tensions must be high enough for 0 theta to be achievable by extension.
        - Expects theta (and phi) information at ~100hz from ESP32 

12-2-22: Finally started to code

12-16-22: Got theta closed loop control in a tendon plane. Working on phi closed loop control
*/


#include <single_section_functions_IMU.cpp>
#include <shape_space_attitude.h>

double target_theta = 0;
// double target_phi = 0;

uint8_t waypoint = 0;

Shape_space_attitude ss_attitude;

uint8_t divisor = 10;//input theta frequency divisor
uint8_t num_higher_level_setpoints = 0;
 //Assuming current theta is generated at around 100hz(~112hz).
//The frequency of setpoints needs to be smaller than 100Hz, which limits the maximum number of setpoints.
//Creating a tunable integer parameter that divides the input theta frequency ==> divisor.
//Therefore, setpoint interval = 1000ms/(100/divisor) or 1000*divisor/100 or 10*divisor at new theta freq of 100hz 
uint8_t higher_level_setpoint_interval = 10*divisor;

double sp_target_theta = 0, sp_target_phi = 0;//setpoint target theta
double delta_theta = 0;
// double delta_phi = 0;

unsigned long last_higher_level_setpoint_time = 0;
// double l4 = 0;

double kappa_p = 10.0; //60.0 tried and works with 10 onwards

double phi_offset = 4.0*PI/3.0; //TBD

uint8_t getNumSetpoints(double waypoint_interval, double setpoint_interval){
    return waypoint_interval/setpoint_interval; // integer division
}

double interpolater(double current, double target, uint8_t setpoint_count){    
    //linear interpolator
    double delta = (double)(target - current)/setpoint_count;
    return delta;
}


void parseDataThetaPhi(){
    
  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars,",");      // get the first part - the string
  ss_attitude.theta = atof(strtokIndx);    // convert this part to an float

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  ss_attitude.phi = atof(strtokIndx);     // convert this part to an float

}

void updateSSAttitudeAndSyncTendonLengths(){
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
        // this temporary copy is necessary to protect the original data
        //   because strtok() used in parseData() replaces the commas with \0
    // if (~serial_garbage){
    //   parseData();
    // }else{
    //   serial_garbage = false;
    //   Serial.println("Garbage received!");
    // }
    newData = false; //needs to be outside the if garbage statement. Because if it were inside, and garbage was received, the arduino's read buffer will overflow because no reads would be performed.
    parseDataThetaPhi();
    for(int i = 0; i<3;i++ ){
        cap_lengths[i] = capstans[i].get_length();
    }
    
  }
}

inline double specialMod(double a, double n){
    return a - floor(a/n)*n;
}

inline double smallSignedAngleBetween(double target, double current){
    // The source and target angles lie between -360 to 360. It will work for upto 540 but not when target is 721 and current is 0.
    double diff = target - current;
    return specialMod(diff + PI,2.0*PI) - PI;
}

void higherLevelPID(){
    //TO DO: Modify to run this only when new IMU data is in but that might affect loop timing consistency

    //PID on theta is performed. Output is scaled (since theta setpoint interval > length setpoint interval) and fed into cap_lengths
    // double output = -1*kappa_p*(sp_target_theta - ss_attitude.theta); // negative becuase delta_l is negative when theta is positive
    // double scaled_output = output*(double)setpoint_interval/(double)theta_setpoint_interval;
    double output[3] = {0}, scaled_output[3] = {0};
    double theta_err = sp_target_theta - ss_attitude.theta;
    double phi_robot_frame = wrapAngle(ss_attitude.phi - phi_offset);

    double phi_err = smallSignedAngleBetween(wrapAngle(sp_target_phi),phi_robot_frame);
    
    output[0] = -1 * kappa_p*(cos(PI/3 - phi_robot_frame)*theta_err + ss_attitude.theta*sin(PI/3 - phi_robot_frame)*phi_err);
    output[1] = -1 * kappa_p*(cos(PI - phi_robot_frame)*theta_err + ss_attitude.theta*sin(PI - phi_robot_frame)*phi_err);
    output[2] = -1 * kappa_p*(cos(5*PI/3 - phi_robot_frame)*theta_err + ss_attitude.theta*sin(5*PI/3 - phi_robot_frame)*phi_err);
    
    for(int i = 0; i<3; i++){
        scaled_output[i] = output[i]*(double)setpoint_interval/(double)higher_level_setpoint_interval;
        // If the capstans are updated with instantaneous lengths, this output should produce the same result as directly generating PWM from theta PID loop without the presence of encoder PID. 
        // capstans[i].set_length(capstans[i].get_length() + scaled_output[i]); //This get_length() corresponds to length during the previous call of capstan.update(). So it moves lesser than what it should have but that difference is linear
        //since the scaled output remains constant.
        capstans[i].set_length(cap_lengths[i] + scaled_output[i]);
    }

}

void loop(){
    /*
    Note 1: @115200bps, if your loop time is greater than 5.5ms, the 64byte serial input buffer will be filled if 64bytes are written in 1 fell swoop.
    Note 2: @100Hz IMU operation speed, we write 18 bytes(when numbers are sent as chars and not considering raw data) every 10ms. If loop time is 
            less than 10ms, the input buffer never contains more than 18 bytes. To write 18 bytes to the input buffer it takes 1.6ms.
    Note 3: We write 15 + 18(presently we are not doing decimal RPY) + 10 + 4 + millis + (setlengths) = greater than 64bytes output buffer every 10ms. There is some blocking that will be 
            experienced by a 64 byte buffer. We arent changing the buffer size, because we will remove the optional setlengths.
    
    
    */
    current_time = millis();
    if(current_time - last_waypoint_time >= waypoint_interval){
        last_waypoint_time = current_time;
        
        
        //last_setpoint_time = 0;//This should result in avoiding the discrepancies in the arduino millis function . SHOULD BE REPLACED WITH LAST HIGHER LEVEL SETPOINT. SOMEHOW DIDNT AFFECT THE THETA CLOSED LOOP EXPERIMENTS
        
        
        target_theta = PI/6;
        // target_phi = target_phi + 2.0*PI/3.0; //PI/3;//target_phi + 2.0*PI/3.0;
        
        //The target phi is current phi based and not prev target phi based. So assuming that the current phi has reached prev target, one can add or subtract next target for CW or CCW rotation.
        target_phi =  wrapAngle(ss_attitude.phi - phi_offset) + 2.0*PI/3.0;// ss_attitude.phi = [0 2pi];; ss_attitude.phi - phi_offset = [-phi_offset 2pi-phi_offset];; wrapAngle(ss_attitude.phi - phi_offset) = [0 2pi];; wrapAngle(ss_attitude.phi - phi_offset) + (-)waypoint = see next line
        //Target_phi can be greater than 2pi and less than 0 at zero crossings if the delta is not a factor of 360.

        // if (waypoint < 2){
        //     target_theta = target_theta + PI/4.0;
        //     waypoint++;
        // }
        // // else if (waypoint == 2){
        // //     waypoint++;
        // // }
        // else{
        //     target_theta = target_theta - PI/4.0;
        //     waypoint++;
        //     if (waypoint == 4) waypoint = 0;
        //     // if (waypoint == 5) waypoint = 0;
        // }
       
        num_higher_level_setpoints = getNumSetpoints(waypoint_interval, higher_level_setpoint_interval);
        delta_theta = interpolater(ss_attitude.theta, target_theta, num_higher_level_setpoints);//linear interpolation
        delta_phi = interpolater(wrapAngle(ss_attitude.phi - phi_offset), target_phi, num_higher_level_setpoints);//linear interpolation
        
        // interpolate_jones_space(0.5*PI/1397.0,target_phi); 
        // interpolate_jones_space(0.0,target_phi);
    }
    




    current_time = millis();
    if(current_time - last_higher_level_setpoint_time >= higher_level_setpoint_interval){
        last_higher_level_setpoint_time = current_time;

        if(num_higher_level_setpoints > 0){
            sp_target_theta += delta_theta; //ss_attitude.theta should not be used here becuase that is likely going to be behind the sp_target_theta. It might not matter much but why not use the correct equation.
            sp_target_phi += delta_phi;
            // sp_target_phi = wrapAngle(ss_attitude.phi - phi_offset) + delta_phi;

            // k = k + delta_k;     
            // phi = phi + delta_phi;

            // inv_kinematics_jones_curved(k,phi);
            // inv_kinematics_jones_piecewise_linear(k,phi);
            // inv_kinematics_moments_axial_compression(1.0,phi,2.4, -1397.0*k*30.0/(4.0/3.0));//the variable k only sets the max curvature. curvatures below it are controlled by the first entry. //axial compression entry was set at 2.4 for MB. 
            
            // inv_kinematics_moments_axial_compression(1.0,phi,0.0, -1397.0*k*30.0*3.0/2.0);//Curved kinematics
            // for(int i=0;i<3;i++){
            //     capstans[i].set_length(cap_lengths[i]);
            // }

            num_higher_level_setpoints--;
        }   
    }


    higherLevelPID(); //Run this from the updateSSAttitudeAndSyncTendonLengths function for efficiency perhaps at the cost of loop time inconsistency.



    for(int i=0;i<3;i++){
        capstans[i].update();
    }
    
    updateSSAttitudeAndSyncTendonLengths();

    // if (printData) //prints data each time IMU sends it
    //     loop_debug_info();

    // if (current_time - last_print_time >= 200){
    //     last_print_time = current_time;
    //     loop_debug_info();
    // }  
}
// #endif