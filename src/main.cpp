#include <manual_adjustment.cpp>
// #include <manual_adjustment_with_IMU.cpp>
// #include <manual_adjustment_with_IMU_dataprocessing.cpp>

/*
6-18-23: Final experiment.
PURPOSE:This code implements EC model in feed forward and active steering as feedback.

2-21-23: streamlined code from main_02-15-25.cpp
Added code enforcing no compression kinematic constraint

2-23-23: reintroduced printing functionality
*/


 #if 0

#include <single_section_functions_IMU.cpp>
#include <shape_space_attitude.h>

double theta_ratio = 0.0;
double target_theta = 0;
// double target_phi = 0; //defined globally in single_section_functions_IMU.cpp

uint8_t waypoint = 0;

Shape_space_attitude ss_attitude;

uint8_t divisor = 10;//input theta frequency divisor
uint8_t num_higher_level_setpoints = 0;
 //Assuming current theta is generated at around 100hz(~112hz).
//The frequency of setpoints needs to be smaller than 100Hz, which limits the maximum number of setpoints.
//Creating a tunable integer parameter that divides the input theta frequency ==> divisor.
//Therefore, setpoint interval = 1000ms/(100/divisor) or 1000*divisor/100 or 10*divisor at new theta freq of 100hz 
uint8_t higher_level_setpoint_interval = 5*divisor;

double sp_target_theta = 0, sp_target_phi = 0;//setpoint target theta
double delta_theta = 0;
double R = 30.0, theta_max = 1.0*PI/3.0;
// double delta_phi = 0; //defined globally in single_section_functions_IMU.cpp

unsigned long last_higher_level_setpoint_time = 0;
// double l4 = 0;

double kappa_p = 1;//10.0; //60.0 tried and works with 10 onwards

double phi_offset = 0.9428;//0.903; //0.973//4.0*PI/3.0; //TBD

double caplengths_FF[3] = {0.0};


inline double specialMod(double a, double n){
    return a - floor(a/n)*n;
}

void loop_debug_info(){
    Serial.print(millis());
    Serial.print('\t');
    Serial.print(num_higher_level_setpoints);
    Serial.print('\t');
    // Serial.print(cap_lengths[0],1);
    // Serial.print('\t');
    Serial.print(capstans[0].get_length(),0);
    Serial.print(';');
    // Serial.print(cap_lengths[1],1);
    // Serial.print('\t');
    Serial.print(capstans[1].get_length(),0);
    Serial.print(';');
    // Serial.print(cap_lengths[2],1);
    // Serial.print('\t');
    Serial.print(capstans[2].get_length(),0);
    Serial.print('\t');
    Serial.print(sp_target_theta*180.0/PI,0);
    Serial.print(';');
    Serial.print(specialMod(sp_target_phi*180.0/PI,360.0),0);
    Serial.print('\t');
    Serial.print(ss_attitude.theta*180.0/PI,1);
    Serial.print(';');
    Serial.println(ss_attitude.phi*180.0/PI,0);
    // Serial.println('\t');
      //Debug prints
    
    // Serial.print(attitude.Roll,0);
    // Serial.print('\t');
    // Serial.print(attitude.Pitch,0);
    // Serial.print('\t');
    // Serial.print(attitude.Yaw,0);
    // Serial.print('\t');
    // Serial.write(tempChars);
    // Serial.println('\t');
    
    // Serial.println(Serial.availableForWrite());

    // Serial.print('\t');
    // Serial.print(capstan_5.get_PId_output(),1);
    // Serial.print('\t');
    // Serial.println(capstan_5.get_current());

    printData = false;
}

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
  ss_attitude.phi = wrapAngle(atof(strtokIndx) - phi_offset);     // convert this part to an float

  printData = true;

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


inline double smallSignedAngleBetween(double target, double current){
    // The source and target angles lie between -360 to 360. It will work for upto 540 but not when target is 721 and current is 0.
    double diff = target - current;
    return specialMod(diff + PI,2.0*PI) - PI;
}


void higherLevelPID_kinematic(){
    //TO DO: Modify to run this only when new IMU data is in but that might affect loop timing consistency

    //PID on theta is performed. Output is scaled (since theta setpoint interval > length setpoint interval) and fed into cap_lengths
    // double output = -1*kappa_p*(sp_target_theta - ss_attitude.theta); // negative becuase delta_l is negative when theta is positive
    // double scaled_output = output*(double)setpoint_interval/(double)theta_setpoint_interval;
    double output[3] = {0}, scaled_output[3] = {0};
    double theta_err = sp_target_theta - ss_attitude.theta;
    double phi_robot_frame = ss_attitude.phi;

    double phi_err = smallSignedAngleBetween(wrapAngle(sp_target_phi),phi_robot_frame); //since sp_target_phi starts at 0 and we dont know what phi_robot_frame is(singularity), the first phi_err calculated at the startup is random but after that it isn't.
    
    //Kinematic Jacobian
    output[0] = -1 * kappa_p*(cos(PI/3 - phi_robot_frame)*theta_err + ss_attitude.theta*sin(PI/3 - phi_robot_frame)*phi_err);
    output[1] = -1 * kappa_p*(cos(PI - phi_robot_frame)*theta_err + ss_attitude.theta*sin(PI - phi_robot_frame)*phi_err);
    output[2] = -1 * kappa_p*(cos(5*PI/3 - phi_robot_frame)*theta_err + ss_attitude.theta*sin(5*PI/3 - phi_robot_frame)*phi_err);

    //Enforce length constraints
    double sum = 0;
    for(int i = 0; i<3;i++){
        sum += cap_lengths[i];
    }

    if(PI/3.0 <= ss_attitude.phi && ss_attitude.phi < PI){
        output[2] = -(sum + output[0] + output[1]);
    }else if (PI <= ss_attitude.phi && ss_attitude.phi < 5.0*PI/3.0){
        output[0] = -(sum + output[1] + output[2]);
    }else{
        output[1] = -(sum + output[2] + output[0]);
    }
    
    for(int i = 0; i<3; i++){
        scaled_output[i] = output[i]*(double)setpoint_interval/(double)higher_level_setpoint_interval;
        // If the capstans are updated with instantaneous lengths, this output should produce the same result as directly generating PWM from theta PID loop without the presence of encoder PID. 
        // capstans[i].set_length(capstans[i].get_length() + scaled_output[i]); //This get_length() corresponds to length during the previous call of capstan.update(). So it moves lesser than what it should have but that difference is linear
        //since the scaled output remains constant.
  
        capstans[i].set_length(cap_lengths[i] + scaled_output[i]);
    }

}

void higherLevelPID_MBAX(){
    double output[3] = {0}, scaled_output[3] = {0};
    double theta_err = sp_target_theta - ss_attitude.theta;
    double phi_robot_frame = ss_attitude.phi;

    double phi_err = smallSignedAngleBetween(wrapAngle(sp_target_phi),phi_robot_frame); //since sp_target_phi starts at 0 and we dont know what phi_robot_frame is(singularity), the first phi_err calculated at the startup is random but after that it isn't.
    
    //MBAX Jacobian
    output[0] = -1 * kappa_p*(theta_err + cos(PI/3 - phi_robot_frame)*theta_err + ss_attitude.theta*sin(PI/3 - phi_robot_frame)*phi_err);
    output[1] = -1 * kappa_p*(theta_err + cos(PI - phi_robot_frame)*theta_err + ss_attitude.theta*sin(PI - phi_robot_frame)*phi_err);
    output[2] = -1 * kappa_p*(theta_err + cos(5*PI/3 - phi_robot_frame)*theta_err + ss_attitude.theta*sin(5*PI/3 - phi_robot_frame)*phi_err);

    //Enforce MBAX length constraints
    double sum = 0;
    for(int i = 0; i<3;i++){
        sum += cap_lengths[i];
    }
    double compression_multiplier = 2.0;
    if(PI/3.0 <= ss_attitude.phi && ss_attitude.phi < PI){
        output[2] = -compression_multiplier*R*theta_max -(sum + output[0] + output[1]);
    }else if (PI <= ss_attitude.phi && ss_attitude.phi < 5.0*PI/3.0){
        output[0] = -compression_multiplier*R*theta_max -(sum + output[1] + output[2]);
    }else{
        output[1] = -compression_multiplier*R*theta_max -(sum + output[2] + output[0]);
    }
    
    for(int i = 0; i<3; i++){
        scaled_output[i] = output[i]*(double)setpoint_interval/(double)higher_level_setpoint_interval;
        // If the capstans are updated with instantaneous lengths, this output should produce the same result as directly generating PWM from theta PID loop without the presence of encoder PID. 
        // capstans[i].set_length(capstans[i].get_length() + scaled_output[i]); //This get_length() corresponds to length during the previous call of capstan.update(). So it moves lesser than what it should have but that difference is linear
        //since the scaled output remains constant.
        capstans[i].set_length(cap_lengths[i] + scaled_output[i]);
    }

}


void higherLevelPID_MBAX_FF(){
    double output[3] = {0}, scaled_output[3] = {0};
    double theta_err = sp_target_theta - ss_attitude.theta;
    double phi_robot_frame = ss_attitude.phi;

    double phi_err = smallSignedAngleBetween(wrapAngle(sp_target_phi),phi_robot_frame); //since sp_target_phi starts at 0 and we dont know what phi_robot_frame is(singularity), the first phi_err calculated at the startup is random but after that it isn't.
    
    //Kinematic Jacobian
    output[0] = -1 * kappa_p*(cos(PI/3 - phi_robot_frame)*theta_err + ss_attitude.theta*sin(PI/3 - phi_robot_frame)*phi_err);
    output[1] = -1 * kappa_p*(cos(PI - phi_robot_frame)*theta_err + ss_attitude.theta*sin(PI - phi_robot_frame)*phi_err);
    output[2] = -1 * kappa_p*(cos(5*PI/3 - phi_robot_frame)*theta_err + ss_attitude.theta*sin(5*PI/3 - phi_robot_frame)*phi_err);

       //MBAX Jacobian
    // output[0] = -1 * kappa_p*(theta_err + cos(PI/3 - phi_robot_frame)*theta_err + ss_attitude.theta*sin(PI/3 - phi_robot_frame)*phi_err);
    // output[1] = -1 * kappa_p*(theta_err + cos(PI - phi_robot_frame)*theta_err + ss_attitude.theta*sin(PI - phi_robot_frame)*phi_err);
    // output[2] = -1 * kappa_p*(theta_err + cos(5*PI/3 - phi_robot_frame)*theta_err + ss_attitude.theta*sin(5*PI/3 - phi_robot_frame)*phi_err);
    //Enforce length constraints
    // double sum = 0;
    // for(int i = 0; i<3;i++){
    //     sum += cap_lengths[i];
    // }

    // if(PI/3.0 <= ss_attitude.phi && ss_attitude.phi < PI){
    //     output[2] = -(sum + output[0] + output[1]);
    // }else if (PI <= ss_attitude.phi && ss_attitude.phi < 5.0*PI/3.0){
    //     output[0] = -(sum + output[1] + output[2]);
    // }else{
    //     output[1] = -(sum + output[2] + output[0]);
    // }

    //Enforce MBAX length constraints
    // double sum = 0;
    // for(int i = 0; i<3;i++){
    //     sum += cap_lengths[i];
    // }
    // double compression_multiplier = 2.0;
    // if(PI/3.0 <= ss_attitude.phi && ss_attitude.phi < PI){
    //     output[2] = -compression_multiplier*R*theta_max -(sum + output[0] + output[1]);
    // }else if (PI <= ss_attitude.phi && ss_attitude.phi < 5.0*PI/3.0){
    //     output[0] = -compression_multiplier*R*theta_max -(sum + output[1] + output[2]);
    // }else{
    //     output[1] = -compression_multiplier*R*theta_max -(sum + output[2] + output[0]);
    // }

    static double scaled_op_FB[3] = {0.0};
    for(int i = 0; i<3; i++){
        scaled_output[i] = output[i]*(double)setpoint_interval/(double)higher_level_setpoint_interval;
        // If the capstans are updated with instantaneous lengths, this output should produce the same result as directly generating PWM from theta PID loop without the presence of encoder PID. 
        // capstans[i].set_length(capstans[i].get_length() + scaled_output[i]); //This get_length() corresponds to length during the previous call of capstan.update(). So it moves lesser than what it should have but that difference is linear
        //since the scaled output remains constant.
        // capstans[i].set_length(cap_lengths[i] + scaled_output[i]);
        
        scaled_op_FB[i] += scaled_output[i];
        capstans[i].set_length(caplengths_FF[i] + scaled_op_FB[i]); //caplengths_FF is updates every ~100ms and scaled_op_FF every ~10ms
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
        

        target_theta = PI/6.0;//REMEMBER theta_max controls compression
        // target_phi = target_phi + 2.0*PI/3.0; //PI/3;//target_phi + 2.0*PI/3.0;
        
        //The target phi is current phi based and not prev target phi based. So assuming that the current phi has reached prev target, one can add or subtract next target for CW or CCW rotation. This is better than target_phi += 2pi/3 because
        //of 1. we do not know what phi is at the straight config and 2. Due to the next comment, zero crossings are managed correctly. The other case would have resulted in a large delta_phi after interpolation at zero crossings.
        double delta = 2.0*PI/3.0;
        // target_phi =  ss_attitude.phi + delta;// ss_attitude.phi = [0 2pi];; ss_attitude.phi - phi_offset = [-phi_offset 2pi-phi_offset];; wrapAngle(ss_attitude.phi - phi_offset) = [0 2pi];; wrapAngle(ss_attitude.phi - phi_offset) + (-)waypoint = see next line
        //Target_phi can be greater than 2pi and less than 0 at zero crossings if the delta is not a factor of 360.

       
        num_higher_level_setpoints = getNumSetpoints(waypoint_interval, higher_level_setpoint_interval);
        // delta_theta = interpolater(ss_attitude.theta, target_theta, num_higher_level_setpoints);//linear interpolation
        // delta_phi = interpolater(ss_attitude.phi, target_phi, num_higher_level_setpoints);//linear interpolation
        delta_theta = interpolater(sp_target_theta, target_theta, num_higher_level_setpoints);//linear interpolation; interpolating using the current IMU theta (above) results in an increasing theta becuase the waypoints are in the midplanes where robot dips 
        delta_phi = delta/num_higher_level_setpoints;

        // interpolate_jones_space(0.5*PI/1397.0,target_phi); 
        // interpolate_jones_space(0.0,0.0);
    }
    




    current_time = millis();
    if(current_time - last_higher_level_setpoint_time >= higher_level_setpoint_interval){
        last_higher_level_setpoint_time = current_time;

        if(num_higher_level_setpoints > 0){
            sp_target_theta += delta_theta; //ss_attitude.theta should not be used here becuase that is likely going to be behind the sp_target_theta. It might not matter much but why not use the correct equation.
            sp_target_phi += delta_phi; //unbounded and starts at 0
            // sp_target_phi = 0;
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
            
            // inv_kinematics_moments_axial_compression(0.7,sp_target_phi,2.0, -2.0*theta_max*30.0/(4.0/3.0));//the variable k only sets the max curvature. curvatures below it are controlled by the first entry. //axial compression entry was set at 2.4 for MB. 
            // inv_kinematics_moments_axial_compression(1.0,sp_target_phi,0.0, -theta_max*30.0*3.0/2.0);//Curved kinematics
            inv_kinematics_moments_axial_compression(0.5,sp_target_phi,2.0, -1.00*theta_max*30.0*3.0/2.0);//EC model experiment
            inv_kinematics_moments_axial_compression(1.0,0.0,0.0, 0.0);//Straight config
            

            for(int i=0;i<3;i++){
                caplengths_FF[i] = cap_lengths[i];//workaround to have caplengths do double duty; copies caplengths immediately after kinematics generates it
                capstans[i].set_length(cap_lengths[i]); //use only when feedback is not used
            }
            num_higher_level_setpoints--;
        }   
    }


    // higherLevelPID_kinematic(); //Run this from the updateSSAttitudeAndSyncTendonLengths function for efficiency perhaps at the cost of loop time inconsistency.

    // higherLevelPID_MBAX();

    // higherLevelPID_MBAX_FF();

    for(int i=0;i<3;i++){
        capstans[i].update();
    }
    
    updateSSAttitudeAndSyncTendonLengths(); //the advantage of not using interrupts is that we can use cap_lengths[] as originally intended with kinematics, as long as the capstans are updated before the IMU update! 

    // if (printData) //prints data each time IMU sends it. I am printing phi after processing it. Make sure to know whether phi_offset has been subtracted or not as part of the processing.
    //     loop_debug_info();

    if (current_time - last_print_time >= 200){
        last_print_time = current_time;
        loop_debug_info();
    }  
}
#endif