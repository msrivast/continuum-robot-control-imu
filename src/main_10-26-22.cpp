// #include <manual_adjustment.cpp>

/*
9-9-22: Removed all the IMU stuff. Removed all the two section stuff. 
Code for doing the single distal(total) section experiments to demonstrate axial compression.
Unspooled all proximal tendons.

The goal: Compare the EE movement with 
1. piecewise linear kinematics
2. regular kinematics or [1;0;0]*3/2
3. MB + AX [1;0;2] * theta*d /(4/3)
4. [1;0;1] *theta *d

//4-14-22: Re-adjusted the section lengths 

//4-6-22: Removing proximal phi's dependence on distal phi. Removing redundant code
//Major changes: phi2 is considered the dominant phi. Anything that's superflous has been removed.

//6-29-22: Added IMU data relay mechanism
*/


#include <single_section_functions.cpp>

// double theta = 0.0;
void loop(){

    current_time = millis();
    //Next waypoint in terms of tendon length
    if(current_time - last_waypoint_time >= waypoint_interval){
        last_waypoint_time = current_time;
        last_setpoint_time = 0;//This should result in avoiding the discrepancies in the arduino millis function 
        
        target_phi = target_phi + 2.0*PI/3.0;

        // interpolate_jones_space(0.0,target_phi);
        // theta = 0.0; // setting theta to 0 will override any matrix calculations and send the section to initial rest position
        // theta = 0.8*PI;
        interpolate_jones_space(0.8*PI/1397.0,target_phi);

    }
    
    current_time = millis();//If this line is commented, the setpoint set length that needed to be 
    //executed in between last_waypoint_time and current_time won't be executed and would be delayed by loop time.
    
    if(current_time - last_setpoint_time >= setpoint_interval){
        last_setpoint_time = current_time;
        
        //Setpoint counting logic
        if(num_setpoints>1){

            k = k + delta_k;     
            phi = phi + delta_phi;

            //k usage is WRONG for every function becuase it keeps changing everytime it enters this section!!!!!!!!!!!!!!!!!!!

            // inv_kinematics_jones_curved(k,phi);
            // inv_kinematics_jones_piecewise_linear(k,phi);
            // inv_kinematics_moments_axial_compression(true,phi,2.4, -1397.0*k*30.0/(4.0/3.0));
            inv_kinematics_moments_axial_compression(1.0,phi,2.4, -1397.0*k*30.0/(4.0/3.0));//the variable k only sets the max curvature. curvatures below it are controlled by the first entry.
            // inv_kinematics_moments_axial_compression(true,phi,0, -1397.0*k*30.0); //Need to change refret and third entry when running kinematics!!

            for(int i=0;i<3;i++){
                capstans[i].set_length(cap_lengths[i]);
            }
            num_setpoints--;
        }
        // else if (num_setpoints==1){
        //     num_setpoints--; //Need this to prevent setting a setpoint above the target waypoint when the w/s division is exact.
        // }
        
    }

    for(int i=0;i<3;i++){
        capstans[i].update();
    }
    
    // if (current_time - last_print_time >= 1000){
    //     last_print_time = current_time;
    //     loop_debug_info();
    // }  
}
