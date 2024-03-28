// #include <manual_adjustment.cpp>
// #include <manual_adjustment_with_IMU.cpp>
// #if 0


/*
This is just an IMU data logger.
10-26-22: Added IMU stuff for a single section. Will also explore current sensing.
        Will also need to add non blocking serial printing capability


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


#include <single_section_functions_IMU.cpp>



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
        last_setpoint_time = 0;//This should result in avoiding the discrepancies in the arduino millis function 
        
        target_phi = target_phi + 2.0*PI/3.0;

        // interpolate_jones_space(0.5*PI/1397.0,target_phi); 
        interpolate_jones_space(0.0,target_phi);
    }
    




    current_time = millis();
    if(current_time - last_setpoint_time >= setpoint_interval){
        last_setpoint_time = current_time;
        /*
        ******Setpoint counting logic******
        The robot reaches the waypoint with a little more than one setpoint interval to spare.
        Therefore, the last setpoint should not be executed. The drawback of this strategy is that
        the speed of the EE is uneven. This can be imperceptible but is something to look at if
        EE speed unevenness starts to show up.(Based on waypoint interval being exactly divisible by
        setpoint interval)
        */
        // if(num_setpoints>1){//Above comment holds for this line
        if(num_setpoints>0){
            k = k + delta_k;     
            phi = phi + delta_phi;

            // inv_kinematics_jones_curved(k,phi);
            // inv_kinematics_jones_piecewise_linear(k,phi);
            inv_kinematics_moments_axial_compression(1.0,phi,2.4, -1397.0*k*30.0/(4.0/3.0));//the variable k only sets the max curvature. curvatures below it are controlled by the first entry. //axial compression entry was set at 2.4 for MB. 
            // inv_kinematics_moments_axial_compression(1.0,phi,0.0, -1397.0*k*30.0*3.0/2.0);//Curved kinematics
            for(int i=0;i<3;i++){
                capstans[i].set_length(cap_lengths[i]);
            }
            num_setpoints--;
        }   
    }




    for(int i=0;i<3;i++){
        capstans[i].update();
    }
    
    updateAttitude();

    if (printData)
        loop_debug_info();

    // if (current_time - last_print_time >= 200){
    //     last_print_time = current_time;
    //     loop_debug_info();
    // }  
}
// #endif