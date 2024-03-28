// #include <manual_adjustment_with_IMU.cpp>
/*
//  #include <manual_adjustment.cpp>

#include <Arduino.h>

#define SERIAL_PORT Serial2

void setup(){
    Serial.begin(115200);
    SERIAL_PORT.begin(57600); //clock speeds have been modified for PWM - something to look into.
    //Except for timer0, all other timer's have their frequency modified for PWM. 
    //Timers frequency doesnt control serial interrupts 
    while(SERIAL_PORT.available()){
        SERIAL_PORT.read();
    }
}



void loop(){
    if (SERIAL_PORT.available())
        Serial.write(SERIAL_PORT.read());
}
*/

/*
// #include <manual_adjustment.cpp>

//4-14-22: Re-adjusted the section lengths 

//4-6-22: Removing proximal phi's dependence on distal phi. Removing redundant code
//Major changes: phi2 is considered the dominant phi. Anything that's superflous has been removed.

//6-29-22: Added IMU data relay mechanism

#include <functions.cpp>

inline Matrix<2> convert_to_curvatures(double angle){
    double k_x,k_z;
    Matrix<2> section_curvature;
    angle = wrapAngle(angle);
    if (angle >= PI){
        k_x = abs(cos(-angle-PI/2.0));
    }else{
        k_x = -abs(cos(-angle-PI/2.0));
    }
    if (angle<PI/2.0 || angle>3.0*PI/2.0){
        k_z = -abs(tan(-angle-PI/2.0)*k_x);
    }else{
        k_z = abs(tan(-angle-PI/2)*k_x);
    }
    section_curvature(0) = k_x;
    section_curvature(1) = k_z;
    
    return section_curvature;
}
    
inline Matrix<6> relative_bending(Matrix<6,6> inv_A, double phi1,double phi2){
    double k_x_1,k_z_1,k_x_2,k_z_2;
    Matrix<2> section_curvature;    
    section_curvature = convert_to_curvatures(phi2);
    k_x_2 = section_curvature(0);
    k_z_2 = section_curvature(1);
    section_curvature = convert_to_curvatures(phi1);
    k_x_1 = section_curvature(0);
    k_z_1 = section_curvature(1);
    Matrix<6> config= {k_x_1,k_z_1,2,k_x_2,k_z_2,2};//Next steps in research!!!!
    // Matrix<6> config= {0,0,6,k_x_2,k_z_2,2};//Next steps in research!!!!
    return inv_A*config;
}

inline Matrix<6> bending(Matrix<3,3> inv_A_proximal, Matrix<3,3> inv_A_distal, double phi1,double phi2, double ref_ret_proximal, double ref_ret_distal){
    double k_x_1,k_z_1,k_x_2,k_z_2;
    Matrix<2> section_curvature;    
    section_curvature = convert_to_curvatures(phi2);
    k_x_2 = section_curvature(0);
    k_z_2 = section_curvature(1);
    section_curvature = convert_to_curvatures(phi1);
    k_x_1 = section_curvature(0);
    k_z_1 = section_curvature(1);
    Matrix<3> config_proximal= {k_x_1,k_z_1,2};
    Matrix<3> config_distal = {k_x_2,k_z_2,2};
    Matrix<3> config_straight = {0,0,2};
    // Matrix<6> config= {0,0,6,k_x_2,k_z_2,2};//Next steps in research!!!!
    Matrix<3> lengths_proximal = inv_A_proximal*config_proximal*ref_ret_proximal + inv_A_proximal*config_straight*ref_ret_distal*0.5 ;
    Matrix<3> lengths_distal = inv_A_distal*config_proximal*ref_ret_proximal + inv_A_distal*config_distal*ref_ret_distal;
    Matrix<6> lengths = lengths_proximal && lengths_distal;
    return lengths;
}

void inv_kinematics_not_moment(double k1,double phi1, double k2, double phi2){
    // Three tendon retraction pivot model for distal section
    double R = 30.0, s1 = 698.50,s2 = 698.50;
    double ref_retraction = -s2 * R * k2; // CAUTION: While this is used as a measurement of the tendon lengths in the tendon midplanes, this is used as a proxy for the actual theta measurment.
    // Actually, retraction is a function of sin(theta) while this suggests it is a linear function of theta. This doesn't matter in our particular implementation when 
    // we use theta in a relative context but this retraction doesn't capture the real behavior at the capstans with respect to theta.

    cap_lengths[3] = (2.0/3.0 + cos(PI/3.0 -phi2 + PI/3.0)/3.0 + sin(PI/3.0 -phi2 + PI/3.0)/sqrt(3))*ref_retraction;
    cap_lengths[4] = (2.0/3.0 + cos(PI/3.0 -phi2 + PI)/3.0 + sin(PI/3.0 -phi2 + PI)/sqrt(3))*ref_retraction;
    cap_lengths[5] = (2.0/3.0 + cos(PI/3.0 -phi2 + 5.0*PI/3.0)/3.0 + sin(PI/3.0 -phi2 + 5.0*PI/3.0)/sqrt(3))*ref_retraction;
}
void inv_kinematics_moments_axial_compression(double k1,double phi1, double k2, double phi2){
    double R = 30.0, s1 = 698.50,s2 = 698.50;
    
    double ref_retraction1 = -s1 * R *k1; //divide by 3 for 0 proximal curvature
    double ref_retraction2 = -s2 * R * k2;
    
    // double ref_retraction1 = -10.0;//-25.0; //divide by 3 for 0 proximal curvature
    // double ref_retraction2 = -50.0;

    // double ref_retraction1 = -50; //divide by 3 for 0 proximal curvature
    // double ref_retraction2 = -30;

    Matrix<6> relative_lengths;
    phi1=wrapAngle(phi1);
    phi2 = wrapAngle(phi2);
    relative_lengths = relative_bending(inv_A, phi1,phi2);
    cap_lengths[0] = relative_lengths(0)*ref_retraction1;
    cap_lengths[1] = relative_lengths(1)*ref_retraction1;
    cap_lengths[2] = relative_lengths(2)*ref_retraction1;
    cap_lengths[3] = relative_lengths(3)*ref_retraction2;
    cap_lengths[4] = relative_lengths(4)*ref_retraction2;
    cap_lengths[5] = relative_lengths(5)*ref_retraction2;

}

void inv_kinematics_moments_axial_compression_path_coupling(double k1,double phi1, double k2, double phi2){
    double R = 30.0, s1 = 698.50,s2 = 698.50;
    
    double ref_retraction1 = -s1 * R *k1; //divide by 3 for 0 proximal curvature
    double ref_retraction2 = -s2 * R * k2;
    
    // double ref_retraction1 = -10.0;//-25.0; //divide by 3 for 0 proximal curvature
    // double ref_retraction2 = -50.0;

    // double ref_retraction1 = -50; //divide by 3 for 0 proximal curvature
    // double ref_retraction2 = -30;

    Matrix<6> lengths;
    phi1=wrapAngle(phi1);
    phi2 = wrapAngle(phi2);
    lengths = bending(inv_A_proximal, inv_A_distal, phi1,phi2, ref_retraction1,ref_retraction2);
    for(int i =0; i<6;i++){
        cap_lengths[i] = lengths(i);
    }

}

void print_that_stuff(){
        // Serial.write(buf2);
        Serial.print(S.substring(0,S.length()-1));
        Serial.print(',');
        Serial.print(capstans[3].get_length(),0);
        Serial.print(',');
        Serial.print(capstans[4].get_length(),0);
        Serial.print(',');
        Serial.print(capstans[5].get_length(),0);
        // Serial.println(capstans[5].get_length(),0);
        Serial.print(',');
        Serial.println(64 - Serial.availableForWrite());
        S = "";
}

void loop(){
    // uint8_t distal_str_length = readline_buffer2(Serial2.read(), buf2, 80);

    // if (distal_str_length> 0) {
    //     // strcpy(distal_orientation,buf2);
    //     print_that_stuff();
    //     // distal_orientation[0] = 0;

    // }

    // if (IMU_data_available){
    //     S = Serial2.readStringUntil('\n');
    //     print_that_stuff();
    //     IMU_data_available = false;
    // }

    // uint8_t data_in_buffer = Serial2.available();
    // if (data_in_buffer > 0){
    //     Serial.print(data_in_buffer);
    //     Serial.print(',');
    //     IMU_data_available = true;
    // }

    current_time = millis();
    //Next waypoint in terms of tendon length
    if(current_time - last_waypoint_time >= waypoint_interval){
        last_waypoint_time = current_time;
        last_setpoint_time = 0;//This should result in avoiding the discrepancies in the arduino millis function 
        
        target_phi2 = target_phi2 + 2.0*PI/3.0;

        target_phi1 = PI + target_phi2;

        interpolate_jones_space(0.0,target_phi1,0.0,target_phi2);
        // interpolate_jones_space(0.25*PI/698.50,target_phi1,0.25*PI/698.50,target_phi2); //0.35 for VC_manu just so that it stays within the cage; 
        // interpolate_jones_space(0.0,target_phi1,0.4*PI/698.50,target_phi2);
        // interpolate_jones_space(0.35*PI/698.50,target_phi1,0.0,target_phi2);

    }
    
    current_time = millis();//If this line is commented, the setpoint set length that needed to be executed in between last_waypoint_time and current_time won't be executed and would be delayed by loop time.
    
    if(current_time - last_setpoint_time >= setpoint_interval){
        last_setpoint_time = current_time;
        
        //Setpoint counting logic
        if(num_setpoints>1){

            k1 = k1 + delta_k1;     phi1 = phi1 + delta_phi1;
            k2 = k2 + delta_k2;     phi2 = phi2 + delta_phi2;

            // inv_kinematics_moments(k1,phi1,k2,phi2);//Calculates the length difference at the capstans
            // inv_kinematics_moments_section2(k1,phi1,k2,phi2);//Comment out FK when using this! results in jerks otherwise
            // inv_kinematics_jones(k1,phi1,k2,phi2);
            // inv_kinematics_not_moment(k1,phi1,k2,phi2);
            
            // inv_kinematics_moments_axial_compression(k1,phi1,k2,phi2);
            inv_kinematics_moments_axial_compression_path_coupling(k1,phi1,k2,phi2);
            // inv_kinematics_manu(k1,phi1,k2,phi2);
            
            for(int i=0;i<6;i++){
                capstans[i].set_length(cap_lengths[i]);
            }
            num_setpoints--;
        }
        // else if (num_setpoints==1){
        //     num_setpoints--; //Need this to prevent setting a setpoint above the target waypoint when the w/s division is exact.
        // }
        
    }

    for(int i=0;i<6;i++){
        capstans[i].update();
    }
    
    // if (current_time - last_print_time >= 1000){
    //     last_print_time = current_time;
    //     loop_debug_info();
    // }  
}
*/