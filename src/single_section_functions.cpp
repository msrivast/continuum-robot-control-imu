//09/09/22: Retained only the necessary code for single section operation. Modified variables. e.g. phi1 and phi2 replaced with phi.
//6-29-22: Added IMU relay functionality

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "capstan.h"
#include <math.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;


Matrix<3, 3> inv_A_distal =       {0.5774,   -0.3333,    0.3333,
                                        0,    0.6667,    0.3333,
                                  -0.5774,   -0.3333,    0.3333};

// capstan constructor sets starting encoder position as zero
Capstan capstans[] = {Capstan(M4_DIR, M4_PWM, M4_FLT, M4_CS, MUX_ADDR, M4_ENC, KP, KI, KD, CIRCUMFERENCE),Capstan(M5_DIR, M5_PWM, M5_FLT, M5_CS, MUX_ADDR, M5_ENC, KP, KI, KD, CIRCUMFERENCE),Capstan(M6_DIR, M6_PWM, M6_FLT, M6_CS, MUX_ADDR, M6_ENC, KP, KI, KD, CIRCUMFERENCE)};
// double *setpoint_lengths;
// setpoint_lengths = new double[6];
double setpoint_lengths[3] = {0};
double cap_lengths[3] = {0};

double k=0,phi=0;
double delta_k=0,delta_phi=0;
double target_k=0, target_phi=0;

unsigned long current_time;
unsigned long last_waypoint_time = 0,waypoint_interval = 3000;
unsigned long last_setpoint_time = 0,setpoint_interval = 50;

unsigned long last_print_time = 0;

byte waypoint_number = 0,num_setpoints=0; //cant have more than 255 setpoints and waypoints(if used)

inline double wrapAngle( double angle )
{
    double twoPi = 2.0 * 3.141592865358979;
    return angle - twoPi * floor( angle / twoPi );
}

void setup_communication() {
    // change PIn 3 pwm frequency to 31kHz
    TCCR3B = (TCCR3B & B11111000) | B00000001;
    // change PIn 6 pwm frequency to 31kHz
    TCCR4B = (TCCR4B & B11111000) | B00000001;
    // change PIn 9 pwm frequency to 31kHz
    TCCR2B = (TCCR2B & B11111000) | B00000001;
    // change PIn 12 pwm frequency to 31kHz
    TCCR1B = (TCCR1B & B11111000) | B00000001;
    // change PIn 44, 45 pwm frequency to 31kHz
    TCCR5B = (TCCR5B & B11111000) | B00000001;
  // initialize i2c and UART

  Wire.begin();
  Wire.setClock(10000L);
//  TWBR = 158;  // 12.5 kHz 
// TWSR |= bit (TWPS0);  // change prescaler
  Serial.begin(115200);
  // Serial.print("Serial started.");
}

void loop_debug_info(){
    Serial.print(millis());
    Serial.print('\t');
    Serial.print(num_setpoints);
    Serial.print('\t');
    Serial.print(cap_lengths[0]);
    Serial.print('\t');
    Serial.print(capstans[0].get_length(),1);
    Serial.print('\t');
    Serial.print(cap_lengths[1]);
    Serial.print('\t');
    Serial.print(capstans[1].get_length(),1);
    Serial.print('\t');
    Serial.print(cap_lengths[2]);
    Serial.print('\t');
    Serial.print(capstans[2].get_length(),1);
    Serial.print('\t');
    Serial.print(k);
    Serial.print('\t');
    Serial.print(wrapAngle(phi));
    Serial.println('\t');
    // Serial.print('\t');
    // Serial.print(capstan_5.get_PId_output(),1);
    // Serial.print('\t');
    // Serial.println(capstan_5.get_current());
}

inline Matrix<2> convert_to_curvatures(double angle){
    //helper function to go from k_phi to k_x,k_z 
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

// inline Matrix<3> bending(Matrix<3,3> inv_A, boolean k_zero, double phi, double ac_third_entry, double ref_ret){
//     //helper function to do matrix multiplication
//     if (k_zero){
//         Matrix<3> config = {0.0,0.0,ac_third_entry};
//         return inv_A * config  * ref_ret;
//     }
//     double k_x,k_z;
//     Matrix<2> section_curvature;    
    
//     section_curvature = convert_to_curvatures(phi);
    
//     k_x = section_curvature(0);
//     k_z = section_curvature(1);

//     Matrix<3> config = {k_x,k_z,ac_third_entry};
    
//     Matrix<3> lengths = inv_A * config * ref_ret;

//     return lengths;
// }
inline Matrix<3> bending(Matrix<3,3> inv_A, double first_entry, double phi, double ac_third_entry, double ref_ret){
    //helper function to do matrix multiplication
    double k_x,k_z;
    Matrix<2> section_curvature;    
    
    section_curvature = convert_to_curvatures(phi);
    
    k_x = first_entry * section_curvature(0);
    k_z = first_entry * section_curvature(1);

    Matrix<3> config = {k_x,k_z,ac_third_entry};
    
    Matrix<3> lengths = inv_A * config * ref_ret;

    return lengths;
}
void interpolate_jones_space(double target_k, double target_phi)
{

    target_phi=wrapAngle(target_phi);//fmod is a bad idea in C++

    num_setpoints = waypoint_interval/setpoint_interval; //Assumption is that the division is exact

    delta_k = (double)(target_k - k)/(num_setpoints - 1);
    delta_phi = (double)wrapAngle(target_phi - phi)/(num_setpoints - 1);
}

void inv_kinematics_jones_curved(double k,double phi){
    // Assumes CC
    double R = 30.0, s = 1397.0;
    double ref_retraction = -s * R * k;
// swap cap_lengths 2 - 1 and 3 - 5 to change the rotation direction
    cap_lengths[0] = ref_retraction * cos((PI / 3.0) - phi);
    cap_lengths[1] = ref_retraction * cos(PI - phi);
    cap_lengths[2] = ref_retraction * cos((5.0 * PI / 3.0) - phi);
}

void inv_kinematics_jones_piecewise_linear(double k,double phi){
    // Assumes CC
    double R = 30.0, s = 1397.0, n = 10.0;
    double same_multiplier = 2.0*n*sin(s*k/(2.0*n));
// swap cap_lengths 2 - 1 and 3 - 5 to change the rotation direction
    cap_lengths[0] = same_multiplier * (1/k - R*cos((PI / 3.0) - phi)) - s;
    cap_lengths[1] = same_multiplier * (1/k - R*cos(PI - phi)) - s;
    cap_lengths[2] = same_multiplier * (1/k - R*cos((5.0 * PI / 3.0) - phi)) - s;
}


// void inv_kinematics_moments_axial_compression(boolean k_zero,double phi, double ac_third_entry, double ref_ret){
//     Matrix<3> lengths;
    
//     phi=wrapAngle(phi);

//     lengths = bending(inv_A_distal,k_zero, phi, ac_third_entry, ref_ret);
    
//     for(int i =0; i<3;i++){
//         cap_lengths[i] = lengths(i);
//     }

// }

void inv_kinematics_moments_axial_compression(double first_entry,double phi, double ac_third_entry, double ref_ret){
    Matrix<3> lengths;
    
    phi=wrapAngle(phi);

    lengths = bending(inv_A_distal,first_entry, phi, ac_third_entry, ref_ret);
    
    for(int i =0; i<3;i++){
        cap_lengths[i] = lengths(i);
    }

}

void setup(){
    setup_communication();
    //Capstans will hold position at startup
    for (int i=0;i<3;i++){
      capstans[i].init();
    }
  
}
