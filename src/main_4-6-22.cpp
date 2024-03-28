
//Major changes: phi2 is considered the dominant phi. Anything that's superflous has been removed.
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "capstan.h"
#include <math.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

#define heaviside(x) (x>=0)

// Matrix<6, 6,Array<6,6,double>> inv_A ={-0.0000,   -0.6667,    0.3333,         0,    0.6667,   -0.3333,
//                                         0.5774,    0.3333,    0.3333,   -0.5774,   -0.3333,   -0.3333,
//                                        -0.5774,    0.3333,    0.3333,    0.5774,   -0.3333,   -0.3333,
//                                              0,         0,         0,    0.5774,   -0.3333,    0.3333,
//                                              0,         0,         0,         0,    0.6667,    0.3333,
//                                              0,         0,         0,   -0.5774,   -0.3333,    0.3333};

Matrix<6, 6                  > inv_A ={-0.0000,   -0.6667,    0.3333,         0,    0.6667,   -0.3333,
                                        0.5774,    0.3333,    0.3333,   -0.5774,   -0.3333,   -0.3333,
                                       -0.5774,    0.3333,    0.3333,    0.5774,   -0.3333,   -0.3333,
                                             0,         0,         0,    0.5774,   -0.3333,    0.3333,
                                             0,         0,         0,         0,    0.6667,    0.3333,
                                             0,         0,         0,   -0.5774,   -0.3333,    0.3333};
// capstan constructor sets starting encoder position as zero
Capstan capstans[] = {Capstan(M1_DIR, M1_PWM, M1_FLT, M1_CS, MUX_ADDR, M1_ENC, KP, KI, KD, CIRCUMFERENCE),Capstan(M2_DIR, M2_PWM, M2_FLT, M2_CS, MUX_ADDR, M2_ENC, KP, KI, KD, CIRCUMFERENCE),Capstan(M3_DIR, M3_PWM, M3_FLT, M3_CS, MUX_ADDR, M3_ENC, KP, KI, KD, CIRCUMFERENCE),Capstan(M4_DIR, M4_PWM, M4_FLT, M4_CS, MUX_ADDR, M4_ENC, KP, KI, KD, CIRCUMFERENCE),Capstan(M5_DIR, M5_PWM, M5_FLT, M5_CS, MUX_ADDR, M5_ENC, KP, KI, KD, CIRCUMFERENCE),Capstan(M6_DIR, M6_PWM, M6_FLT, M6_CS, MUX_ADDR, M6_ENC, KP, KI, KD, CIRCUMFERENCE)};
// double *setpoint_lengths;
// setpoint_lengths = new double[6];
double setpoint_lengths[6] = {0};
double cap_lengths[6] = {0};
double k1=0,phi1=0,k2=0,phi2=0;
double delta_k1=0,delta_phi1=0,delta_k2=0,delta_phi2=0;
double target_k1=0, target_phi1=0, target_k2=0, target_phi2=0;
// double setpoint_length_5=0,cap_length_5=0;
unsigned long current_time;
unsigned long last_waypoint_time = 0,waypoint_interval = 1500;
unsigned long last_setpoint_time = 0,setpoint_interval = 50;

unsigned long last_print_time = 0;

byte waypoint_number = 0,num_setpoints=0;

inline double wrapAngle( double angle )
{
    double twoPi = 2.0 * 3.141592865358979;
    return angle - twoPi * floor( angle / twoPi );
}
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
    Matrix<6> config= {k_x_1,k_z_1,6,k_x_2,k_z_2,2};//Next steps in research!!!!
    // Matrix<6> config= {0,0,6,k_x_2,k_z_2,2};//Next steps in research!!!!
    return inv_A*config;
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
    Serial.print(k1);
    Serial.print('\t');
    Serial.print(wrapAngle(phi1));
    Serial.print('\t');

    Serial.print(cap_lengths[3]);
    Serial.print('\t');
    Serial.print(capstans[3].get_length(),1);
    Serial.print('\t');
    Serial.print(cap_lengths[4]);
    Serial.print('\t');
    Serial.print(capstans[4].get_length(),1);
    Serial.print('\t');
    Serial.print(cap_lengths[5]);
    Serial.print('\t');
    Serial.print(capstans[5].get_length(),1);
    Serial.print('\t');
    Serial.print(k2);
    Serial.print('\t');
    Serial.println(wrapAngle(phi2));
    // Serial.print('\t');
    // Serial.print(capstan_5.get_PId_output(),1);
    // Serial.print('\t');
    // Serial.println(capstan_5.get_current());
}

void interpolate_length(double *target_lengths){
    num_setpoints = waypoint_interval/setpoint_interval; //Assumption is that the division is exact

    // cap_length_5 = capstan_5.get_length();//get_length here works on the last update time which is lesser than loop time -> the next best thing to polling for current length
    for(int i=0;i<6;i++){
        setpoint_lengths[i] = (double) (target_lengths[i] - capstans[i].get_length())/(num_setpoints-1);
    }
}

double get_curvature(double l1, double l2, double l3){
    double R=30.0;
    return 2.0*sqrt(pow(l1,2) + pow(l2,2) + pow(l3,2) - l1*l2 - l2*l3 - l1*l3)/(double)(R*(l1+l2+l3));
}



void interpolate_jones_space(double target_k1, double target_phi1, double target_k2, double target_phi2)
{

//-figure out current configuration
//-interpolate in the configuration space
//-unlike linear interpolation, there are no setpoint lengths but the inv kinematic transformation of the interpolated configurations 

    target_phi1=wrapAngle(target_phi1);//fmod is a bad idea in C++
    target_phi2=wrapAngle(target_phi2);

    num_setpoints = waypoint_interval/setpoint_interval; //Assumption is that the division is exact
//Forward kinematics to determine the current shape
    double R = 30.0, s1 = 660.00, s2 = 710.00;
    // for(int i=0;i<3;i++){
    //     cap_lengths[i] = s1 + capstans[i].get_length();//trick to not have to add s1 to individual cap_lengths!
    // }
    // k1 = get_curvature(cap_lengths[0],cap_lengths[1],cap_lengths[2]);
    // phi1 = wrapAngle(atan2(sqrt(3)*(cap_lengths[2]-cap_lengths[1]),(cap_lengths[2] + cap_lengths[1] - 2*cap_lengths[0])));//atan2 returns 0 for atan2(0,0)
    
    // Serial.print("target phi1: ");
    // Serial.print(target_phi1);
    // Serial.print('\t');
    // Serial.print("phi1: ");
    // Serial.print(phi1);
    // Serial.print('\t');
    
    // int j = 2;
    // for(int i=3;i<6;i++){
    //     cap_lengths[i] = capstans[i].get_length() + s1*k1*R*cos((double)(i-j)*PI/3 - phi1);
    //     j--;
    // }
    // if (cap_lengths[4] == 0 && k1 == 0){
    //   phi2=0; //takes care of the nan in the 0/0 division within atan2
    // }else{
    //   phi2 = wrapAngle(atan2((cap_lengths[3] + R*s1*k1*cos(PI/3 - phi1))/(cap_lengths[4] + R*s1*k1*cos(PI- phi1)) + 1/2,-sqrt(3)/2)); //Can't use IK at this trajectory due to ratio of cosines problem
    // }
    
    // phi2 = wrapAngle(atan2(sqrt(3)*(cap_lengths[5]-cap_lengths[3]),(2*cap_lengths[4] - cap_lengths[3] - cap_lengths[5])));//this is different from phi1 and hence can't be turned into a function

    // Serial.print("target phi2: ");
    // Serial.print(target_phi2);
    // Serial.print('\t');
    // Serial.print("phi2: ");
    // Serial.print(phi2);
    // Serial.print('\t');
   
    // k2 = get_curvature(cap_lengths[3]+s2,cap_lengths[4]+s2,cap_lengths[5]+s2);
//k and phi1 deltas for inv_kinematics() to act upon
    delta_k1 = (double)(target_k1 - k1)/(num_setpoints - 1);
    delta_phi1 = (double)wrapAngle(target_phi1 - phi1)/(num_setpoints - 1);

    // Serial.print("delta_phi1: ");
    // Serial.print(delta_phi1);
    // Serial.print('\t');

    delta_k2 = (double)(target_k2 - k2)/(num_setpoints - 1);
    delta_phi2 = (double)wrapAngle(target_phi2 - phi2)/(num_setpoints - 1);
    
    // Serial.print("delta_phi2: ");
    // Serial.println(delta_phi2);

}

void inv_kinematics_jones(double k1,double phi1, double k2, double phi2){
    double R = 30.0, s1 = 660.00,s2 = 710.00;
//     cap_lengths[0] = -s1 * R * k1 * cos(phi1);
//     cap_lengths[2] = -s1 * R * k1 * cos((2.0 * PI / 3.0) - phi1);
//     cap_lengths[1] = -s1 * R * k1 * cos((4.0 * PI / 3.0) - phi1));
// //   k2=0;
//     cap_lengths[4] = -s1 * R * k1 * cos(PI - phi1) -s2 * R * k2 * cos(PI - phi2);
// //   k2=0;
//     cap_lengths[5] = -s1 * R * k1 * cos((PI / 3.0) - phi1) -s2 * R * k2 * cos((PI / 3.0) - phi2);
//     cap_lengths[3] = -s1 * R * k1 * cos((5.0 * PI / 3.0) - phi1) -s2 * R * k2 * cos((5.0 * PI / 3.0) - phi2);
    cap_lengths[0] = -s1 * R * k1 * cos(phi1);
    cap_lengths[1] = -s1 * R * k1 * cos((2.0 * PI / 3.0) - phi1);
    cap_lengths[2] = -s1 * R * k1 * cos((4.0 * PI / 3.0) - phi1);
//   k2=0;
    cap_lengths[4] = -s1 * R * k1 * cos(PI - phi1) -s2 * R * k2 * cos(PI - phi2);
//   k2=0;
    cap_lengths[3] = -s1 * R * k1 * cos((PI / 3.0) - phi1) -s2 * R * k2 * cos((PI / 3.0) - phi2);
    cap_lengths[5] = -s1 * R * k1 * cos((5.0 * PI / 3.0) - phi1) -s2 * R * k2 * cos((5.0 * PI / 3.0) - phi2);
}

void inv_kinematics_manu(double k1,double phi1, double k2, double phi2){
    //Euler curve based VC kinematics

    double R = 30.0, s1 = 660.00,s2 = 710.00;
//     cap_lengths[0] = -s1 * R * k1 * cos(phi1);
//     cap_lengths[2] = -s1 * R * k1 * cos((2.0 * PI / 3.0) - phi1);
//     cap_lengths[1] = -s1 * R * k1 * cos((4.0 * PI / 3.0) - phi1));
// //   k2=0;
//     cap_lengths[4] = -s1 * R * k1 * cos(PI - phi1) -s2 * R * k2 * cos(PI - phi2);
// //   k2=0;
//     cap_lengths[5] = -s1 * R * k1 * cos((PI / 3.0) - phi1) -s2 * R * k2 * cos((PI / 3.0) - phi2);
//     cap_lengths[3] = -s1 * R * k1 * cos((5.0 * PI / 3.0) - phi1) -s2 * R * k2 * cos((5.0 * PI / 3.0) - phi2);
    cap_lengths[0] = -2.0*s1 * R * k1 * cos(phi1);
    cap_lengths[1] = -2.0*s1 * R * k1 * cos((2.0 * PI / 3.0) - phi1);
    cap_lengths[2] = -2.0*s1 * R * k1 * cos((4.0 * PI / 3.0) - phi1);
//   k2=0;
    cap_lengths[4] = -2.0*s1 * R * k1 * cos(PI - phi1) -2.0*s2 * R * k2 * cos(PI - phi2);
//   k2=0;
    cap_lengths[3] = -2.0*s1 * R * k1 * cos((PI / 3.0) - phi1) -2.0*s2 * R * k2 * cos((PI / 3.0) - phi2);
    cap_lengths[5] = -2.0*s1 * R * k1 * cos((5.0 * PI / 3.0) - phi1) -2.0*s2 * R * k2 * cos((5.0 * PI / 3.0) - phi2);
}

void inv_kinematics_not_moment(double k1,double phi1, double k2, double phi2){
    // Three tendon retraction pivot model for distal section
    double R = 30.0, s1 = 660.00,s2 = 710.00;
    double ref_retraction = -s2 * R * k2; // CAUTION: While this is used as a measurement of the tendon lengths in the tendon midplanes, this is used as a proxy for the actual theta measurment.
    // Actually, retraction is a function of sin(theta) while this suggests it is a linear function of theta. This doesn't matter in our particular implementation when 
    // we use theta in a relative context but this retraction doesn't capture the real behavior at the capstans with respect to theta.

//     cap_lengths[0] = -s1 * R * k1 * cos(phi1);
//     cap_lengths[2] = -s1 * R * k1 * cos((2.0 * PI / 3.0) - phi1);
//     cap_lengths[1] = -s1 * R * k1 * cos((4.0 * PI / 3.0) - phi1));
// //   k2=0;
//     cap_lengths[4] = -s1 * R * k1 * cos(PI - phi1) -s2 * R * k2 * cos(PI - phi2);
// //   k2=0;
//     cap_lengths[5] = -s1 * R * k1 * cos((PI / 3.0) - phi1) -s2 * R * k2 * cos((PI / 3.0) - phi2);
//     cap_lengths[3] = -s1 * R * k1 * cos((5.0 * PI / 3.0) - phi1) -s2 * R * k2 * cos((5.0 * PI / 3.0) - phi2);
    
    // cap_lengths[0] = -s1 * R * k1 * cos(phi1);
    // cap_lengths[1] = -s1 * R * k1 * cos((2.0 * PI / 3.0) - phi1);
    // cap_lengths[2] = -s1 * R * k1 * cos((4.0 * PI / 3.0) - phi1);
//   k2=0;
    // cap_lengths[4] = -s1 * R * k1 * cos(PI - phi1) - s2 * R * k2 * cos(PI - phi2); 2/3+ cos(pi/3-phi)/3 + sin(pi/3-phi)/sqrt(3)
//   k2=0;
    // cap_lengths[3] = -s1 * R * k1 * cos((PI / 3.0) - phi1) -s2 * R * k2 * cos((PI / 3.0) - phi2);
    // cap_lengths[5] = -s1 * R * k1 * cos((5.0 * PI / 3.0) - phi1) -s2 * R * k2 * cos((5.0 * PI / 3.0) - phi2);

    cap_lengths[3] = (2.0/3.0 + cos(PI/3.0 -phi2 + PI/3.0)/3.0 + sin(PI/3.0 -phi2 + PI/3.0)/sqrt(3))*ref_retraction;
    cap_lengths[4] = (2.0/3.0 + cos(PI/3.0 -phi2 + PI)/3.0 + sin(PI/3.0 -phi2 + PI)/sqrt(3))*ref_retraction;
    cap_lengths[5] = (2.0/3.0 + cos(PI/3.0 -phi2 + 5.0*PI/3.0)/3.0 + sin(PI/3.0 -phi2 + 5.0*PI/3.0)/sqrt(3))*ref_retraction;
}
void inv_kinematics_moments_axial_compression(double k1,double phi1, double k2, double phi2){
    double R = 30.0, s1 = 660.00,s2 = 710.00;
    double ref_retraction1 = -s1 * R *k1/6; //divide by 3 for 0 proximal curvature
    double ref_retraction2 = -s2 * R * k2;
    
    // double ref_retraction1 = -10.0;//-25.0; //divide by 3 for 0 proximal curvature
    // double ref_retraction2 = -50.0;

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
void inv_kinematics_moments_section2(double k1,double phi1, double k2, double phi2){
    double R = 30.0, s1 = 660.00,s2 = 710.00;
    double ref_retraction = -s2 * R * k2;
    phi2 = wrapAngle(phi2);

    cap_lengths[0] = 0;
    cap_lengths[1] = 0;
    cap_lengths[2] = 0;

    if(PI/3.0 <= phi2 && phi2 <= PI){
        cap_lengths[3] = (-sin(phi2)/(cos(phi2)*sin(phi2 - PI/3.0) - cos(phi2 - PI/3.0)*sin(phi2)))*ref_retraction;
        cap_lengths[4] = (-sin(phi2 - PI/3.0)/(cos(phi2)*sin(phi2 - PI/3.0) - cos(phi2 - PI/3.0)*sin(phi2)))*ref_retraction;
    }else if (PI < phi2 && phi2 <= 5.0*PI/3.0){
        cap_lengths[4] = (-sin(phi2 - (5.0*PI)/3.0)/(cos(phi2)*sin(phi2 - (5.0*PI)/3.0) - cos(phi2 - (5.0*PI)/3.0)*sin(phi2)))*ref_retraction;
        cap_lengths[5] = (-sin(phi2)/(cos(phi2)*sin(phi2 - (5.0*PI)/3.0) - cos(phi2- (5.0*PI)/3.0)*sin(phi2)))*ref_retraction;
    }else{
        cap_lengths[3] = (sin(phi2 - (5.0*PI)/3.0)/(cos(phi2 - PI/3.0)*sin(phi2 - (5.0*PI)/3.0) - cos(phi2 - (5.0*PI)/3.0)*sin(phi2 - PI/3.0)))*ref_retraction;
        cap_lengths[5] = (-sin(phi2 - PI/3.0)/(cos(phi2 - PI/3.0)*sin(phi2 - (5.0*PI)/3.0) - cos(phi2 - (5.0*PI)/3.0)*sin(phi2 - PI/3.0)))*ref_retraction;
    }
}
void inv_kinematics_moments(double k1,double phi1, double k2, double phi2){
    double R = 30.0, s1 = 660.00,s2 = 710.00;
    double ref_retraction1 = -s1 * R *k1;
    double ref_retraction2 = -s2 * R * k2;
    phi1=wrapAngle(phi1);
    phi2 = wrapAngle(phi2);
    cap_lengths[0] = ((cos(phi1 - PI/6)*sin(phi1 + PI/6) - cos(phi1 + PI/6)*sin(phi1 - PI/6) + cos(phi1 + PI/6)*heaviside(phi1 - (4*PI)/3)*sin(phi1 - PI/6) - heaviside((2*PI)/3 - phi1)*heaviside(phi1)*cos(phi1 - PI/6)*sin(phi1 + PI/6))/(sin(phi1)*heaviside(phi1 - (4*PI)/3)*sin(phi1 - PI/6)*sin(phi1 + PI/6) + cos(phi1)*cos(phi1 + PI/6)*heaviside(phi1 - (4*PI)/3)*sin(phi1 - PI/6) - heaviside((2*PI)/3 - phi1)*cos(phi1)*heaviside(phi1)*cos(phi1 - PI/6)*sin(phi1 + PI/6) + heaviside((4*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/6)*heaviside(phi1 - (2*PI)/3)*sin(phi1 + PI/6) - heaviside((4*PI)/3 - phi1)*cos(phi1)*cos(phi1 + PI/6)*heaviside(phi1 - (2*PI)/3)*sin(phi1 - PI/6) - heaviside((2*PI)/3 - phi1)*heaviside(phi1)*sin(phi1)*sin(phi1 - PI/6)*sin(phi1 + PI/6)) - (cos(phi1 - PI/6)*sin(phi1 + PI/6) - cos(phi1 + PI/6)*sin(phi1 - PI/6))/(sin(phi1)*heaviside(phi1 - (4*PI)/3)*sin(phi1 - PI/6)*sin(phi1 + PI/6) + cos(phi1)*cos(phi1 + PI/6)*heaviside(phi1 - (4*PI)/3)*sin(phi1 - PI/6) - heaviside((2*PI)/3 - phi1)*cos(phi1)*heaviside(phi1)*cos(phi1 - PI/6)*sin(phi1 + PI/6) + heaviside((4*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/6)*heaviside(phi1 - (2*PI)/3)*sin(phi1 + PI/6) - heaviside((4*PI)/3 - phi1)*cos(phi1)*cos(phi1 + PI/6)*heaviside(phi1 - (2*PI)/3)*sin(phi1 - PI/6) - heaviside((2*PI)/3 - phi1)*heaviside(phi1)*sin(phi1)*sin(phi1 - PI/6)*sin(phi1 + PI/6)))*ref_retraction1;

    cap_lengths[1] = ((cos(phi1)*cos(phi1 + PI/6) + sin(phi1)*sin(phi1 + PI/6) - heaviside((4*PI)/3 - phi1)*cos(phi1)*cos(phi1 + PI/6)*heaviside(phi1 - (2*PI)/3) - heaviside((2*PI)/3 - phi1)*heaviside(phi1)*sin(phi1)*sin(phi1 + PI/6))/(sin(phi1)*heaviside(phi1 - (4*PI)/3)*sin(phi1 - PI/6)*sin(phi1 + PI/6) + cos(phi1)*cos(phi1 + PI/6)*heaviside(phi1 - (4*PI)/3)*sin(phi1 - PI/6) - heaviside((2*PI)/3 - phi1)*cos(phi1)*heaviside(phi1)*cos(phi1 - PI/6)*sin(phi1 + PI/6) + heaviside((4*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/6)*heaviside(phi1 - (2*PI)/3)*sin(phi1 + PI/6) - heaviside((4*PI)/3 - phi1)*cos(phi1)*cos(phi1 + PI/6)*heaviside(phi1 - (2*PI)/3)*sin(phi1 - PI/6) - heaviside((2*PI)/3 - phi1)*heaviside(phi1)*sin(phi1)*sin(phi1 - PI/6)*sin(phi1 + PI/6)) - (cos(phi1)*cos(phi1 + PI/6) + sin(phi1)*sin(phi1 + PI/6))/(sin(phi1)*heaviside(phi1 - (4*PI)/3)*sin(phi1 - PI/6)*sin(phi1 + PI/6) + cos(phi1)*cos(phi1 + PI/6)*heaviside(phi1 - (4*PI)/3)*sin(phi1 - PI/6) - heaviside((2*PI)/3 - phi1)*cos(phi1)*heaviside(phi1)*cos(phi1 - PI/6)*sin(phi1 + PI/6) + heaviside((4*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/6)*heaviside(phi1 - (2*PI)/3)*sin(phi1 + PI/6) - heaviside((4*PI)/3 - phi1)*cos(phi1)*cos(phi1 + PI/6)*heaviside(phi1 - (2*PI)/3)*sin(phi1 - PI/6) - heaviside((2*PI)/3 - phi1)*heaviside(phi1)*sin(phi1)*sin(phi1 - PI/6)*sin(phi1 + PI/6)))*ref_retraction1;
    
    cap_lengths[2] = ((cos(phi1)*cos(phi1 - PI/6) + sin(phi1)*sin(phi1 - PI/6) - sin(phi1)*heaviside(phi1 - (4*PI)/3)*sin(phi1 - PI/6) - heaviside((4*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/6)*heaviside(phi1 - (2*PI)/3))/(sin(phi1)*heaviside(phi1 - (4*PI)/3)*sin(phi1 - PI/6)*sin(phi1 + PI/6) + cos(phi1)*cos(phi1 + PI/6)*heaviside(phi1 - (4*PI)/3)*sin(phi1 - PI/6) - heaviside((2*PI)/3 - phi1)*cos(phi1)*heaviside(phi1)*cos(phi1 - PI/6)*sin(phi1 + PI/6) + heaviside((4*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/6)*heaviside(phi1 - (2*PI)/3)*sin(phi1 + PI/6) - heaviside((4*PI)/3 - phi1)*cos(phi1)*cos(phi1 + PI/6)*heaviside(phi1 - (2*PI)/3)*sin(phi1 - PI/6) - heaviside((2*PI)/3 - phi1)*heaviside(phi1)*sin(phi1)*sin(phi1 - PI/6)*sin(phi1 + PI/6)) - (cos(phi1)*cos(phi1 - PI/6) + sin(phi1)*sin(phi1 - PI/6))/(sin(phi1)*heaviside(phi1 - (4*PI)/3)*sin(phi1 - PI/6)*sin(phi1 + PI/6) + cos(phi1)*cos(phi1 + PI/6)*heaviside(phi1 - (4*PI)/3)*sin(phi1 - PI/6) - heaviside((2*PI)/3 - phi1)*cos(phi1)*heaviside(phi1)*cos(phi1 - PI/6)*sin(phi1 + PI/6) + heaviside((4*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/6)*heaviside(phi1 - (2*PI)/3)*sin(phi1 + PI/6) - heaviside((4*PI)/3 - phi1)*cos(phi1)*cos(phi1 + PI/6)*heaviside(phi1 - (2*PI)/3)*sin(phi1 - PI/6) - heaviside((2*PI)/3 - phi1)*heaviside(phi1)*sin(phi1)*sin(phi1 - PI/6)*sin(phi1 + PI/6)))*ref_retraction1;

    //fixed lengths without 1
    // cap_lengths[3] = ((cos(phi2)*sin(phi2 - (5*PI)/3) - cos(phi2 - (5*PI)/3)*sin(phi2))/(heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - (5*PI)/3) - cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - PI)*sin(phi2 - (5*PI)/3) + heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(phi2 - PI) + cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)*sin(phi2 - PI/3) - cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)) + (cos(phi2 - (5*PI)/3)*sin(phi2) - cos(phi2)*sin(phi2 - (5*PI)/3) + cos(phi2)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - (5*PI)/3) + heaviside(PI/3 - phi2)*cos(phi2)*sin(phi2 - (5*PI)/3) - cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(PI - phi2)*heaviside(phi2 - PI/3))/(heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - (5*PI)/3) - cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - PI)*sin(phi2 - (5*PI)/3) + heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(phi2 - PI) + cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)*sin(phi2 - PI/3) - cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)))*ref_retraction2;

    // cap_lengths[4] = ((cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3))/(heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - (5*PI)/3) - cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - PI)*sin(phi2 - (5*PI)/3) + heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(phi2 - PI) + cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)*sin(phi2 - PI/3) - cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)) - (cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + cos(phi2 - (5*PI)/3)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*heaviside(phi2 - PI)*sin(phi2 - (5*PI)/3))/(heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - (5*PI)/3) - cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - PI)*sin(phi2 - (5*PI)/3) + heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(phi2 - PI) + cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)*sin(phi2 - PI/3) - cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)))*ref_retraction2;

    // cap_lengths[5] = (- (cos(phi2 - PI/3)*sin(phi2) - cos(phi2)*sin(phi2 - PI/3) + cos(phi2)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + heaviside(PI/3 - phi2)*cos(phi2)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*sin(phi2)*heaviside(phi2 - PI))/(heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - (5*PI)/3) - cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - PI)*sin(phi2 - (5*PI)/3) + heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(phi2 - PI) + cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)*sin(phi2 - PI/3) - cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)) - (cos(phi2)*sin(phi2 - PI/3) - cos(phi2 - PI/3)*sin(phi2))/(heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - (5*PI)/3) - cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - PI)*sin(phi2 - (5*PI)/3) + heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(phi2 - PI) + cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)*sin(phi2 - PI/3) - cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)))*ref_retraction2;

//fixed lengths with 1

    cap_lengths[3] = ((cos(phi1)*sin(phi1 - (5*PI)/3) - cos(phi1 - (5*PI)/3)*sin(phi1))/(heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - (5*PI)/3) - cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - PI)*sin(phi1 - (5*PI)/3) + heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(phi1 - PI) + cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)*sin(phi1 - PI/3) - cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)) + (cos(phi1 - (5*PI)/3)*sin(phi1) - cos(phi1)*sin(phi1 - (5*PI)/3) + cos(phi1)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - (5*PI)/3) + heaviside(PI/3 - phi1)*cos(phi1)*sin(phi1 - (5*PI)/3) - cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(PI - phi1)*heaviside(phi1 - PI/3))/(heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - (5*PI)/3) - cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - PI)*sin(phi1 - (5*PI)/3) + heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(phi1 - PI) + cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)*sin(phi1 - PI/3) - cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)))*ref_retraction1 + ((cos(phi2)*sin(phi2 - (5*PI)/3) - cos(phi2 - (5*PI)/3)*sin(phi2))/(heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - (5*PI)/3) - cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - PI)*sin(phi2 - (5*PI)/3) + heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(phi2 - PI) + cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)*sin(phi2 - PI/3) - cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)) + (cos(phi2 - (5*PI)/3)*sin(phi2) - cos(phi2)*sin(phi2 - (5*PI)/3) + cos(phi2)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - (5*PI)/3) + heaviside(PI/3 - phi2)*cos(phi2)*sin(phi2 - (5*PI)/3) - cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(PI - phi2)*heaviside(phi2 - PI/3))/(heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - (5*PI)/3) - cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - PI)*sin(phi2 - (5*PI)/3) + heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(phi2 - PI) + cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)*sin(phi2 - PI/3) - cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)))*ref_retraction2;

    cap_lengths[4] = ((cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3))/(heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - (5*PI)/3) - cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - PI)*sin(phi1 - (5*PI)/3) + heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(phi1 - PI) + cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)*sin(phi1 - PI/3) - cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)) - (cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + cos(phi1 - (5*PI)/3)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*heaviside(phi1 - PI)*sin(phi1 - (5*PI)/3))/(heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - (5*PI)/3) - cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - PI)*sin(phi1 - (5*PI)/3) + heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(phi1 - PI) + cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)*sin(phi1 - PI/3) - cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)))*ref_retraction1 + ((cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3))/(heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - (5*PI)/3) - cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - PI)*sin(phi2 - (5*PI)/3) + heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(phi2 - PI) + cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)*sin(phi2 - PI/3) - cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)) - (cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + cos(phi2 - (5*PI)/3)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*heaviside(phi2 - PI)*sin(phi2 - (5*PI)/3))/(heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - (5*PI)/3) - cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - PI)*sin(phi2 - (5*PI)/3) + heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(phi2 - PI) + cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)*sin(phi2 - PI/3) - cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)))*ref_retraction2;

    cap_lengths[5] = (- (cos(phi1 - PI/3)*sin(phi1) - cos(phi1)*sin(phi1 - PI/3) + cos(phi1)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + heaviside(PI/3 - phi1)*cos(phi1)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*sin(phi1)*heaviside(phi1 - PI))/(heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - (5*PI)/3) - cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - PI)*sin(phi1 - (5*PI)/3) + heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(phi1 - PI) + cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)*sin(phi1 - PI/3) - cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)) - (cos(phi1)*sin(phi1 - PI/3) - cos(phi1 - PI/3)*sin(phi1))/(heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - (5*PI)/3) - cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - PI)*sin(phi1 - (5*PI)/3) + heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(phi1 - PI) + cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)*sin(phi1 - PI/3) - cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)))*ref_retraction1 + (- (cos(phi2 - PI/3)*sin(phi2) - cos(phi2)*sin(phi2 - PI/3) + cos(phi2)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + heaviside(PI/3 - phi2)*cos(phi2)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*sin(phi2)*heaviside(phi2 - PI))/(heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - (5*PI)/3) - cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - PI)*sin(phi2 - (5*PI)/3) + heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(phi2 - PI) + cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)*sin(phi2 - PI/3) - cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)) - (cos(phi2)*sin(phi2 - PI/3) - cos(phi2 - PI/3)*sin(phi2))/(heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*sin(phi2 - (5*PI)/3) - heaviside(PI/3 - phi2)*cos(phi2)*cos(phi2 - (5*PI)/3)*sin(phi2 - PI/3) + cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - (5*PI)/3) - cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(phi2 - (5*PI)/3)*sin(phi2 - PI/3) - heaviside((5*PI)/3 - phi2)*cos(phi2)*cos(phi2 - PI/3)*heaviside(phi2 - PI)*sin(phi2 - (5*PI)/3) + heaviside((5*PI)/3 - phi2)*cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(phi2 - PI) + cos(phi2)*cos(phi2 - (5*PI)/3)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)*sin(phi2 - PI/3) - cos(phi2 - PI/3)*cos(phi2 - (5*PI)/3)*sin(phi2)*heaviside(PI - phi2)*heaviside(phi2 - PI/3)))*ref_retraction2;


// Section 1's retraction that need to be added to section 2's cap_lengths - fixed
// ((cos(phi1)*sin(phi1 - (5*PI)/3) - cos(phi1 - (5*PI)/3)*sin(phi1))/(heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - (5*PI)/3) - cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - PI)*sin(phi1 - (5*PI)/3) + heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(phi1 - PI) + cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)*sin(phi1 - PI/3) - cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)) + (cos(phi1 - (5*PI)/3)*sin(phi1) - cos(phi1)*sin(phi1 - (5*PI)/3) + cos(phi1)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - (5*PI)/3) + heaviside(PI/3 - phi1)*cos(phi1)*sin(phi1 - (5*PI)/3) - cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(PI - phi1)*heaviside(phi1 - PI/3))/(heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - (5*PI)/3) - cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - PI)*sin(phi1 - (5*PI)/3) + heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(phi1 - PI) + cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)*sin(phi1 - PI/3) - cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)))*ref_retraction1

// ((cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3))/(heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - (5*PI)/3) - cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - PI)*sin(phi1 - (5*PI)/3) + heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(phi1 - PI) + cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)*sin(phi1 - PI/3) - cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)) - (cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + cos(phi1 - (5*PI)/3)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*heaviside(phi1 - PI)*sin(phi1 - (5*PI)/3))/(heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - (5*PI)/3) - cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - PI)*sin(phi1 - (5*PI)/3) + heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(phi1 - PI) + cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)*sin(phi1 - PI/3) - cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)))*ref_retraction1

// (- (cos(phi1 - PI/3)*sin(phi1) - cos(phi1)*sin(phi1 - PI/3) + cos(phi1)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + heaviside(PI/3 - phi1)*cos(phi1)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*sin(phi1)*heaviside(phi1 - PI))/(heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - (5*PI)/3) - cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - PI)*sin(phi1 - (5*PI)/3) + heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(phi1 - PI) + cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)*sin(phi1 - PI/3) - cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)) - (cos(phi1)*sin(phi1 - PI/3) - cos(phi1 - PI/3)*sin(phi1))/(heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*sin(phi1 - (5*PI)/3) - heaviside(PI/3 - phi1)*cos(phi1)*cos(phi1 - (5*PI)/3)*sin(phi1 - PI/3) + cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - (5*PI)/3) - cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(phi1 - (5*PI)/3)*sin(phi1 - PI/3) - heaviside((5*PI)/3 - phi1)*cos(phi1)*cos(phi1 - PI/3)*heaviside(phi1 - PI)*sin(phi1 - (5*PI)/3) + heaviside((5*PI)/3 - phi1)*cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(phi1 - PI) + cos(phi1)*cos(phi1 - (5*PI)/3)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)*sin(phi1 - PI/3) - cos(phi1 - PI/3)*cos(phi1 - (5*PI)/3)*sin(phi1)*heaviside(PI - phi1)*heaviside(phi1 - PI/3)))*ref_retraction1

}

void setup(){
    setup_communication();
    //Capstans will hold position at startup
    for (int i=0;i<6;i++){
      capstans[i].init();
    }
  
}

void loop(){
    current_time = millis();
    //Next waypoint in terms of tendon length
    if(current_time - last_waypoint_time >= waypoint_interval){
        last_waypoint_time = current_time;
        last_setpoint_time = 0;//This should result in avoiding the discrepancies in the arduino millis function 
        
        //Waypoint logic
        // if (waypoint_number == 0) {
        //     interpolate_length(-15);//Target length -mm. This function breaks down the target tendon length into the lengths to be moved per setpoint_interval
        //     waypoint_number = 1;
        // }else{
        //     interpolate_length(0);
        //     waypoint_number = 0;
        // }
        // target_phi1=fmod(target_phi1+PI/3,2.0*PI);
        // target_phi2=fmod(target_phi1+PI,2.0*PI);//target_phi2 should initially be PI but we'll wait and see.
        target_phi2 = target_phi2 + PI/3;
        // Serial.print("Global variable target phi1: ");
        // Serial.print(target_phi1);
        // Serial.print('\t');
        target_phi1 = PI + target_phi2;
        // Serial.print("Global variable target phi2: ");
        // Serial.println(target_phi2);
        // interpolate_jones_space(0.5*PI/660.00,target_phi1,0.5*PI/710.00,target_phi2); //0.35 for VC_manu; 
        // interpolate_jones_space(0.8*PI/660.00,target_phi1,0.8*PI/710.00,target_phi2);
        // interpolate_jones_space(0.0,target_phi1,0.5*PI/710.00,target_phi2);
        interpolate_jones_space(0.0,target_phi1,0.0,target_phi2);
        // interpolate_jones_space(0.35*PI/660.00,target_phi1,0.0,target_phi2);
        // interpolate_jones_space(0.0,target_phi1,0.5*PI*(1.0+0.5*sin(7.0*phi2))/710.00,target_phi2);


    }
    
    //current_time = millis();//If this line is commented, the setpoint set length that needed to be executed in between last_waypoint_time and current_time won't be executed and would be delayed by loop time.
    if(current_time - last_setpoint_time >= setpoint_interval){
        last_setpoint_time = current_time;
        
        //Setpoint counting logic
        if(num_setpoints>1){
            // cap_length_5 = cap_length_5 + setpoint_length_5;//Assumption is that the robot has reached the setpoint_length in the setpoint_interval
            // capstan_5.set_length(cap_length_5);
            k1 = k1 + delta_k1;     phi1 = phi1 + delta_phi1;
            k2 = k2 + delta_k2;     phi2 = phi2 + delta_phi2;
            // inv_kinematics_mod_cosine(k1,phi1,k2,phi2);//Calculates the length difference at the capstans
            // inv_kinematics_moments_mod(k1,phi1,k2,phi2);//Calculates the length difference at the capstans
            // inv_kinematics_moments(k1,phi1,k2,phi2);//Calculates the length difference at the capstans
            
            // inv_kinematics_jones(k1,phi1,k2,phi2);
            // inv_kinematics_not_moment(k1,phi1,k2,phi2);
            
            inv_kinematics_moments_axial_compression(k1,phi1,k2,phi2);
            
            // inv_kinematics_manu(k1,phi1,k2,phi2);
            
            // inv_kinematics_moments_section2(k1,phi1,k2,phi2);//Comment out FK when using this! results in jerks otherwise

            // elliptical_tendon_lengths(k1,phi1,k2,phi2);
            // quadratic_tendon_lengths(k1,phi1,k2,phi2);
            // quadratic_elliptical_tendon_lengths(k1,phi1,k2,phi2);

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
    
    if (current_time - last_print_time >= 100){
        last_print_time = current_time;
        loop_debug_info();
    }
    
    //loop_debug_info();
    
}


/*
Current problem with selecting an setpoint_interval time as the basic quantum of length setting - 
No matter what the target tendon length is, if the waypoint interval is the same, then the same # of setpoints(waypoint_interval/setpoint_interval) will be created.
This means that the EE moves very slowly when the new waypoint is closer to it and faster when it is away from it.
The alternative is to specify a constant joint velocity - each section should have a different joint velocity for a constant EE velocity.
# of setpoints = target tendon length/constant tendon lenth/setpoint_interval
*/

/* TO DO
1. change the location of the last_waypoint_time = current_time & last_setpoint_time = current_time line to the bottom of the if loop becuase while it is at the top, the next waypoint arrives sooner than the waypoint interval by the amount of time it takes to process the waypoint logic code.
2. Wire.setclock
3. make interpolate return a pointer to the setpoint_lengths
4. polling for current length during interpolation AND/OR set_length()
5. Motor 1 keeps making a squeaky noise and the current sensors need to be fitted with a larger capacitor
6. Don't mess with I term in the PID without looking at adjusting the sample time frequency the same as the loop time 
*/

/*
#include <Arduino.h>
#include <wire.h>
#include "config.h"
#include "capstan.h"
#include <math.h>

// capstan constructor sets starting encoder position as zero
Capstan capstan_1(M1_DIR, M1_PWM, M1_FLT, M1_CS, MUX_ADDR, M1_ENC, KP, KI, KD, CIRCUMFERENCE);
Capstan capstan_2(M2_DIR, M2_PWM, M2_FLT, M2_CS, MUX_ADDR, M2_ENC, KP, KI, KD, CIRCUMFERENCE);
Capstan capstan_3(M3_DIR, M3_PWM, M3_FLT, M3_CS, MUX_ADDR, M3_ENC, KP, KI, KD, CIRCUMFERENCE);
Capstan capstan_4(M4_DIR, M4_PWM, M4_FLT, M4_CS, MUX_ADDR, M4_ENC, KP, KI, KD, CIRCUMFERENCE);
Capstan capstan_5(M5_DIR, M5_PWM, M5_FLT, M5_CS, MUX_ADDR, M5_ENC, KP, KI, KD, CIRCUMFERENCE);
Capstan capstan_6(M6_DIR, M6_PWM, M6_FLT, M6_CS, MUX_ADDR, M6_ENC, KP, KI, KD, CIRCUMFERENCE);

void setup_communication() {
    // change pin 3 pwm frequency to 31kHz
    TCCR3B = (TCCR3B & B11111000) | B00000001;
    // change pin 6 pwm frequency to 31kHz
    TCCR4B = (TCCR4B & B11111000) | B00000001;
    // change pin 9 pwm frequency to 31kHz
    TCCR2B = (TCCR2B & B11111000) | B00000001;
    // change pin 12 pwm frequency to 31kHz
    TCCR1B = (TCCR1B & B11111000) | B00000001;
    // change pin 44, 45 pwm frequency to 31kHz
    TCCR5B = (TCCR5B & B11111000) | B00000001;
  // initialize i2c and UART

  Wire.begin();
  Wire.setClock(10000L);
//  TWBR = 158;  // 12.5 kHz 
// TWSR |= bit (TWPS0);  // change prescaler
  Serial.begin(115200);
  // Serial.print("Serial started.");
}
Capstan *current;

// void process_command()
// {
// 	if (Serial.available() > 0) {
//     // Serial.println("Ready to process commands!");
// 		char command = Serial.read();
// 		if (command == '1') {
//       current = &capstan_1;
// 		}
//         else if (command == '2') {
//       current = &capstan_2;
// 		}else if (command == '3') {
//       current = &capstan_3;
// 		}else if (command == '4') {
//       current = &capstan_4;
// 		}else if (command == '5') {
//       current = &capstan_5;
// 		}else if (command == '6') {
//       current = &capstan_6;
// 		}
//         else if (command == 'e') {
//       current->set_angle(current->get_angle() + 2.5);
// 		}else if (command == 'r') {
//       current->set_angle(current->get_angle() - 2.5);
// 		}
// 	}
//   Serial.print("Capstan 1: ");
//   Serial.print(capstan_1.get_angle());
//   Serial.print(" Capstan 2: ");
//   Serial.print(capstan_2.get_angle());
//   Serial.print(" Capstan 3: ");
//   Serial.print(capstan_3.get_angle());
//   Serial.print(" Capstan 4: ");
//   Serial.print(capstan_4.get_angle());
//   Serial.print(" Capstan 5: ");
//   Serial.print(capstan_5.get_angle());
//   Serial.print(" Capstan 6: ");
//   Serial.println(capstan_6.get_angle());
//   // Serial.print(capstan_1.get_current());
//   // Serial.print(" ");
//   // Serial.print(capstan_2.get_current());
//   // Serial.print(" ");
//   // Serial.print(capstan_3.get_current());
//   // Serial.print(" ");
//   // Serial.print(capstan_4.get_current());
//   // Serial.print(" ");
//   // Serial.print(capstan_5.get_current());
//   // Serial.print(" ");
//   // Serial.println(capstan_6.get_current());
// }

void process_command() //length based
{
	if (Serial.available() > 0) {
    // Serial.println("Ready to process commands!");
		char command = Serial.read();
		if (command == '1') {
      current = &capstan_1;
		}
        else if (command == '2') {
      current = &capstan_2;
		}else if (command == '3') {
      current = &capstan_3;
		}else if (command == '4') {
      current = &capstan_4;
		}else if (command == '5') {
      current = &capstan_5;
		}else if (command == '6') {
      current = &capstan_6;
		}
        else if (command == 'e') {
      current->set_length(current->get_length() + 2.5);
		}else if (command == 'r') {
      current->set_length(current->get_length() - 2.5);
		}
	}
  Serial.print("Capstan 1: ");
  Serial.print(capstan_1.get_length());
  Serial.print(" Capstan 2: ");
  Serial.print(capstan_2.get_length());
  Serial.print(" Capstan 3: ");
  Serial.print(capstan_3.get_length());
  Serial.print(" Capstan 4: ");
  Serial.print(capstan_4.get_length());
  Serial.print(" Capstan 5: ");
  Serial.print(capstan_5.get_length());
  Serial.print(" Capstan 6: ");
  Serial.println(capstan_6.get_length());
}



void setup() {
    setup_communication();
    
    capstan_1.init();
    capstan_2.init();
    capstan_3.init();
    capstan_4.init();
    capstan_5.init();
    capstan_6.init();

}

void loop() {
    process_command();
    capstan_1.update();

    capstan_2.update();

    capstan_3.update();

    capstan_4.update();

    capstan_5.update();

    capstan_6.update();
}
*/