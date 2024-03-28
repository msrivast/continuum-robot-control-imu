//6-29-22: Added IMU relay functionality
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "capstan.h"
#include <math.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

char buf2[80];
char distal_orientation[80];
bool IMU_data_available = false;
String S;
int readline_buffer2(int readch, char *buffer, int len) {
    static int pos = 0;
    int rpos;

    if (readch > 0) {
        switch (readch) {
            case '\r': // Ignore CR
                break;
            case '\n': // Return on new-line
                rpos = pos;
                pos = 0;  // Reset position index ready for next time
                return rpos;
            default:
                if (pos < len-1) {
                    buffer[pos++] = readch;
                    buffer[pos] = 0;
                }
        }
    }
    return 0;
}

#define heaviside(x) (x>=0)

// Matrix<6, 6                  > inv_A ={-0.0000,   -0.6667,    0.3333,         0,    0.6667,   -0.3333,
//                                         0.5774,    0.3333,    0.3333,   -0.5774,   -0.3333,   -0.3333,
//                                        -0.5774,    0.3333,    0.3333,    0.5774,   -0.3333,   -0.3333,
//                                              0,         0,         0,    0.5774,   -0.3333,    0.3333,
//                                              0,         0,         0,         0,    0.6667,    0.3333,
//                                              0,         0,         0,   -0.5774,   -0.3333,    0.3333};

Matrix<6, 6                  > inv_A ={-0.0000,   -0.6667,    0.3333,         0,        0,          0,
                                        0.5774,    0.3333,    0.3333,         0,        0,          0,
                                       -0.5774,    0.3333,    0.3333,         0,        0,          0,
                                             0,         0,         0,    0.5774,   -0.3333,    0.3333,
                                             0,         0,         0,         0,    0.6667,    0.3333,
                                             0,         0,         0,   -0.5774,   -0.3333,    0.3333};

Matrix<3, 3> inv_A_proximal =    {-0.0000,   -0.6667,    0.3333,
                                   0.5774,    0.3333,    0.3333,
                                  -0.5774,    0.3333,    0.3333};

Matrix<3, 3> inv_A_distal =       {0.5774,   -0.3333,    0.3333,
                                        0,    0.6667,    0.3333,
                                  -0.5774,   -0.3333,    0.3333};

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
unsigned long last_waypoint_time = 0,waypoint_interval = 3000;
unsigned long last_setpoint_time = 0,setpoint_interval = 50;

unsigned long last_print_time = 0;

byte waypoint_number = 0,num_setpoints=0;

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
  Serial.begin(76800);
  // Serial.print("Serial started.");
  Serial2.begin(76800);
  while (Serial2.available()) // Make sure the serial RX buffer is empty
    Serial2.read();

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
    double R = 30.0, s1 = 698.50, s2 = 698.50;
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
    double R = 30.0, s1 = 698.50,s2 = 698.50;
// swap cap_lengths 2 - 1 and 3 - 5 to change the rotation direction
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

    double R = 30.0, s1 = 698.50,s2 = 698.50;
    cap_lengths[0] = -2.0*s1 * R * k1 * cos(phi1);
    cap_lengths[1] = -2.0*s1 * R * k1 * cos((2.0 * PI / 3.0) - phi1);
    cap_lengths[2] = -2.0*s1 * R * k1 * cos((4.0 * PI / 3.0) - phi1);
//   k2=0;
    cap_lengths[4] = -2.0*s1 * R * k1 * cos(PI - phi1) -2.0*s2 * R * k2 * cos(PI - phi2);
//   k2=0;
    cap_lengths[3] = -2.0*s1 * R * k1 * cos((PI / 3.0) - phi1) -2.0*s2 * R * k2 * cos((PI / 3.0) - phi2);
    cap_lengths[5] = -2.0*s1 * R * k1 * cos((5.0 * PI / 3.0) - phi1) -2.0*s2 * R * k2 * cos((5.0 * PI / 3.0) - phi2);
}

void inv_kinematics_moments_section2(double k1,double phi1, double k2, double phi2){
    double R = 30.0, s1 = 698.50,s2 = 698.50;
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
    double R = 30.0, s1 = 698.50,s2 = 698.50;
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
