/*
//10-26-22: Added IMU functionality; used global r and s
            Changed the setpoint logic to reach the waypoint exactly at the final setpoint interval and not one interval and some
            small time before. Assumption is that the waypoint_interval/setpoint_interval is exact and a byte. 
//09/09/22: Retained only the necessary code for single section operation. Modified variables. e.g. phi1 and phi2 replaced with phi.
//6-29-22: Added IMU relay functionality
*/
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "capstan.h"
#include <math.h>
#include <BasicLinearAlgebra.h>
#include "attitude.h"

using namespace BLA;
#define SERIAL_PORT Serial2

Attitude attitude;

// Matrix<3, 3> inv_A_distal =       {0.5774,   -0.3333,    0.3333,
//                                         0,    0.6667,    0.3333,
//                                   -0.5774,   -0.3333,    0.3333};

Matrix<3, 3> inv_A_distal =       {0.3333,    0.5774,    0.3333,
                                  -0.6667,       0.0,    0.3333,
                                   0.3333,   -0.5774,    0.3333};

// capstan constructor sets starting encoder position as zero
Capstan capstans[] = {Capstan(M4_DIR, M4_PWM, M4_FLT, M4_CS, MUX_ADDR, M4_ENC, KP, KI, KD, CIRCUMFERENCE),Capstan(M5_DIR, M5_PWM, M5_FLT, M5_CS, MUX_ADDR, M5_ENC, KP, KI, KD, CIRCUMFERENCE),Capstan(M6_DIR, M6_PWM, M6_FLT, M6_CS, MUX_ADDR, M6_ENC, KP, KI, KD, CIRCUMFERENCE)};
double cap_lengths[3] = {0};

double k=0,phi=0;
double delta_k=0,delta_phi=0;
double target_k=0, target_phi=0;

unsigned long current_time;
unsigned long last_waypoint_time = 0,waypoint_interval = 4000; //remember that num setpoints is a uint8 so max 256
unsigned long last_setpoint_time = 0,setpoint_interval = 50;

unsigned long last_print_time = 0;

byte waypoint_number = 0,num_setpoints=0; //cant have more than 255 setpoints and waypoints(if used)



//*** Non blocking reading variables***
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
boolean newData = false;

boolean printData = false;

inline double wrapAngle( double angle )
{ 
    double twoPi = 2.0 * 3.141592865358979;
    return angle - twoPi * floor( angle / twoPi ); //[0 2pi)
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
  SERIAL_PORT.begin(115200);
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (SERIAL_PORT.available() > 0 && newData == false) {
    rc = SERIAL_PORT.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx++] = rc;
        // if (~(rc == '-' || isDigit(rc) || rc == '.' || rc ==',')) //Could still get confused by multiple successive , or . but still helpful
        //   serial_garbage = true;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData() {      // split the data into its parts

  // char * strtokIndx; // this is used by strtok() as an index

  // strtokIndx = strtok(tempChars,",");      // get the first part - the string
  // attitude.Roll = atoi(strtokIndx); 

  // strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  // attitude.Pitch = atoi(strtokIndx);     // convert this part to an integer

  // strtokIndx = strtok(NULL, ",");
  // attitude.Yaw = atoi(strtokIndx);     // convert this part to an integer

  printData = true;
}


void updateAttitude(){
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
    parseData(); 
  }
}

// void loop_debug_info(){
//     Serial.print(millis());
//     Serial.print(';');
//     Serial.print(num_higher_level_setpoints);
//     Serial.print(';');
//     // Serial.print(cap_lengths[0],1);
//     // Serial.print('\t');
//     Serial.print(capstans[0].get_length(),0);
//     Serial.print(';');
//     // Serial.print(cap_lengths[1],1);
//     // Serial.print('\t');
//     Serial.print(capstans[1].get_length(),0);
//     Serial.print(';');
//     // Serial.print(cap_lengths[2],1);
//     // Serial.print('\t');
//     Serial.print(capstans[2].get_length(),0);
//     Serial.print(';');
//     Serial.print(ss_attitude.theta*180.0/PI,0);
//     Serial.print('\t');
//     Serial.println(ss_attitude.phi*180.0/PI,0);
//     // Serial.println('\t');
//       //Debug prints
    
//     // Serial.print(attitude.Roll,0);
//     // Serial.print('\t');
//     // Serial.print(attitude.Pitch,0);
//     // Serial.print('\t');
//     // Serial.print(attitude.Yaw,0);
//     // Serial.print('\t');
//     // Serial.write(tempChars);
//     // Serial.println('\t');
    
//     // Serial.println(Serial.availableForWrite());

//     // Serial.print('\t');
//     // Serial.print(capstan_5.get_pid_output(),1);
//     // Serial.print('\t');
//     // Serial.println(capstan_5.get_current());

//     printData = false;
// }

inline Matrix<2> convert_to_curvatures(double angle){
    //helper function to go from k_phi to k_x,k_z 
    double k_x,k_z;
    Matrix<2> section_curvature;
    angle = wrapAngle(angle);
    // if (angle >= PI){
    //     k_x = abs(cos(-angle-PI/2.0));
    // }else{
    //     k_x = -abs(cos(-angle-PI/2.0));
    // }
    // if (angle<PI/2.0 || angle>3.0*PI/2.0){
    //     k_z = -abs(tan(-angle-PI/2.0)*k_x);
    // }else{
    //     k_z = abs(tan(-angle-PI/2)*k_x);
    // }
    k_x = cos(angle);
    k_z = sin(angle);
    section_curvature(0) = k_x;
    section_curvature(1) = k_z;
    
    return section_curvature;
}

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

    // delta_k = (double)(target_k - k)/(num_setpoints - 1);
    // delta_phi = (double)wrapAngle(target_phi - phi)/(num_setpoints - 1);
    delta_k = (double)(target_k - k)/(num_setpoints);
    delta_phi = (double)wrapAngle(target_phi - phi)/(num_setpoints);
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
