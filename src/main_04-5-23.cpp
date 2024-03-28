// #include <manual_adjustment.cpp>
/*

3-15-23: PURPOSE: Exploring current sensing as torque proxy
-have a very fast loop
-output retracting tendon length and polled current
-retract a total of 90 mm in 30mm increments every 3 seconds

Built on top of the DUET comm code
*/

// #if 0
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "capstan.h"
#include <math.h>

#define SERIAL_PORT Serial3

uint8_t setpoint_interval = 50;
uint32_t current_time = 0,last_setpoint_time = 0, last_waypoint_time = 0,last_print_time = 0;

uint8_t counter;
uint16_t num_setpoints = 0;

//*** Non blocking reading variables***
const byte numChars = 80;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
boolean newData = false;
boolean dataReady = false;

float g_code_lengths[6] = {0.0}; //These are delta lengths or length as the capstan sees it
float delta_lengths[6] = {0.0};
float current_lengths[6] = {0.0};
uint32_t time_to_dest = 3000;

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

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    g_code_lengths[0] = atof(strtokIndx);
    // SERIAL_PORT.println(g_code_lengths[0]);
  for(int i = 1; i<6;i++)
  {
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    g_code_lengths[i] = atof(strtokIndx);
    // SERIAL_PORT.println(g_code_lengths[i]);
  }
  
  
  strtokIndx = strtok(NULL, ","); 
  time_to_dest = atoi(strtokIndx);
    // SERIAL_PORT.println(time_to_dest);
  dataReady = true;
}

void updateGcodeLengths(){
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


// capstan constructor sets starting encoder position as zero
Capstan capstans[] = {Capstan(M1_DIR, M1_PWM, M1_FLT, M1_CS, MUX_ADDR, M1_ENC, KP, KI, KD, CIRCUMFERENCE),Capstan(M2_DIR, M2_PWM, M2_FLT, M2_CS, MUX_ADDR, M2_ENC, KP, KI, KD, CIRCUMFERENCE),Capstan(M3_DIR, M3_PWM, M3_FLT, M3_CS, MUX_ADDR, M3_ENC, KP, KI, KD, CIRCUMFERENCE),Capstan(M4_DIR, M4_PWM, M4_FLT, M4_CS, MUX_ADDR, M4_ENC, KP, KI, KD, CIRCUMFERENCE),Capstan(M5_DIR, M5_PWM, M5_FLT, M5_CS, MUX_ADDR, M5_ENC, KP, KI, KD, CIRCUMFERENCE),Capstan(M6_DIR, M6_PWM, M6_FLT, M6_CS, MUX_ADDR, M6_ENC, KP, KI, KD, CIRCUMFERENCE)};


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
  SERIAL_PORT.begin(9600);
  while(SERIAL_PORT.available()){
    SERIAL_PORT.read();
  }
  
}
// Capstan *current;


void print_stuff(){

    Serial.print(millis());
    Serial.print('\t');
    Serial.print(num_setpoints);
    Serial.print('\t');
    Serial.print(current_lengths[5],1);
    Serial.print('\t');
    Serial.print(capstans[5].get_pid_output(),1);
    Serial.print('\t');
    Serial.println(capstans[5].get_current());

//   Serial.print("Rec: ");
//   Serial.write(receivedChars);
//   Serial.print(" ");
//   Serial.print("Curr: ");
//   for (int i = 0; i<5; i++){
//     Serial.print(current_lengths[i],0);
//     Serial.print(", ");
//   }
//   Serial.println(current_lengths[5],0);


  // Serial.print("C1: ");
  // Serial.print(current_lengths[0],0);
  // Serial.print(" C2: ");
  // Serial.print(current_lengths[1],0);
  // Serial.print(" C3: ");
  // Serial.print(current_lengths[2],0);
  // Serial.print(" C4: ");
  // Serial.print(current_lengths[3],0);
  // Serial.print(" C5: ");
  // Serial.print(current_lengths[4],0);
  // Serial.print(" C6: ");
  // Serial.println(current_lengths[5],0);
  // Serial.print("C1: ");
  // Serial.print(capstans[0].get_length(),0);
  // Serial.print(" C2: ");
  // Serial.print(capstans[1].get_length(),0);
  // Serial.print(" C3: ");
  // Serial.print(capstans[2].get_length(),0);
  // Serial.print(" C4: ");
  // Serial.print(capstans[3].get_length(),0);
  // Serial.print(" C5: ");
  // Serial.print(capstans[4].get_length(),0);
  // Serial.print(" C6: ");
  // Serial.println(capstans[5].get_length(),0);
//   Serial.print(" ");
//   Serial.write(buf2);
//   Serial.println();
//   Serial.println(S.substring(0,S.length()-1));
}

void interpolate_length(){

    num_setpoints = time_to_dest/setpoint_interval; //Assumption is that the division is exact
    // cap_length_5 = capstan_5.get_length();//get_length here works on the last update time which is lesser than loop time -> the next best thing to polling for current length
    for(int i=5;i<6;i++){
        current_lengths[i] = capstans[i].get_length();
        delta_lengths[i] = (double) (g_code_lengths[i] - current_lengths[i])/num_setpoints;
        // SERIAL_PORT.println(delta_lengths[i]);
    }
}
void process_command() //length based
{   
    if (dataReady){
        interpolate_length();
        print_stuff();
        last_setpoint_time = 0; //added 03/25/23 after seeing that the new waypoint only takes effect once the current setpoint period is over.
        dataReady = false;
    }

}



void setup() {
    setup_communication();
    
    for (int i=5;i<6;i++){
      capstans[i].init();
    }
  
    // Serial.println("Arduino setup complete!");

}

void loop() {
    
    updateGcodeLengths(); //target lengths

    process_command(); //Also prints after each new command is processed

    current_time = millis();
    //Next waypoint in terms of tendon length
    if(current_time - last_waypoint_time >= time_to_dest){
        last_waypoint_time = current_time;
        last_setpoint_time = 0; // added 03/25/23 after seeing that the new waypoint only takes effect once the current setpoint period is over.

        if (counter<3){
            g_code_lengths[5] += -30;
            counter++;
        }else{
            g_code_lengths[5] += 30;
            counter++;
            if (counter==6)counter=0;
        }

        interpolate_length();

    }
    current_time = millis();
    if(current_time - last_setpoint_time >= setpoint_interval){
      //NOTE: if this executes before but close to when the next waypoint hits, the target current lengths are going to be different from those that the interpolation used. This can cause waypoint transitions to be especially violent when
      //direction of tendon motion is change. It gets fixed by the next setpoint but the violence is observable because the length change almost doubles. Setting the last_setpoint_time to 0, as soon as the next waypoint hits is imperative.
        last_setpoint_time = current_time;

        if(num_setpoints > 0){
            for(int i=5;i<6;i++){
                current_lengths[i] += delta_lengths[i];
                capstans[i].set_length(current_lengths[i]);//target current lengths; current_lengths is a misnomer becuase it contains the current setpoint targer. When a new waypoint hits, the current length is pinned to the "actual" current length
                //from the previous loop's encoder poll.
            }
            num_setpoints--;
        }   
    }

    
    for(int i=5;i<6;i++){
        capstans[i].update();
    }

    current_time = millis();
    if (current_time - last_print_time >= 5){
        last_print_time = current_time;
        print_stuff();
    }
}
// #endif