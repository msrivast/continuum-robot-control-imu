#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "capstan.h"
#include <math.h>
#include "shape_space_attitude.h"

#define SERIAL_PORT Serial2

uint32_t current_time=0, last_print_time=0;

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

Shape_space_attitude ss_attitude;
double phi_offset = 0.9428;//0.903; //0.973//4.0*PI/3.0; //TBD

char buf2[80];
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
  SERIAL_PORT.begin(115200);
  while(SERIAL_PORT.available()){
    SERIAL_PORT.read();
  }
  
}
Capstan *current;


void print_stuff(){
  Serial.print("C1: ");
  Serial.print(capstan_1.get_length(),1);
  Serial.print(" C2: ");
  Serial.print(capstan_2.get_length(),1);
  Serial.print(" C3: ");
  Serial.print(capstan_3.get_length(),1);
  Serial.print(" C4: ");
  Serial.print(capstan_4.get_length(),1);
  Serial.print(" C5: ");
  Serial.print(capstan_5.get_length(),1);
  Serial.print(" C6: ");
  Serial.print(capstan_6.get_length(),1);
  Serial.print(" ");
//   Serial.write(buf2);
//   Serial.println();
    Serial.print(ss_attitude.theta*180.0/PI,1);
    Serial.print(';');
    Serial.println(ss_attitude.phi*180.0/PI,0);
//   Serial.println(S.substring(0,S.length()-1));
}
void parseDataThetaPhi(){
    
  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars,",");      // get the first part - the string
  ss_attitude.theta = atof(strtokIndx);    // convert this part to an float

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  ss_attitude.phi = wrapAngle(atof(strtokIndx) - phi_offset);     // convert this part to an float

  printData = true;

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

void updateSSAttitude(){
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
    
  }
}
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
      current->set_length(current->get_length() + 1.0);
		}else if (command == 'r') {
      current->set_length(current->get_length() - 1.0);
		}
	}
    // uint8_t buff = SERIAL_PORT.available();
    // Serial.println(buff);
    // if (buff>0){
    //     S = SERIAL_PORT.readStringUntil('\n');
    //     Serial.println(SERIAL_PORT.available());
    //     print_stuff();
    // }
    

}



void setup() {
    setup_communication();
    
    capstan_1.init();
    capstan_2.init();
    capstan_3.init();
    capstan_4.init();
    capstan_5.init();
    capstan_6.init();

    Serial.println("Arduino setup complete!");

}

void loop() {
    // while(SERIAL_PORT.available()>0){ //non blocking read
    //     uint8_t distal_str_length = readline_buffer2(SERIAL_PORT.read(), buf2, 80);
    //     if (distal_str_length> 0) {
    //         // strcpy(distal_orientation,buf2);
    //         print_stuff();
    //         // distal_orientation[0] = 0;
    //     }
    // }
    updateSSAttitude();

    process_command();
    capstan_1.update();

    capstan_2.update();

    capstan_3.update();

    capstan_4.update();

    capstan_5.update();

    capstan_6.update();

    // if (printData){
    //     print_stuff();
    //     printData = false;
    // }
    current_time = millis();
    if (current_time - last_print_time >= 200){
        last_print_time = current_time;
        print_stuff();
    }  
}
