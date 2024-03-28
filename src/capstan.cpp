#include "capstan.h"

Capstan::Capstan(uint8_t dir, uint8_t pwm, uint8_t flt, uint8_t cs, uint8_t mux, uint8_t enc, double kp, double ki, double kd, double circumference)
    : dir_(dir),
    pwm_(pwm),
    flt_(flt),
    cs_(cs),
    mux_(mux),
    enc_(enc),
    kp_(kp),
    ki_(ki),
    kd_(kd),
    circumference_(circumference),
    previous_angle(0.0),
    current_angle(0.0),
    revolutions(0),
    pid(&input_, &output_, &setpoint_, kp_, ki_, kd_, DIRECT) {
        pinMode(dir_, OUTPUT);
        pinMode(pwm_, OUTPUT);
        pinMode(flt_, INPUT);
        pinMode(cs_, INPUT);
    }

int Capstan::get_current() {
    return analogRead(cs_);
}

double Capstan::get_pid_output(){
    return output_;
}

void Capstan::switch_pid(boolean state) {
    if (state==0){
        pid.SetMode(MANUAL);
        output_=0;
    }else{
        pid.SetMode(AUTOMATIC);
    }
    
}

void Capstan::init() {
    select_channel();
    // ams5600.setStartPosition();
    pid.SetOutputLimits(-255, 255);
    pid.SetSampleTime(1);
    input_ = calc_angle();
    setpoint_ = input_;//makes the robot stay where it is at startup.
    pid.SetMode(AUTOMATIC);
    // Serial.print("Capstan ");
    // Serial.print(enc_+1);
    // Serial.println(" initialized...");
}

// gets last known relative angle
// does not recalculate current relative angle
double Capstan::get_angle() {
    return (revolutions * 360) + current_angle;
}

// updates setpoint in terms of relative capstan angle
void Capstan::set_angle(double angle) {
    setpoint_ = angle;
}

// gets tendon length using last know relative angle
// does not recalculate current relative angle
double Capstan::get_length() {
    return ((revolutions * 360) + current_angle) * (circumference_ / 360);
}

// updates setpoint in terms of relative tendon length
void Capstan::set_length(double length) {
    setpoint_ = (length * 360) / circumference_;
}

// computes pid control output and updates motor driver
void Capstan::update() {
    input_ = calc_angle();
    pid.Compute();
    if (output_ > 0)
        digitalWrite(dir_, LOW);
    else
        digitalWrite(dir_, HIGH);
    analogWrite(pwm_, abs(output_));//should be an int instead of double but compiler is probably truncating
}

// gets angle from encoder
// converts absolute angle to relative angle
double Capstan::calc_angle()
{
    // get current value (0-4096) from encoder and convert to angle (0-360)
    select_channel();
    current_angle = ams5600.getScaledAngle() * (double) 360/4096;
    // clamp angle to between 0 and 360
    if (current_angle < 0)
        current_angle = 0;
    if (current_angle > 360)
        current_angle = 360;
    // check if angle has crossed zero and adjust revolution count
    if (current_angle < 90 && previous_angle > 270)
        revolutions++;
    if (current_angle > 270 && previous_angle < 90)
        revolutions--;
    previous_angle = current_angle;
    // return angle relative to zero
    return (revolutions * 360) + current_angle;
}

void Capstan::select_channel() 
{
    // write mux channel address to bus
    // Serial.println(" Checking wire comms..");
    // Serial.println(mux_);
    Wire.beginTransmission(mux_);
    // Serial.println("wire begin successful!");
    Wire.write(1 << enc_);
    // Serial.println("wire write successful!");
    // Wire.endTransmission();
    // Serial.println("wire end successful!");
    if (Wire.endTransmission()!=0){
        Serial.println("Hardware error!");
    }
    // Serial.print(enc_+ 1) ;
    // Serial.println(" selected!");
}
