// capstan position control gains
#define KP 40
//40 is default signed (03/19/23)
#define KI 0
#define KD 0
//40,2,1 or 20,0,0

// capstan parameters
#define CIRCUMFERENCE 314

// motor driver pins
#define M1_DIR 10
#define M1_PWM 9
#define M1_FLT 8
#define M1_CS  A0

#define M2_DIR 13
#define M2_PWM 12
#define M2_FLT 11
#define M2_CS  A1

#define M3_DIR 4
#define M3_PWM 3
#define M3_FLT 2
#define M3_CS  A2

#define M4_DIR 7
#define M4_PWM 6
#define M4_FLT 5
#define M4_CS  A3

#define M5_DIR 43
#define M5_PWM 45
#define M5_FLT 47
#define M5_CS  A4

#define M6_DIR 46
#define M6_PWM 44
#define M6_FLT 42
#define M6_CS  A5

// i2c mux channels
#define M1_ENC 0
#define M2_ENC 1
#define M3_ENC 2
#define M4_ENC 3
#define M5_ENC 4
#define M6_ENC 5

// i2c mux address
#define MUX_ADDR 0x70