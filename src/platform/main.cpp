//  _____  _       _    __                     
// |  __ \| |     | |  / _|                    
// | |__) | | __ _| |_| |_ ___  _ __ _ __ ___  
// |  ___/| |/ _` | __|  _/ _ \| '__| '_ ` _ \ 
// | |    | | (_| | |_| || (_) | |  | | | | | |
// |_|    |_|\__,_|\__|_| \___/|_|  |_| |_| |_|

// ===== ANTENNA - RESPONDER MODE =====
// Waits for motor command, responds with IMU data

#include <Wire.h>
#include "ldc1101.h"


// ===== CONFIGURATION =====
#define BAUD_RATE 460800

// Packet markers
#define PKT_START 0xAA
#define PKT_END 0x55

// Pins
#define UART_TX_PIN 43
#define UART_RX_PIN 44
#define I2C_SDA 5
#define I2C_SCL 6


// Motor pins (DRV8835 IN/IN mode)
#define AIN1 1
#define AIN2 2
#define BIN1 3
#define BIN2 4
#define LED_BUILTIN 21



// ===== MOTOR STATE =====
int16_t currentLeftPWM = 0;
int16_t currentRightPWM = 0;

// ===== IMU STATE =====
uint8_t magCounter = 0;
const uint8_t MAG_DIVIDER = 1;  // At 100Hz, read mag every cycle (100Hz)
const int16_t MAG_SENTINEL = 0x7FFF;

// ===== PACKET BUFFERS =====
uint8_t rxBuffer[6];   // [START][4 data][END]
uint8_t txBuffer[22];  // [START][30 data][END]



// ===== SETUP =====
void setup() {
  // Initialize serial first for debugging
  Serial1.begin(BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  
  // Setup motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // PWM setup
  analogWriteFrequency(1000);
  analogWriteResolution(8);
  
  // Start with motors stopped
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);
  currentLeftPWM = 0;
  currentRightPWM = 0;
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  ldc1101_init();

  ldc1101_configure(
    11.8e-6,
    220e-12,
    30
  );
}

// ===== MOTOR FUNCTIONS =====
void setLeftMotor(int16_t pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(AIN1, pwm);
    analogWrite(AIN2, 0);
  } else if (pwm < 0) {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, -pwm);
  } else {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 0);
  }
}

void setRightMotor(int16_t pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(BIN1, pwm);
    analogWrite(BIN2, 0);
  } else if (pwm < 0) {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, -pwm);
  } else {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 0);
  }
}

void setMotors(int16_t left, int16_t right) {
  setLeftMotor(left);
  setRightMotor(right);
  currentLeftPWM = left;
  currentRightPWM = right;
}

void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  currentLeftPWM = 0;
  currentRightPWM = 0;
}

// ===== READ MOTOR COMMAND =====
bool readMotorCommand() {

  int i = 1;

  while (i == 1){
    if (Serial1.read() != PKT_START) {
        i = 1;  // Not a valid start
      }
    else{
      i = 0;
    }
  }
  
  
  // Read the rest of the command
  if (Serial1.available() >= 5) {  // 4 data + 1 end
    uint8_t data[4];
    Serial1.readBytes(data, 4);
    uint8_t endMarker = Serial1.read();
    
    if (endMarker == PKT_END) {
      // Valid command received
      int16_t left = (data[0] << 8) | data[1];
      int16_t right = (data[2] << 8) | data[3];
      setMotors(left, right);
      return true;
    }
  }
  
  return false;
}

// ===== READ LDC AND SEND RESPONSE =====
void sendLDCResponse() {
  txBuffer[0] = PKT_START;

  uint64_t timestamp = micros();
  memcpy(txBuffer + 1, &timestamp, 8);

  int16_t motors[2] = {currentLeftPWM, currentRightPWM};
  memcpy(txBuffer + 9, motors, 4);

 ldc1101_measurement_t ldc = ldc1101_read(220e-12);

  // LDC data (8 bytes)
  float rp = ldc.Rp_ohms;
  float l = ldc.L_uH;

  memcpy(txBuffer + 13, &rp, 4);
  memcpy(txBuffer + 17, &l,  4);
  txBuffer[21] = PKT_END;
  Serial1.write(txBuffer, 22);
}

// ===== MAIN LOOP =====
void loop() {
  // 1. Wait for and process motor command
  readMotorCommand();
  
  // 2. Read IMU and send response
  sendLDCResponse();
  
  // Small delay to prevent CPU spinning
  delayMicroseconds(10);
}
