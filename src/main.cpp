#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include <WiFi.h>
#include "Sensor.h"


#define CMD_IDLE 0
#define CMD_MOVE_FORWARD 1
#define CMD_MOVE_BACKWARD 2
#define CMD_MOVE_LEFT 3
#define CMD_MOVE_RIGHT 4

// ------------ PWM -----------
constexpr unsigned int _PWM_FREQ = 1000;
constexpr unsigned int _PWM_CH = 0;
constexpr unsigned int _PWN_RES = 8;
// ------------------------------

// ------------ MOTOR PINS ------------------
constexpr unsigned int PIN_MOTOR_1_A = 14;
constexpr unsigned int PIN_MOTOR_1_B = 27;

constexpr unsigned int PIN_MOTOR_2_A = 25;
constexpr unsigned int PIN_MOTOR_2_B = 26;
// ------------------------------------------

// ---------- PWM PINS --------------
constexpr unsigned int PIN_PWM = 32;
// ----------------------------------

// ---------- SPEED SENSOR PINS ----------
constexpr unsigned int PIN_SENSOR_L = 16;
constexpr unsigned int PIN_SENSOR_R = 17;
// ---------------------------------------


// ----------- Sensor Data -----------



SpeedSensorData sensorLeft = {0, 20, PIN_SENSOR_L};
SpeedSensorData sensorRight = {0, 20, PIN_SENSOR_R};
// ----------------------------------


// ------------ Time ---------------

unsigned long long prevTime = 0;

// ---------------------------------


// ---------- Move Data ------------
bool isMoving = false;
unsigned char currentCommand = CMD_IDLE;
unsigned char currentSpeed = 256 * 0.40;

// ---------------------------------


const char* SSID = "ALMA";
const char* PASSWORD = "(EVIL)(ALMA)(1322)";

WiFiServer server(1322);
LiquidCrystal_I2C lcd(0x27, 16, 2);


void initMotor() {

  pinMode(PIN_MOTOR_1_A, OUTPUT);
  pinMode(PIN_MOTOR_1_B, OUTPUT);
  pinMode(PIN_MOTOR_2_A, OUTPUT);
  pinMode(PIN_MOTOR_2_B, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  

  digitalWrite(PIN_MOTOR_1_A, LOW);
  digitalWrite(PIN_MOTOR_2_A, LOW);
  digitalWrite(PIN_MOTOR_1_B, LOW);
  digitalWrite(PIN_MOTOR_2_B, LOW);

  ledcSetup(_PWM_CH, _PWM_FREQ, _PWN_RES);
  ledcAttachPin(PIN_PWM, _PWM_CH);
  ledcWrite(_PWM_CH, currentSpeed);
}


void setup() {
  Serial.begin(115200);
  sensorBegin(sensorLeft.pin, sensorRight.pin);
  initMotor();
  lcd.init();
  lcd.backlight();
  

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Connecting");

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
  }

  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print(WiFi.localIP());

  server.begin();
}



void updateLCD() {
  lcd.clear();
  lcd.print("L:"); lcd.print(sensorLeft.ticks);
  lcd.print(" R:"); lcd.print(sensorRight.ticks);
  lcd.print(currentSpeed);
  lcd.setCursor(0,1);
  lcd.print("State:");

   switch(currentCommand){

    case CMD_MOVE_FORWARD:
      lcd.print("FWRD");
    break;
    case CMD_MOVE_BACKWARD:
      lcd.print("BCKWRD");
    break;
    case CMD_MOVE_LEFT:
      lcd.print("LEFT");
    break;
    case CMD_MOVE_RIGHT:
      lcd.print("RIGHT");
    break;
    case CMD_IDLE:
      lcd.print("IDLE");
    break;
  }




}





void move(){
  

  switch(currentCommand){

    case CMD_MOVE_FORWARD:
      
      digitalWrite(PIN_MOTOR_1_A, HIGH);
      digitalWrite(PIN_MOTOR_1_B, LOW);
      digitalWrite(PIN_MOTOR_2_A, LOW);
      digitalWrite(PIN_MOTOR_2_B, HIGH);
     
    break;
    case CMD_MOVE_BACKWARD:
     
      digitalWrite(PIN_MOTOR_1_A, LOW);
      digitalWrite(PIN_MOTOR_1_B, HIGH); // pin 33/25
      digitalWrite(PIN_MOTOR_2_A, HIGH);
      digitalWrite(PIN_MOTOR_2_B, LOW);
    break;
    case CMD_MOVE_LEFT:
      
      digitalWrite(PIN_MOTOR_1_A, HIGH);
      digitalWrite(PIN_MOTOR_1_B, LOW);
      digitalWrite(PIN_MOTOR_2_A, HIGH);
      digitalWrite(PIN_MOTOR_2_B, LOW);
    break;
    case CMD_MOVE_RIGHT:
      digitalWrite(PIN_MOTOR_1_A, LOW);
      digitalWrite(PIN_MOTOR_1_B, HIGH);
      digitalWrite(PIN_MOTOR_2_A, LOW);
      digitalWrite(PIN_MOTOR_2_B, HIGH);
    break;
    case CMD_IDLE:
      digitalWrite(PIN_MOTOR_1_A, LOW);
      digitalWrite(PIN_MOTOR_1_B, LOW);
      digitalWrite(PIN_MOTOR_2_A, LOW);
      digitalWrite(PIN_MOTOR_2_B, LOW);
    break;
  }
}



void loop() {
  WiFiClient client = server.available();
  unsigned long long currentTime = millis();
  unsigned long long elapseTime = currentTime - prevTime;
  prevTime = currentTime;

  

  if(client){
    while (client.connected()) {
      currentTime = millis();
      elapseTime = currentTime - prevTime;
      prevTime = currentTime;


      if(client.available()) {
        
        unsigned int bytePacket = client.read();
        
        if(bytePacket >= 0x00000100 && bytePacket <= 0x0000FF00) {
          unsigned int speed = bytePacket >> 8;
          ledcWrite(_PWM_CH, speed);
          currentSpeed = speed;
          updateLCD();
          
        }
        
        if(currentCommand != CMD_IDLE) {
          updateLCD();
          
        }
        if(bytePacket >= CMD_MOVE_FORWARD && bytePacket <= CMD_MOVE_RIGHT){
          currentCommand = bytePacket;
          updateLCD();
        }
        


      }

      
      
      
    }
    
  }



  if(sensorLeft.ticks >= 20 || sensorRight.ticks >= 20){
        currentCommand = CMD_IDLE;
        sensorLeft.ticks = 0;
        sensorRight.ticks = 0;
        digitalWrite(PIN_MOTOR_1_A, LOW);
        digitalWrite(PIN_MOTOR_1_B, LOW);
        digitalWrite(PIN_MOTOR_2_A, LOW);
        digitalWrite(PIN_MOTOR_2_B, LOW);
        updateLCD();
  }

  move();

  if(currentTime % 1500 == 0){ // every 1.5 sec update lcd
    updateLCD();
  }


}