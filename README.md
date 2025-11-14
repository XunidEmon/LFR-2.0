Line Follower Robot (LFR)

🛠️ Components List:
Mini Mega 2560 Pro
N20 Motor with Encoder (2 ta)
L298N Motor Driver
OLED Display (0.96")
LiPo Battery (3S 1200mAh)
Motor Wheel (2 ta)
Ball Caster
Motor Mounting Bracket
Push Button (3 ta)
IR Sensor (8 ta)
PCB Board

🔌 Wiring Connection:

Motor Driver:
ain1 = 7
ain2 = 6  
bin1 = 9
bin2 = 10
pwma = 5
pwmb = 4

Button:
rbtn_l = 3 (Stop)
rbtn_h = 2 (Start) 
lbtn = 11 (Calibrate)

IR Sensor:
A0, A2, A4, A6, A8, A10, A12, A14

🚀 How to Run:
Code Upload by Arduino IDE
Sensor Calibrate : Left white button press.
Start : Left red high button press.
Stop koro: Right button press.

⚙️ PID Settings:
Kp = 42
Kd = 5.0
Ki = 1.0
Base Speed = 120

🔧 Library use:
1.#include <SPI.h>
2.#include <Wire.h>
3.#include <Adafruit_GFX.h>
4.#include <Adafruit_SH110X.h>

##Code
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

//#define i2c_Address 0x78   //initialize with the I2C addr 0x3C Typically eBay OLED's
#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
//#define i2c_Address 0x3d //initialize with the I2C addr 0x3D Typically Adafruit OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

int ain1 = 7;
int ain2 = 6;
int bin1 = 9;
int bin2 = 10;
int pwma = 5;
int pwmb = 4;
int stby = 8;
int rbtn_l = 3;
int rbtn_h = 2;
int lbtn = 11;   

int minValue[] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
int maxValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
//int midValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
int midValue[] = {350, 350, 350, 350, 350, 350, 350, 350};  //for testing
int sensorValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
int rawValue[] = {0, 0, 0, 0, 0, 0, 0, 0};

int analogPin[] = {A0, A2, A4, A6, A8, A10, A12, A14};
//int analogPin[] = {A14, A12, A10, A8, A6, A4, A2, A0};
int pos_error = 0;

float Kp = 40;  //// Adjust this value based on experimentation
float Kd = 5.0;  //// Adjust this value based on experimentation
float Ki = 2.0; //// Adjust this value based on experimentation

int left_motor_speed = 0;
int right_motor_speed = 0;
int base_speed = 180;

int run_flag = 0;
int calib_flag = 0;
volatile byte stop_flag = 0;

void run_bot(void);
void stop_bot(void);
void calculate_motor_speed(int line_position);
void drive_motors(int right_speed, int left_speed);
void calibrate_bot(void);
void display_graph(void);
void read_sensor(void);
void button_stop_pressed(void);


//this function read the sensor value
void read_sensor(void){
  rawValue[7] = analogRead(A0);
  rawValue[6] = analogRead(A2);
  rawValue[5] = analogRead(A4);
  rawValue[4] = analogRead(A6);
  rawValue[3] = analogRead(A8);
  rawValue[2] = analogRead(A10);
  rawValue[1] = analogRead(A12);
  rawValue[0] = analogRead(A14);
}

void display_graph(void){
  for(int i=0, j=0; i<8; i++, j+=17){
    int sensor_value[8];
    sensor_value[i] = map(rawValue[i], 0, 1023, 0, 64);
    //display.drawLine(0, 64, 0, adc_value, SH110X_WHITE); display.drawLine(0, adc_value, 0, 0, SH110X_BLACK);
    display.drawLine(j+0, 64, j+0, sensor_value[i], SH110X_WHITE); display.drawLine(j+0, sensor_value[i], j+0, 0, SH110X_BLACK);
    display.drawLine(j+1, 64, j+1, sensor_value[i], SH110X_WHITE); display.drawLine(j+1, sensor_value[i], j+1, 0, SH110X_BLACK);
    display.drawLine(j+2, 64, j+2, sensor_value[i], SH110X_WHITE); display.drawLine(j+2, sensor_value[i], j+2, 0, SH110X_BLACK);
  }
  
  display.display();
}

//calibrate the sensors for 10 time ccw and 10 times cw direction
void calibrate_bot(void){
  drive_motors(-150, 150);
  for(int k=0; k<30; k++){
    for(int i=0, j=0; i<8; i++, j+=17){
      rawValue[i] = analogRead(analogPin[i]);
      int adc_value = map(rawValue[i], 0, 1023, 0, 64);
      display.drawLine(j+0, 64, j+0, 64-adc_value, 1); display.drawLine(j+0, 64-adc_value, j+0, 0, 0);
      display.drawLine(j+1, 64, j+1, 64-adc_value, 1); display.drawLine(j+1, 64-adc_value, j+1, 0, 0);
      display.drawLine(j+2, 64, j+2, 64-adc_value, 1); display.drawLine(j+2, 64-adc_value, j+2, 0, 0);
      display.display();
      if(rawValue[i] < minValue[i]) minValue[i] = rawValue[i];
      if(rawValue[i] > maxValue[i]) maxValue[i] = rawValue[i];
    }
    delay(100);
  }
  drive_motors(150, -150);
  for(int k=0; k<30; k++){
    for(int i=0, j=0; i<8; i++, j+=17){
      rawValue[i] = analogRead(analogPin[i]);
      int adc_value = map(rawValue[i], 0, 1023, 0, 64);
      display.drawLine(j+0, 64, j+0, 64-adc_value, 1); display.drawLine(j+0, 64-adc_value, j+0, 0, 0);
      display.drawLine(j+1, 64, j+1, 64-adc_value, 1); display.drawLine(j+1, 64-adc_value, j+1, 0, 0);
      display.drawLine(j+2, 64, j+2, 64-adc_value, 1); display.drawLine(j+2, 64-adc_value, j+2, 0, 0);
      display.display();
      if(rawValue[i] < minValue[i]) minValue[i] = rawValue[i];
      if(rawValue[i] > maxValue[i]) maxValue[i] = rawValue[i];
    }
    delay(100);
  }
  for(int i=0; i<8; i++){
    midValue[i] = (minValue[i] + maxValue[i]) / 2;
  }
  analogWrite(pwma, 0); //stop both motors
  analogWrite(pwmb, 0); //stop both motors
  calib_flag = 0;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 10);
  display.println("Calibration Done");
  display.display();
}

void test_sensor(void){
  
    for(int i=0, j=0; i<8; i++, j+=17){
      rawValue[i] = analogRead(analogPin[i]);
      int adc_value = map(rawValue[i], 0, 1023, 0, 64);
      display.drawLine(j+0, 64, j+0, 64-adc_value, 1); display.drawLine(j+0, 64-adc_value, j+0, 0, 0);
      display.drawLine(j+1, 64, j+1, 64-adc_value, 1); display.drawLine(j+1, 64-adc_value, j+1, 0, 0);
      display.drawLine(j+2, 64, j+2, 64-adc_value, 1); display.drawLine(j+2, 64-adc_value, j+2, 0, 0);
      display.display();
      if(rawValue[i] < minValue[i]) minValue[i] = rawValue[i];
      if(rawValue[i] > maxValue[i]) maxValue[i] = rawValue[i];
    }
    
}

void drive_motors(int right_speed, int left_speed){
  int magnitude_right_speed = abs(right_speed);
  int magnitude_left_speed = abs(left_speed);
  
  digitalWrite(stby, HIGH);
  //controlling right motor
  analogWrite(pwma, magnitude_right_speed);
  if (right_speed>=0){
     digitalWrite(ain1, HIGH);
     digitalWrite(ain2, LOW);
  }
  else{
     digitalWrite(ain1, LOW);
     digitalWrite(ain2, HIGH);
  }
  
  //controlling left motor
  analogWrite(pwmb, magnitude_left_speed);
  if (left_speed>=0){
     digitalWrite(bin1, HIGH);
     digitalWrite(bin2, LOW);
  }    
  else{
     digitalWrite(bin1, LOW);
     digitalWrite(bin2, HIGH);
  }
}

void button_stop_pressed(void){
  stop_flag = 1;
}

void stop_bot(void){
  //controlling right motor
  analogWrite(pwma, 0);
  digitalWrite(ain1, 0);
  digitalWrite(ain2, 0);
  analogWrite(pwmb, 0);
  digitalWrite(bin1, 0);
  digitalWrite(bin2, 0);
 }

void calculate_motor_speed(int line_position){
  //calculate speed here
  int previous_error = 0, PID_value, P = 0, I = 0, D = 0;
  int error = line_position;
  P = error;
  I = I + error;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_error = error;

  left_motor_speed = base_speed + PID_value;
  right_motor_speed = base_speed - PID_value;

  if(left_motor_speed > 255) left_motor_speed = 255;
  if(right_motor_speed > 255) right_motor_speed = 255;
  if(left_motor_speed < -255) left_motor_speed = -255;
  if(right_motor_speed < -255) right_motor_speed = -255;

}

void run_bot(void){
  byte line_position = 0;
  while(1){
    for(int i=0; i<8; i++){
      rawValue[i] = analogRead(analogPin[i]);
      if(rawValue[i] > midValue[i])
        line_position = (line_position << 1) | 1;
      else
        line_position = line_position << 1;
    }
    if(line_position == 0b10000000) pos_error = -7;
    else if(line_position == 0b11000000) pos_error = -6;
    else if(line_position == 0b11100000) pos_error = -5;
    else if(line_position == 0b01100000) pos_error = -4;
    else if(line_position == 0b01110000) pos_error = -3;
    else if(line_position == 0b00110000) pos_error = -2;
    else if(line_position == 0b00111000) pos_error = -1;
    else if(line_position == 0b00011000) pos_error = 0;
    else if(line_position == 0b00011100) pos_error = 1;
    else if(line_position == 0b00001100) pos_error = 2;
    else if(line_position == 0b00001110) pos_error = 3;
    else if(line_position == 0b00000110) pos_error = 4;
    else if(line_position == 0b00000111) pos_error = 5;
    else if(line_position == 0b00000011) pos_error = 6;
    else if(line_position == 0b00000001) pos_error = 7;

    //if all sensors read black
    else if(line_position == 0b11111111){
      
      drive_motors(100, 130);
      delay(160);

      for(int i=0; i<8; i++){
      rawValue[i] = analogRead(analogPin[i]);
      if(rawValue[i] > midValue[i])
        line_position = (line_position << 1) | 1;
      else
        line_position = line_position << 1;
      }
      if(line_position == 0b11111111) {
        stop_bot(); 
        delay(1000);
        stop_flag = 1;
        break;
      }
      
    }

    //if all sensors read black
    else if(line_position == 0b00000000){
       drive_motors(150, -150); 
    } 
    calculate_motor_speed(pos_error);
    drive_motors(right_motor_speed, left_motor_speed);
  }  
  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(32, OUTPUT);
  digitalWrite(32, HIGH);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  pinMode(pwma, OUTPUT);
  pinMode(pwmb, OUTPUT);
  pinMode(stby, OUTPUT);
  pinMode(rbtn_l, INPUT_PULLUP);
  pinMode(rbtn_h, INPUT_PULLUP);
  pinMode(lbtn, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), button_stop_pressed, LOW);

  delay(250); // wait for the OLED to power up
  display.begin(i2c_Address, true); // Address 0x3C default
 //display.setContrast (0); // dim display
 
  display.display();
  delay(2000);

  // Clear the buffer.
  display.clearDisplay();

  drive_motors(100, 130);
  delay(200);
  drive_motors(0, 0);

}

void loop() {

  
  // Calibration button (left button)
  if (!digitalRead(lbtn)) {
    calib_flag = 1;
    run_flag = 0;
    stop_flag = 0;
    delay(160);  // debounce delay
  }

  // Start button (right button HIGH)
  if (!digitalRead(rbtn_h)) {
    calib_flag = 0;
    run_flag = 1;
    stop_flag = 0;
    delay(160);  // debounce delay
  }

  // STOP button (handled via interrupt)
  if (stop_flag == 1) {
    stop_bot();
    run_flag = 0;
    stop_flag = 0;
    calib_flag = 0;
  }

  // Run robot
  else if (run_flag == 1) {
    run_bot();  // এই ফাংশনের ভেতরে while(1) এর ভিতরে stop_flag চেক থাকতে হবে
  }

  // Calibrate robot
  else if (calib_flag == 1) {
    calibrate_bot();  // শেষ হলে calib_flag = 0 করে দিবে
  }
  
  
  //test_sensor();
}
