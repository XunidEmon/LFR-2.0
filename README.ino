# Line Follower Robot (LFR)

An Arduino-based line following robot with 8 IR sensors, PID control, OLED display, and button controls.

## 🎯 Features

- **8 IR Sensors** for precise line detection
- **PID Control** for smooth following
- **OLED Display** (0.96") for sensor visualization
- **3 Button Control** - Start, Stop, Calibrate
- **N20 Motors** with Encoders
- **L298N Motor Driver**

## 🛠️ Components Required

| Component | Quantity |
|-----------|----------|
| Mini Mega 2560 Pro | 1 |
| N20 Motor with Encoder | 2 |
| L298N Motor Driver | 1 |
| OLED Display (0.96") | 1 |
| LiPo Battery (3S 1200mAh) | 1 |
| Motor Wheel | 2 |
| Ball Caster | 1 |
| Motor Mounting Bracket | 2 |
| Push Button | 3 |
| IR Sensor | 8 |
| PCB Board | 1 |

## 🔌 Wiring Connections

### Motor Driver (L298N)

| L298N Pin | Arduino Pin |
|-----------|-------------|
| ain1 | 7 |
| ain2 | 6 |
| bin1 | 9 |
| bin2 | 10 |
| pwma | 5 |
| pwmb | 4 |

### Push Buttons

| Button | Arduino Pin | Function |
|--------|-------------|----------|
| lbtn | 11 | Calibrate (Left button) |
| rbtn_h | 2 | Start (Right button HIGH) |
| rbtn_l | 3 | Stop (Right button LOW) |

### IR Sensors

| Sensor | Arduino Pin |
|--------|-------------|
| IR1 | A0 |
| IR2 | A2 |
| IR3 | A4 |
| IR4 | A6 |
| IR5 | A8 |
| IR6 | A10 |
| IR7 | A12 |
| IR8 | A14 |

## 🚀 How to Use

### 1. Upload Code
- Open Arduino IDE
- Copy the code to your sketch
- Select board: Arduino Mega 2560
- Upload

### 2. Calibrate
- Press **Left White Button** (Pin 11)
- Robot will rotate to calibrate sensors
- OLED shows "Calibration Done"

### 3. Start
- Press **Right Button HIGH** (Pin 2)
- Robot starts following the line

### 4. Stop
- Press **Right Button** (Pin 3)
- Robot stops immediately

## ⚙️ PID Settings

| Parameter | Value |
|-----------|-------|
| Kp | 42 |
| Ki | 1.0 |
| Kd | 5.0 |
| Base Speed | 120 |

## 📚 Required Libraries

Install these from Arduino Library Manager:

```cpp
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// OLED Configuration
#define i2c_Address 0x3c
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Motor Driver Pins
int ain1 = 7;
int ain2 = 6;
int bin1 = 9;
int bin2 = 10;
int pwma = 5;
int pwmb = 4;
int stby = 8;

// Button Pins
int rbtn_l = 3;  // Stop
int rbtn_h = 2;  // Start
int lbtn = 11;   // Calibrate

// Sensor Arrays
int minValue[] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
int maxValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
int midValue[] = {350, 350, 350, 350, 350, 350, 350, 350};
int rawValue[] = {0, 0, 0, 0, 0, 0, 0, 0};

// Sensor Pins
int analogPin[] = {A0, A2, A4, A6, A8, A10, A12, A14};

// PID Variables
int pos_error = 0;
float Kp = 40;
float Kd = 5.0;
float Ki = 2.0;

// Motor Speeds
int left_motor_speed = 0;
int right_motor_speed = 0;
int base_speed = 180;

// Control Flags
int run_flag = 0;
int calib_flag = 0;
volatile byte stop_flag = 0;

// Function to read all sensors
void read_sensor(void) {
  rawValue[7] = analogRead(A0);
  rawValue[6] = analogRead(A2);
  rawValue[5] = analogRead(A4);
  rawValue[4] = analogRead(A6);
  rawValue[3] = analogRead(A8);
  rawValue[2] = analogRead(A10);
  rawValue[1] = analogRead(A12);
  rawValue[0] = analogRead(A14);
}

// Display sensor graph on OLED
void display_graph(void) {
  for(int i=0, j=0; i<8; i++, j+=17) {
    int sensor_value[8];
    sensor_value[i] = map(rawValue[i], 0, 1023, 0, 64);
    
    display.drawLine(j+0, 64, j+0, sensor_value[i], SH110X_WHITE);
    display.drawLine(j+0, sensor_value[i], j+0, 0, SH110X_BLACK);
    display.drawLine(j+1, 64, j+1, sensor_value[i], SH110X_WHITE);
    display.drawLine(j+1, sensor_value[i], j+1, 0, SH110X_BLACK);
    display.drawLine(j+2, 64, j+2, sensor_value[i], SH110X_WHITE);
    display.drawLine(j+2, sensor_value[i], j+2, 0, SH110X_BLACK);
  }
  display.display();
}

// Calibrate sensors
void calibrate_bot(void) {
  // Rotate clockwise
  drive_motors(-150, 150);
  for(int k=0; k<30; k++) {
    for(int i=0, j=0; i<8; i++, j+=17) {
      rawValue[i] = analogRead(analogPin[i]);
      int adc_value = map(rawValue[i], 0, 1023, 0, 64);
      
      display.drawLine(j+0, 64, j+0, 64-adc_value, 1);
      display.drawLine(j+0, 64-adc_value, j+0, 0, 0);
      display.drawLine(j+1, 64, j+1, 64-adc_value, 1);
      display.drawLine(j+1, 64-adc_value, j+1, 0, 0);
      display.drawLine(j+2, 64, j+2, 64-adc_value, 1);
      display.drawLine(j+2, 64-adc_value, j+2, 0, 0);
      display.display();
      
      if(rawValue[i] < minValue[i]) minValue[i] = rawValue[i];
      if(rawValue[i] > maxValue[i]) maxValue[i] = rawValue[i];
    }
    delay(100);
  }
  
  // Rotate counter-clockwise
  drive_motors(150, -150);
  for(int k=0; k<30; k++) {
    for(int i=0, j=0; i<8; i++, j+=17) {
      rawValue[i] = analogRead(analogPin[i]);
      int adc_value = map(rawValue[i], 0, 1023, 0, 64);
      
      display.drawLine(j+0, 64, j+0, 64-adc_value, 1);
      display.drawLine(j+0, 64-adc_value, j+0, 0, 0);
      display.drawLine(j+1, 64, j+1, 64-adc_value, 1);
      display.drawLine(j+1, 64-adc_value, j+1, 0, 0);
      display.drawLine(j+2, 64, j+2, 64-adc_value, 1);
      display.drawLine(j+2, 64-adc_value, j+2, 0, 0);
      display.display();
      
      if(rawValue[i] < minValue[i]) minValue[i] = rawValue[i];
      if(rawValue[i] > maxValue[i]) maxValue[i] = rawValue[i];
    }
    delay(100);
  }
  
  // Calculate mid values
  for(int i=0; i<8; i++) {
    midValue[i] = (minValue[i] + maxValue[i]) / 2;
  }
  
  // Stop motors
  analogWrite(pwma, 0);
  analogWrite(pwmb, 0);
  calib_flag = 0;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 10);
  display.println("Calibration Done");
  display.display();
}

// Drive motors
void drive_motors(int right_speed, int left_speed) {
  int magnitude_right_speed = abs(right_speed);
  int magnitude_left_speed = abs(left_speed);
  
  digitalWrite(stby, HIGH);
  
  // Right motor
  analogWrite(pwma, magnitude_right_speed);
  if (right_speed >= 0) {
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
  } else {
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
  }
  
  // Left motor
  analogWrite(pwmb, magnitude_left_speed);
  if (left_speed >= 0) {
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);
  } else {
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, HIGH);
  }
}

// Stop button interrupt
void button_stop_pressed(void) {
  stop_flag = 1;
}

// Stop robot
void stop_bot(void) {
  analogWrite(pwma, 0);
  digitalWrite(ain1, 0);
  digitalWrite(ain2, 0);
  analogWrite(pwmb, 0);
  digitalWrite(bin1, 0);
  digitalWrite(bin2, 0);
}

// Calculate motor speeds using PID
void calculate_motor_speed(int line_position) {
  static int previous_error = 0;
  static int I = 0;
  
  int error = line_position;
  int P = error;
  I = I + error;
  int D = error - previous_error;
  
  int PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_error = error;
  
  left_motor_speed = base_speed + PID_value;
  right_motor_speed = base_speed - PID_value;
  
  // Limit speeds
  if(left_motor_speed > 255) left_motor_speed = 255;
  if(right_motor_speed > 255) right_motor_speed = 255;
  if(left_motor_speed < -255) left_motor_speed = -255;
  if(right_motor_speed < -255) right_motor_speed = -255;
}

// Main line following logic
void run_bot(void) {
  byte line_position = 0;
  
  while(1) {
    // Read sensors and create binary pattern
    for(int i=0; i<8; i++) {
      rawValue[i] = analogRead(analogPin[i]);
      if(rawValue[i] > midValue[i]) 
        line_position = (line_position << 1) | 1;
      else 
        line_position = line_position << 1;
    }
    
    // Convert binary pattern to position error
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
    
    // All sensors see line (junction/end)
    else if(line_position == 0b11111111) {
      drive_motors(100, 130);
      delay(160);
      
      for(int i=0; i<8; i++) {
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
    
    // No sensor sees line (lost line)
    else if(line_position == 0b00000000) {
      drive_motors(150, -150);  // Spin to find line
    }
    
    calculate_motor_speed(pos_error);
    drive_motors(right_motor_speed, left_motor_speed);
    
    // Check for stop
    if(stop_flag == 1) {
      stop_bot();
      break;
    }
  }
}

void setup() {
  Serial.begin(9600);
  
  // Configure pins
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
  
  // Stop button interrupt
  attachInterrupt(digitalPinToInterrupt(3), button_stop_pressed, LOW);
  
  // Initialize OLED
  delay(250);
  display.begin(i2c_Address, true);
  display.display();
  delay(2000);
  display.clearDisplay();
  
  // Initial motor test
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
    delay(160);
  }
  
  // Start button (right button HIGH)
  if (!digitalRead(rbtn_h)) {
    calib_flag = 0;
    run_flag = 1;
    stop_flag = 0;
    delay(160);
  }
  
  // STOP button (via interrupt)
  if (stop_flag == 1) {
    stop_bot();
    run_flag = 0;
    stop_flag = 0;
    calib_flag = 0;
  }
  
  // Run robot
  else if (run_flag == 1) {
    run_bot();
  }
  
  // Calibrate robot
  else if (calib_flag == 1) {
    calibrate_bot();
  }
}
