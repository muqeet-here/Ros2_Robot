#include <Arduino.h>

// Motor driver pins (L298N)
#define MOTOR_ENA 33
#define MOTOR_IN1 21
#define MOTOR_IN2 22
#define MOTOR_IN3 32
#define MOTOR_IN4 25
#define MOTOR_ENB 19

// Encoder pins
#define ENCODER_A_RIGHT 18
#define ENCODER_B_RIGHT 26
#define ENCODER_A_LEFT 16
#define ENCODER_B_LEFT 17

// Encoder counts
volatile long encoder_count_right = 0;
volatile long encoder_count_left = 0;

// Debounce timing for encoders (microseconds)
volatile unsigned long last_right_a_time = 0;
volatile unsigned long last_right_b_time = 0;
volatile unsigned long last_left_a_time = 0;
volatile unsigned long last_left_b_time = 0;
const unsigned long DEBOUNCE_TIME = 1000; // 1ms debounce

// PWM settings
const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;
const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;

// Current motor speed (0-255)
int current_speed = 128;

// PID Control variables for each motor
struct PIDController {
  float kp = 0.8;     // Proportional gain (reduced)
  float ki = 0.1;     // Integral gain (reduced)
  float kd = 0.05;    // Derivative gain (reduced)
  
  float target_speed = 0.0;
  float last_error = 0.0;
  float integral = 0.0;
  float output = 0.0;
  float last_output = 0.0;  // For smoothing
  
  long last_encoder = 0;
  unsigned long last_time = 0;
  float current_speed = 0.0;
};

PIDController pid_left;
PIDController pid_right;

const float ENCODER_TICKS_PER_REV = 20.0;  // Adjust based on your encoder
const float WHEEL_DIAMETER = 0.06858;  // meters (2.7 inches)
const float WHEEL_CIRCUMFERENCE = 3.14159 * WHEEL_DIAMETER;

bool use_pid = false;  // PID control enable flag

// Interrupt service routines for encoders
void IRAM_ATTR encoderISR_Right_A() {
  unsigned long current_time = micros();
  if (current_time - last_right_a_time < DEBOUNCE_TIME) return;
  last_right_a_time = current_time;
  
  if (digitalRead(ENCODER_A_RIGHT) == digitalRead(ENCODER_B_RIGHT)) {
    encoder_count_right++;
  } else {
    encoder_count_right--;
  }
}

void IRAM_ATTR encoderISR_Right_B() {
  unsigned long current_time = micros();
  if (current_time - last_right_b_time < DEBOUNCE_TIME) return;
  last_right_b_time = current_time;
  
  if (digitalRead(ENCODER_A_RIGHT) != digitalRead(ENCODER_B_RIGHT)) {
    encoder_count_right++;
  } else {
    encoder_count_right--;
  }
}

void IRAM_ATTR encoderISR_Left_A() {
  unsigned long current_time = micros();
  if (current_time - last_left_a_time < DEBOUNCE_TIME) return;
  last_left_a_time = current_time;
  
  if (digitalRead(ENCODER_A_LEFT) == digitalRead(ENCODER_B_LEFT)) {
    encoder_count_left++;
  } else {
    encoder_count_left--;
  }
}

void IRAM_ATTR encoderISR_Left_B() {
  unsigned long current_time = micros();
  if (current_time - last_left_b_time < DEBOUNCE_TIME) return;
  last_left_b_time = current_time;
  
  if (digitalRead(ENCODER_A_LEFT) != digitalRead(ENCODER_B_LEFT)) {
    encoder_count_left++;
  } else {
    encoder_count_left--;
  }
}

float updatePID(PIDController &pid, long current_encoder) {
  unsigned long current_time = millis();
  float dt = (current_time - pid.last_time) / 1000.0;  // seconds
  
  if (dt < 0.01) return pid.output;  // Avoid too frequent updates
  
  if (pid.target_speed == 0) {
    pid.integral = 0;
    pid.last_error = 0;
    pid.output = 0;
    pid.last_output = 0;
    return 0;
  }
  
  // Calculate current speed (ticks per second)
  long delta_ticks = current_encoder - pid.last_encoder;
  float ticks_per_sec = abs(delta_ticks) / dt;
  
  // Convert to m/s (optional, but helps with tuning)
  pid.current_speed = (ticks_per_sec / ENCODER_TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
  
  // Calculate error
  float error = pid.target_speed - ticks_per_sec;
  
  // Deadband to reduce noise
  if (abs(error) < 5.0) {
    error = 0;
  }
  
  // PID calculations
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -50, 50);  // Anti-windup
  
  float derivative = (error - pid.last_error) / dt;
  
  float new_output = (pid.kp * error) + (pid.ki * pid.integral) + (pid.kd * derivative);
  
  // Smooth output changes (low-pass filter)
  pid.output = 0.7 * pid.last_output + 0.3 * new_output;
  pid.output = constrain(pid.output, 0, 255);
  
  // Update state
  pid.last_error = error;
  pid.last_output = pid.output;
  pid.last_encoder = current_encoder;
  pid.last_time = current_time;
  
  return pid.output;
}

void stopMotors() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
  
  // Reset PID controllers
  pid_left.target_speed = 0;
  pid_right.target_speed = 0;
  pid_left.integral = 0;
  pid_right.integral = 0;
}

void setMotorDirection(bool forward_left, bool forward_right) {
  digitalWrite(MOTOR_IN1, forward_left ? HIGH : LOW);
  digitalWrite(MOTOR_IN2, forward_left ? LOW : HIGH);
  digitalWrite(MOTOR_IN3, forward_right ? HIGH : LOW);
  digitalWrite(MOTOR_IN4, forward_right ? LOW : HIGH);
}

void moveForward(int speed) {
  setMotorDirection(true, true);
  if (use_pid) {
    pid_left.target_speed = speed * 1.5;   // Reduced scaling
    pid_right.target_speed = speed * 1.5;
  } else {
    ledcWrite(PWM_CHANNEL_A, speed);
    ledcWrite(PWM_CHANNEL_B, speed);
  }
}

void moveBackward(int speed) {
  setMotorDirection(false, false);
  if (use_pid) {
    pid_left.target_speed = speed * 1.5;
    pid_right.target_speed = speed * 1.5;
  } else {
    ledcWrite(PWM_CHANNEL_A, speed);
    ledcWrite(PWM_CHANNEL_B, speed);
  }
}

void turnLeft(int speed) {
  setMotorDirection(false, true);
  if (use_pid) {
    pid_left.target_speed = speed * 1.5;
    pid_right.target_speed = speed * 1.5;
  } else {
    ledcWrite(PWM_CHANNEL_A, speed);
    ledcWrite(PWM_CHANNEL_B, speed);
  }
}

void turnRight(int speed) {
  setMotorDirection(true, false);
  if (use_pid) {
    pid_left.target_speed = speed * 1.5;
    pid_right.target_speed = speed * 1.5;
  } else {
    ledcWrite(PWM_CHANNEL_A, speed);
    ledcWrite(PWM_CHANNEL_B, speed);
  }
}

void processCommand(String command) {
  command.trim();
  
  // Parse command and speed (format: "command:speed")
  int colonIndex = command.indexOf(':');
  String cmd = command;
  int speed = current_speed;
  
  if (colonIndex > 0) {
    cmd = command.substring(0, colonIndex);
    speed = command.substring(colonIndex + 1).toInt();
    current_speed = constrain(speed, 0, 255);
  }
  
  if (cmd == "forward") {
    moveForward(current_speed);
  } else if (cmd == "backward") {
    moveBackward(current_speed);
  } else if (cmd == "left") {
    turnLeft(current_speed);
  } else if (cmd == "right") {
    turnRight(current_speed);
  } else if (cmd == "stop") {
    stopMotors();
  } else {
    Serial.println("Unknown command: " + cmd);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Setup encoder pins
  pinMode(ENCODER_A_RIGHT, INPUT_PULLUP);
  pinMode(ENCODER_B_RIGHT, INPUT_PULLUP);
  pinMode(ENCODER_A_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_B_LEFT, INPUT_PULLUP);
  
  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), encoderISR_Right_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_RIGHT), encoderISR_Right_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT), encoderISR_Left_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_LEFT), encoderISR_Left_B, CHANGE);
  
  // Setup motor control pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  
  // Setup PWM for motor speed control
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_ENA, PWM_CHANNEL_A);
  ledcAttachPin(MOTOR_ENB, PWM_CHANNEL_B);
  
  // Initialize motors to stop
  stopMotors();
  
  // Initialize PID controllers
  pid_left.last_time = millis();
  pid_right.last_time = millis();
  pid_left.last_encoder = encoder_count_left;
  pid_right.last_encoder = encoder_count_right;
  
  Serial.println("ESP32 Robot Controller Ready");
  Serial.println("PID Control: DISABLED (set use_pid=true to enable)");
}

void loop() {
  // Update PID control for both motors (if enabled)
  if (use_pid) {
    static unsigned long last_pid_update = 0;
    if (millis() - last_pid_update >= 50) {  // 20Hz PID update (slower for stability)
      float pwm_left = updatePID(pid_left, encoder_count_left);
      float pwm_right = updatePID(pid_right, encoder_count_right);
      
      ledcWrite(PWM_CHANNEL_A, (int)pwm_left);
      ledcWrite(PWM_CHANNEL_B, (int)pwm_right);
      
      last_pid_update = millis();
    }
  }
  
  // Send encoder values to ROS2 node
  static unsigned long last_encoder_send = 0;
  if (millis() - last_encoder_send >= 100) {  // Send every 100ms
    Serial.print("ENCODER:");
    Serial.print(encoder_count_left);
    Serial.print(",");
    Serial.println(encoder_count_right);
    last_encoder_send = millis();
  }
  
  // Read commands from ROS2 node
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
}