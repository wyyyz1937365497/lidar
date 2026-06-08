#include <Arduino.h>
#include <ESP32Servo.h>

// ==================== 硬件引脚配置 ====================
const int M_PWM[2] = {21, 20}; 
const int M_IN1[2] = {14, 16}; 
const int M_IN2[2] = {15, 17}; 
const int STBY = 41;
#define STEERING_SERVO_PIN 18

const int PWM_FREQ = 10000;
const int PWM_RES = 8;

// 从你之前的标定结果取值
#define MIN_PWM 46
#define MAX_PWM 100

// ==================== 全局变量 ====================
Servo testServo;

int servoCenter = 45;  // 初始中位角度 (根据你之前的配置)
int testPWM = 60;      // 测试用的电机PWM (可在 MIN~MAX 之间调节)
bool isMoving = false; // 电机运行状态标志

// ==================== 电机控制函数 ====================
void setMotor(int index, int pwm_val) {
  if (index < 0 || index > 1) return;
  pwm_val = constrain(pwm_val, -MAX_PWM, MAX_PWM);

  if (pwm_val == 0) {
    digitalWrite(M_IN1[index], LOW);
    digitalWrite(M_IN2[index], LOW);
    ledcWrite(index, 0);
  } else if (pwm_val > 0) {
    digitalWrite(M_IN1[index], HIGH);
    digitalWrite(M_IN2[index], LOW);
    ledcWrite(index, pwm_val);
  } else {
    digitalWrite(M_IN1[index], LOW);
    digitalWrite(M_IN2[index], HIGH);
    ledcWrite(index, -pwm_val);
  }
}

void stopMotors() {
  setMotor(0, 0);
  setMotor(1, 0);
  isMoving = false;
}

void driveForward() {
  setMotor(0, testPWM);
  setMotor(1, testPWM);
  isMoving = true;
}

void driveBackward() {
  setMotor(0, -testPWM);
  setMotor(1, -testPWM);
  isMoving = true;
}

// ==================== Setup & Loop ====================
void setup() {
  Serial.begin(115200);
  delay(500);

  // 初始化电机引脚
  for (int i = 0; i < 2; ++i) {
    pinMode(M_IN1[i], OUTPUT);
    pinMode(M_IN2[i], OUTPUT);
    digitalWrite(M_IN1[i], LOW);
    digitalWrite(M_IN2[i], LOW);
    ledcSetup(i, PWM_FREQ, PWM_RES);
    ledcAttachPin(M_PWM[i], i);
    ledcWrite(i, 0);
  }
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // 启用电机驱动

  // 初始化舵机
  testServo.setPeriodHertz(50);
  testServo.attach(STEERING_SERVO_PIN, 500, 2500);
  testServo.write(servoCenter);

  Serial.println("\n========================================");
  Serial.println("🔧 舵机中位 & 直行跑偏标定工具");
  Serial.println("========================================");
  Serial.println("📌 舵机微调指令:");
  Serial.println("  'a' : 角度 -1 (左微调)");
  Serial.println("  'd' : 角度 +1 (右微调)");
  Serial.println("  'q' : 角度 -5 (左快调)");
  Serial.println("  'e' : 角度 +5 (右快调)");
  Serial.println("📌 电机控制指令:");
  Serial.println("  'w' : 前进 (保持按住或单次触发)");
  Serial.println("  's' : 后退");
  Serial.println("  'x' : 停止电机 🛑");
  Serial.println("📌 速度调整指令:");
  Serial.println("  '+' : 增加测试PWM (+5)");
  Serial.println("  '-' : 降低测试PWM (-5)");
  Serial.println("========================================");
  Serial.printf("🎯 当前舵机中位: %d° | 当前测试PWM: %d\n", servoCenter, testPWM);
  Serial.println("准备就绪！请调整舵机直到车轮正向前方。\n");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // 舵机调整
    if (cmd == 'a' || cmd == 'A') servoCenter -= 1;
    else if (cmd == 'd' || cmd == 'D') servoCenter += 1;
    else if (cmd == 'q' || cmd == 'Q') servoCenter -= 5;
    else if (cmd == 'e' || cmd == 'E') servoCenter += 5;
    
    // 电机控制
    else if (cmd == 'w' || cmd == 'W') driveForward();
    else if (cmd == 's' || cmd == 'S') driveBackward();
    else if (cmd == 'x' || cmd == 'X') stopMotors();
    
    // 速度调整
    else if (cmd == '+' || cmd == '=') testPWM = constrain(testPWM + 5, MIN_PWM, MAX_PWM);
    else if (cmd == '-' || cmd == '_') testPWM = constrain(testPWM - 5, MIN_PWM, MAX_PWM);

    // 更新并限制舵机角度
    servoCenter = constrain(servoCenter, 0, 180);
    testServo.write(servoCenter);

    // 打印当前状态
    Serial.printf("🎯 舵机角度: %d° | ⚡测试PWM: %d | 🚗 电机: %s\n", 
                  servoCenter, testPWM, isMoving ? "运行中" : "已停止");
  }
}
