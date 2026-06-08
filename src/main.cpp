#include <Arduino.h>

// ==================== 硬件引脚配置 ====================
const int M_PWM[2] = {21, 20};
const int M_IN1[2] = {14, 16};
const int M_IN2[2] = {15, 17};
const int STBY = 41;

#define ENC_RL_PULSE_PIN 4
#define ENC_RL_DIR_PIN 5
#define ENC_RR_PULSE_PIN 13
#define ENC_RR_DIR_PIN 12

#define ENC_RL_DIR -1.0f
#define ENC_RR_DIR 1.0f
#define ENC_PULSES_PER_REV 2048
#define WHEEL_RADIUS 0.025f

const int PWM_FREQ = 10000;
const int PWM_RES = 8;

// ==================== PID 参数 (需根据实际电机调整) ====================
float Kp = 20.0;
float Ki = 0.5;
float Kd = 0.1;
#define PID_INTEGRAL_LIMIT 100.0 // 积分限幅，防饱和

// ==================== 全局变量 ====================
volatile int64_t enc_rl_count = 0;
volatile int64_t enc_rr_count = 0;

float target_v_left = 0.0;
float target_v_right = 0.0;

// PID 状态变量
float integral[2] = {0.0, 0.0};
float prev_error[2] = {0.0, 0.0};
int64_t last_enc_count[2] = {0, 0};

unsigned long last_pid_time = 0;

// ==================== 中断服务程序 ====================
void IRAM_ATTR encRL_ISR()
{
  bool dirState = digitalRead(ENC_RL_DIR_PIN);
  enc_rl_count += (ENC_RL_DIR > 0) ? (dirState ? 1 : -1) : (dirState ? -1 : 1);
}

void IRAM_ATTR encRR_ISR()
{
  bool dirState = digitalRead(ENC_RR_DIR_PIN);
  enc_rr_count += (ENC_RR_DIR > 0) ? (dirState ? 1 : -1) : (dirState ? -1 : 1);
}

// ==================== 底层电机 PWM 控制 ====================
void setMotorPWM(int index, int pwm)
{
  pwm = constrain(pwm, -255, 255);
  if (abs(pwm) < 5)
  { // 克服死区，PWM过小时直接抱死
    digitalWrite(M_IN1[index], LOW);
    digitalWrite(M_IN2[index], LOW);
    ledcWrite(index, 0);
    return;
  }
  if (pwm > 0)
  {
    digitalWrite(M_IN1[index], HIGH);
    digitalWrite(M_IN2[index], LOW);
  }
  else
  {
    digitalWrite(M_IN1[index], LOW);
    digitalWrite(M_IN2[index], HIGH);
  }
  ledcWrite(index, abs(pwm));
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  // 电机初始化
  for (int i = 0; i < 2; ++i)
  {
    pinMode(M_IN1[i], OUTPUT);
    pinMode(M_IN2[i], OUTPUT);
    digitalWrite(M_IN1[i], LOW);
    digitalWrite(M_IN2[i], LOW);
    ledcSetup(i, PWM_FREQ, PWM_RES);
    ledcAttachPin(M_PWM[i], i);
    ledcWrite(i, 0);
  }
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // 编码器初始化
  pinMode(ENC_RL_PULSE_PIN, INPUT_PULLUP);
  pinMode(ENC_RL_DIR_PIN, INPUT_PULLUP);
  attachInterrupt(ENC_RL_PULSE_PIN, encRL_ISR, RISING);

  pinMode(ENC_RR_PULSE_PIN, INPUT_PULLUP);
  pinMode(ENC_RR_DIR_PIN, INPUT_PULLUP);
  attachInterrupt(ENC_RR_PULSE_PIN, encRR_ISR, RISING);

  last_pid_time = millis();
  Serial.println("\n🚗 Motor PID Test Ready");
  Serial.println("Commands: L0.5 R0.3 (Set speed m/s) | P20 I0.5 D0.1 (Set PID) | S (Stop)");
}

void loop()
{
  // 1. 串口指令解析
  if (Serial.available())
  {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    if (msg.startsWith("L"))
    {
      int rIdx = msg.indexOf('R');
      if (rIdx > 0)
      {
        target_v_left = msg.substring(1, rIdx).toFloat();
        target_v_right = msg.substring(rIdx + 1).toFloat();
      }
    }
    else if (msg.startsWith("P"))
    {
      int iIdx = msg.indexOf('I');
      int dIdx = msg.indexOf('D');
      if (iIdx > 0 && dIdx > 0)
      {
        Kp = msg.substring(1, iIdx).toFloat();
        Ki = msg.substring(iIdx + 1, dIdx).toFloat();
        Kd = msg.substring(dIdx + 1).toFloat();
        integral[0] = integral[1] = 0; // 改参后清空积分
        Serial.printf("🔄 PID Updated -> P:%.2f I:%.2f D:%.2f\n", Kp, Ki, Kd);
      }
    }
    else if (msg == "S")
    {
      target_v_left = target_v_right = 0;
    }
  }

  // 2. 50Hz (20ms) PID 计算
  unsigned long now = millis();
  if (now - last_pid_time >= 20)
  {
    float dt = (now - last_pid_time) / 1000.0f;
    last_pid_time = now;

    noInterrupts();
    int64_t current_counts[2] = {enc_rl_count, enc_rr_count};
    interrupts();

    float dir_factors[2] = {ENC_RL_DIR, ENC_RR_DIR};
    float actual_v[2];

    for (int i = 0; i < 2; i++)
    {
      int64_t delta = current_counts[i] - last_enc_count[i];
      last_enc_count[i] = current_counts[i];

      // 计算实际速度并修正方向（物理前进为正）
      float raw_speed = (float)delta / ENC_PULSES_PER_REV * (2.0f * M_PI * WHEEL_RADIUS) / dt;
      actual_v[i] = raw_speed * dir_factors[i];

      // PID 运算
      float target_v = (i == 0) ? target_v_left : target_v_right;
      float error = target_v - actual_v[i];

      integral[i] += error * dt;
      integral[i] = constrain(integral[i], -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT); // 积分限幅

      float derivative = (error - prev_error[i]) / dt;
      prev_error[i] = error;

      int pwm = (int)(Kp * error + Ki * integral[i] + Kd * derivative);
      setMotorPWM(i, pwm);
    }

    // 3. 打印用于 Serial Plotter 的波形
    Serial.printf("TL:%.3f,AL:%.3f,TR:%.3f,AR:%.3f\n",
                  target_v_left, actual_v[0], target_v_right, actual_v[1]);
  }
}
