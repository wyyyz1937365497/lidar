#include <Arduino.h>

// ==================== 硬件引脚配置 ====================
const int M_PWM[2] = {21, 20};
const int M_IN1[2] = {14, 16}; // 对应 INA
const int M_IN2[2] = {15, 17}; // 对应 INB
const int STBY = 41;

#define ENC_RL_PULSE_PIN 4
#define ENC_RL_DIR_PIN 5
#define ENC_RR_PULSE_PIN 13
#define ENC_RR_DIR_PIN 12

#define ENC_RL_DIR 1.0f
#define ENC_RR_DIR 1.0f

#define ENC_PULSES_PER_REV 2048
#define MOTOR_ENCODER_RATIO 2.0f
#define WHEEL_RADIUS 0.025f

const int PWM_FREQ = 10000;
const int PWM_RES = 8;

// ==================== PID 参数 ====================
float Kp = 30.0;
float Ki = 1.0;
float Kd = 0.05;

#define MAX_INTEGRAL_PWM 60.0
#define MIN_START_PWM 46

// ==================== 全局变量 ====================
volatile int64_t enc_rl_count = 0;
volatile int64_t enc_rr_count = 0;

float target_v_left = 0.0;
float target_v_right = 0.0;

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
  int direction = (ENC_RR_DIR > 0) ? (dirState ? 1 : -1) : (dirState ? -1 : 1);
  enc_rr_count -= direction; // 原来是 +=，改为 -= 实现取反
}
// ==================== 底层电机控制 (INAB + PWM) ====================
void setMotorDirectionAndSpeed(int index, int speed_pwm)
{
  speed_pwm = constrain(speed_pwm, -255, 255);

  if (abs(speed_pwm) < 5)
  {
    digitalWrite(M_IN1[index], LOW);
    digitalWrite(M_IN2[index], LOW);
    ledcWrite(index, 0);
    return;
  }

  // 🚨 核心修正：反转 IN1/IN2 逻辑，让正 PWM 对应物理“向前”
  if (speed_pwm > 0)
  {
    digitalWrite(M_IN1[index], HIGH); // INA=1
    digitalWrite(M_IN2[index], LOW);  // INB=0  → Forward (向前)
  }
  else
  {
    digitalWrite(M_IN1[index], LOW);  // INA=0
    digitalWrite(M_IN2[index], HIGH); // INB=1  → Reverse (向后)
  }

  // PWM 控制速度大小（始终为正）
  ledcWrite(index, abs(speed_pwm));
}

void setup()
{
  Serial.begin(115200);
  delay(500);

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

  pinMode(ENC_RL_PULSE_PIN, INPUT_PULLUP);
  pinMode(ENC_RL_DIR_PIN, INPUT_PULLUP);
  attachInterrupt(ENC_RL_PULSE_PIN, encRL_ISR, RISING);

  pinMode(ENC_RR_PULSE_PIN, INPUT_PULLUP);
  pinMode(ENC_RR_DIR_PIN, INPUT_PULLUP);
  attachInterrupt(ENC_RR_PULSE_PIN, encRR_ISR, RISING);

  last_pid_time = millis();

  Serial.println("🚗 Motor PID Test Ready (INAB + PWM Control)");
  Serial.println("Commands: L0.3 R0.3 (Set speed m/s) | P30 I1 D0.05 (Set PID) | S (Stop)");
}

void loop()
{
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
        integral[0] = integral[1] = 0;
        Serial.printf("🔄 PID Updated -> P:%.2f I:%.2f D:%.2f\n", Kp, Ki, Kd);
      }
    }
    else if (msg == "S")
    {
      target_v_left = target_v_right = 0;
      integral[0] = integral[1] = 0;
    }
  }

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

    static int print_counter = 0;
    print_counter++;
    bool should_print = (print_counter % 5 == 0);

    for (int i = 0; i < 2; i++)
    {
      int64_t delta = current_counts[i] - last_enc_count[i];
      last_enc_count[i] = current_counts[i];

      float wheel_revolutions = (float)delta / (ENC_PULSES_PER_REV * MOTOR_ENCODER_RATIO);
      float raw_speed = wheel_revolutions * (2.0f * M_PI * WHEEL_RADIUS) / dt;
      actual_v[i] = raw_speed * dir_factors[i];

      float target_v = (i == 0) ? target_v_left : target_v_right;
      float error = target_v - actual_v[i];

      if (error * integral[i] >= 0 && abs(Kp * error + Ki * integral[i]) < 240)
      {
        integral[i] += error * dt;
      }
      if (abs(Ki * integral[i]) > MAX_INTEGRAL_PWM)
      {
        integral[i] = (integral[i] > 0 ? 1 : -1) * MAX_INTEGRAL_PWM / Ki;
      }

      float derivative = (error - prev_error[i]) / dt;
      prev_error[i] = error;

      // PID 计算出的是速度值，需要转换为 PWM
      int speed_pwm = (int)(Kp * error + Ki * integral[i] + Kd * derivative);

      // 死区补偿
      if (abs(speed_pwm) >= 5 && abs(speed_pwm) < MIN_START_PWM)
      {
        speed_pwm = (speed_pwm > 0) ? MIN_START_PWM : -MIN_START_PWM;
      }

      setMotorDirectionAndSpeed(i, speed_pwm);

      if (should_print)
      {
        if (i == 0)
        {
          // 🚨 修改：打印 PWM 绝对值，避免负值
          Serial.printf("[L] dta:%5d spd:%.3f err:%.3f intg:%.2f | pwm:%4d || ",
                        (int)delta, actual_v[i], error, integral[i], abs(speed_pwm));
        }
        else
        {
          //  修改：打印 PWM 绝对值，避免负值
          Serial.printf("[R] dta:%5d spd:%.3f err:%.3f intg:%.2f | pwm:%4d\n",
                        (int)delta, actual_v[i], error, integral[i], abs(speed_pwm));
        }
      }
    }
  }
}