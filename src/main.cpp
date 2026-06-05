#include <Arduino.h>
#include <ESP32Servo.h>

// 定义舵机引脚（与你的主程序一致）
#define STEERING_SERVO_PIN 18

Servo testServo;

// 当前舵机角度 (默认从中位90度开始，标准舵机范围0-180度)
int currentAngle = 45;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // 配置舵机参数（与主程序一致）
  testServo.setPeriodHertz(50);
  testServo.attach(STEERING_SERVO_PIN, 500, 2500);

  // 先回到中位
  testServo.write(currentAngle);

  Serial.println("\n========================================");
  Serial.println("🔧 舵机串口调试工具已启动");
  Serial.println("========================================");
  Serial.println("📌 指令说明:");
  Serial.println(" - 输入 0~180 之间的数字，舵机转到对应绝对角度");
  Serial.println(" - 输入 'a' 或 'A'，角度减少 1 度 (微调左转)");
  Serial.println(" - 输入 'd' 或 'D'，角度增加 1 度 (微调右转)");
  Serial.println(" - 输入 'q' 或 'Q'，角度减少 5 度 (快调左转)");
  Serial.println(" - 输入 'e' 或 'E'，角度增加 5 度 (快调右转)");
  Serial.println(" - 输入 'c' 或 'C'，回到中位 (90度)");
  Serial.println("========================================");
  Serial.printf("🎯 当前角度: %d°\n", currentAngle);
  Serial.println("请输入指令:");
}

void loop()
{
  if (Serial.available() > 0)
  {
    String input = Serial.readStringUntil('\n');
    input.trim(); // 去除首尾空格和换行符

    if (input.length() == 0)
      return;

    // 1. 处理单字符指令
    if (input.length() == 1)
    {
      char cmd = input.charAt(0);
      switch (cmd)
      {
      case 'a':
      case 'A':
        currentAngle -= 1;
        break;
      case 'd':
      case 'D':
        currentAngle += 1;
        break;
      case 'q':
      case 'Q':
        currentAngle -= 5;
        break;
      case 'e':
      case 'E':
        currentAngle += 5;
        break;
      case 'c':
      case 'C':
        currentAngle = 45;
        break;
      default:
        break;
      }
    }
    // 2. 处理数字指令
    else
    {
      int targetAngle = input.toInt();
      if (targetAngle >= 0 && targetAngle <= 180)
      {
        currentAngle = targetAngle;
      }
      else
      {
        Serial.println("⚠️ 角度超出范围 (0-180)，请重新输入！");
        return;
      }
    }

    // 安全限幅：防止输入数字超出 0-180 损坏舵机
    currentAngle = constrain(currentAngle, 0, 180);

    // 执行转动
    testServo.write(currentAngle);

    // 打印反馈
    Serial.printf("➡️ 舵机角度设置为: %d°\n", currentAngle);

    // 倾斜角度换算提示（假设90度是0度偏角）
    float steering_offset = currentAngle - 90.0;
    Serial.printf("   (相对中位偏转: %.1f°)\n", steering_offset);
  }
}
