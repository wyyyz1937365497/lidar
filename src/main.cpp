/**
 * @file main.cpp
 * @brief MPU6500/9250 最小化 I2C 诊断程序 (使用 Wire1)
 *        目的：测试 Adafruit 库 + Wire1 是否能正常初始化
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define I2C_SDA 45
#define I2C_SCL 46

Adafruit_MPU6050 mpu;

void setup()
{
  Serial.begin(115200);
  delay(2000); // 等待串口监视器打开
  Serial.println("\n========================================");
  Serial.println("🔧 MPU6500/9250 Wire1 最小化测试");
  Serial.println("========================================\n");

  // 1. 初始化 Wire1 并降频到 100kHz
  Serial.println("[步骤1] 初始化 Wire1 (SDA=45, SCL=46, 100kHz)...");
  Wire1.begin(I2C_SDA, I2C_SCL, 100000);

  // 2. 手动唤醒 MPU
  Serial.println("[步骤2] 尝试手动唤醒 MPU...");
  Wire1.beginTransmission(0x68);
  Wire1.write(0x6B); // PWR_MGMT_1 寄存器
  Wire1.write(0x00); // 清除睡眠位
  uint8_t error = Wire1.endTransmission();

  if (error == 0)
  {
    Serial.println("✅ 手动唤醒成功");
  }
  else
  {
    Serial.printf("❌ 手动唤醒失败, I2C 错误码: %d\n", error);
  }
  delay(100);

  // 3. 底层诊断：检查 WHO_AM_I (验证 Wire1 读写正常)
  Serial.println("[步骤3] 底层读取 WHO_AM_I 寄存器...");
  Wire1.beginTransmission(0x68);
  Wire1.write(0x75);
  Wire1.endTransmission(false);
  Wire1.requestFrom(0x68, (uint8_t)1);
  if (Wire1.available())
  {
    uint8_t who_am_i = Wire1.read();
    Serial.printf("✅ WHO_AM_I 读取成功: 0x%02X (期望 0x70)\n", who_am_i);
  }
  else
  {
    Serial.println("❌ WHO_AM_I 读取失败，I2C 总线可能已死锁");
  }

  // 4. 启动 Adafruit 库 (传入 &Wire1)
  Serial.println("[步骤4] 尝试初始化 Adafruit MPU6050 库 (传入 &Wire1)...");
  if (!mpu.begin(0x68, &Wire1))
  {
    Serial.println("❌ Adafruit 库初始化失败！");
    Serial.println("👉 结论：Adafruit 库内部可能与 Wire1 存在兼容性问题，建议使用纯 Wire1 手写驱动。");
  }
  else
  {
    Serial.println("✅ Adafruit 库初始化成功！");

    // 5. SLAM 专用量程配置
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    Serial.println("✅ 已配置 SLAM 优化量程 (±4G, ±500°/s, 44Hz BW)");
  }

  Serial.println("\n========================================");
  Serial.println("🚀 开始循环读取数据...");
  Serial.println("========================================\n");
}

void loop()
{
  sensors_event_t a, g, temp;

  // 如果库初始化成功，使用库读取
  if (mpu.getEvent(&a, &g, &temp))
  {
    Serial.printf("Accel -> X: %6.2f, Y: %6.2f, Z: %6.2f m/s^2 | ", a.acceleration.x, a.acceleration.y, a.acceleration.z);
    Serial.printf("Gyro -> X: %6.2f, Y: %6.2f, Z: %6.2f rad/s | ", g.gyro.x, g.gyro.y, g.gyro.z);
    Serial.printf("Temp: %.2f C\n", temp.temperature);
  }
  else
  {
    Serial.println("❌ Adafruit 库读取数据失败");

    // 如果库失败，尝试底层直接读取，确认芯片没死
    Wire1.beginTransmission(0x68);
    Wire1.write(0x3B);
    if (Wire1.endTransmission(false) == 0)
    {
      Serial.println("➡️ 但底层 Wire1 直接通信仍然正常，说明是 Adafruit 库的问题。");
    }
    else
    {
      Serial.println("➡️ 底层 Wire1 也断开了，芯片可能已死锁。");
    }
  }

  delay(100); // 10Hz 刷新
}
