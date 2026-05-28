#include <Arduino.h>
#include <Wire.h>

// ================= I2C 配置 =================
// MT6701 默认 7位 I2C 地址为 0x06（部分模块可能固定为 0x36 或 0x1A）
const uint8_t MT6701_I2C_ADDR = 0x06;

// 角度寄存器地址（官方数据手册定义）
const uint8_t REG_ANGLE_MSB = 0x03; // 高8位寄存器
const uint8_t REG_ANGLE_LSB = 0x04; // 低6位寄存器

// MT6701 为 14bit 分辨率，满量程值为 16384 (2^14)
const float MAX_ANGLE_VALUE = 16384.0;

// ESP32-S3 DevKitC-1 常用 I2C 引脚（可根据实际接线修改）
const int I2C_SDA = 11;
const int I2C_SCL = 0;

// const int ENC_SDA[4] = {1, 8, 11, 45}; // FL, FR, RL, RR
// const int ENC_SCL[4] = {2, 3, 0, 46};

void setup()
{
  Serial.begin(115200);
  delay(500); // 等待串口稳定

  // 初始化 I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // 400kHz 高速模式

  // 检测 MT6701 是否在线
  Wire.beginTransmission(MT6701_I2C_ADDR);
  if (Wire.endTransmission() == 0)
  {
    Serial.println("✅ MT6701 设备已就绪！");
  }
  else
  {
    Serial.println("❌ 错误：未检测到 MT6701，请检查接线或 I2C 地址！");
    while (1)
      delay(1000); // 暂停运行
  }

  Serial.println("========================================");
  Serial.println("MT6701 I2C 绝对角度读取程序");
  Serial.println("请转动磁环或磁铁...");
  Serial.println("========================================");
}

void loop()
{
  // 1. 发送寄存器指针
  Wire.beginTransmission(MT6701_I2C_ADDR);
  Wire.write(REG_ANGLE_MSB);
  if (Wire.endTransmission(false) != 0)
  {
    Serial.println("⚠️ I2C 写入失败");
    delay(100);
    return;
  }

  // 2. 读取 2 个字节数据
  uint8_t bytesRead = Wire.requestFrom(MT6701_I2C_ADDR, (uint8_t)2);
  if (bytesRead != 2)
  {
    Serial.println("⚠️ I2C 读取失败");
    delay(100);
    return;
  }

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();

  // 3. 合并为 14bit 原始值
  // MSB 占高8位，LSB 仅低6位有效（高2位为状态位）
  uint16_t rawAngle = (msb << 6) | (lsb & 0x3F);

  // 4. 转换为角度 (0.00 ~ 360.00)
  float angle = (rawAngle / MAX_ANGLE_VALUE) * 360.0;

  // 5. 串口打印
  Serial.print("原始值(14bit): ");
  Serial.print(rawAngle);
  Serial.print("\t角度: ");
  Serial.print(angle, 2);
  Serial.println(" °");

  delay(50); // 约 20Hz 采样率
}