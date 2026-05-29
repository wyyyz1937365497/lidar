#include <Arduino.h>
#include <BitBang_I2C.h>

// MT6701 编码器配置
#define MT6701_ADDR 0x06
#define REG_ANGLE_MSB 0x03
#define MAX_ANGLE_VALUE 16384.0f

// 接在刚才测试 MPU6050 的引脚上
#define SDA_PIN 7
#define SCL_PIN 6

BBI2C myI2C;

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== MT6701 软件I2C读取测试 (引脚 7/6) ===\n");

  // 初始化总线
  memset(&myI2C, 0, sizeof(myI2C));
  myI2C.iSDA = SDA_PIN;
  myI2C.iSCL = SCL_PIN;
  myI2C.bWire = 0;          // BitBang 模式
  I2CInit(&myI2C, 100000L); // 100kHz

  Serial.printf("总线初始化完成 (SDA:%d, SCL:%d)\n", SDA_PIN, SCL_PIN);
  Serial.println("正在读取 MT6701 角度...\n");
}

void loop()
{
  uint8_t buf[2];

  // 读取 MT6701 寄存器 0x03，读取 2 字节
  int bytesRead = I2CReadRegister(&myI2C, MT6701_ADDR, REG_ANGLE_MSB, buf, 2);

  if (bytesRead == 2)
  {
    // 数据解析：14-bit 分辨率
    // buf[0] 是高 8 位，buf[1] 是低 6 位
    uint16_t rawAngle = (buf[0] << 6) | (buf[1] & 0x3F);
    float angle = (rawAngle / MAX_ANGLE_VALUE) * 360.0f;

    Serial.printf("✅ 读取成功 | Raw: [0x%02X, 0x%02X] -> Angle: %.2f°\n", buf[0], buf[1], angle);

    // 简单的跳变检测提示
    if (rawAngle == 0 || rawAngle == 16383)
    {
      Serial.println("  ⚠️ 注意: Raw 值接近极限 (0或16383)，可能未连接或数据异常");
    }
  }
  else
  {
    Serial.printf("❌ 读取失败 (返回: %d) | 请检查 MT6701 接线 (3.3V, GND, SDA, SCL)\n", bytesRead);
  }

  delay(100);
}
