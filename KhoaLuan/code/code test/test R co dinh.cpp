#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// --- THÔNG SỐ BỘ LỌC KALMAN CỐ ĐỊNH ---
const float R_FIXED = 0.5;    // Nhiễu đo lường cố định (Measurement Noise)
const float Q_PROCESS = 0.01; // Nhiễu hệ thống (Process Noise)

Adafruit_MPU6050 mpu;
float x_est = 9.8; // Giá trị ước lượng ban đầu
float P = 1.0;     // Sai số ước lượng ban đầu

void setup()
{
  Serial.begin(115200);

  // Khởi tạo cảm biến MPU6050
  if (!mpu.begin())
  {
    Serial.println("Loi: Khong tim thay MPU6050!");
    while (1)
      delay(10);
  }

  // Cấu hình dải đo (Tùy chọn để tăng độ chính xác)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  Serial.println("Gia_toc_tho,Gia_toc_loc_Fixed_R");
}

void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Đọc trục Z (Trục chịu trọng lực)
  float z = a.acceleration.z;

  // --- THUẬT TOÁN KALMAN CỐ ĐỊNH (FIXED) ---

  // 1. Bước Dự đoán (Predict)
  float x_pred = x_est;
  P = P + Q_PROCESS;

  // 2. Bước Cập nhật (Update)
  // Sử dụng R_FIXED = 0.5 xuyên suốt
  float K = P / (P + R_FIXED);       // Tính hệ số Kalman (Gain)
  x_est = x_pred + K * (z - x_pred); // Cập nhật giá trị ước lượng
  P = (1 - K) * P;                   // Cập nhật sai số ước lượng

  // --- XUẤT DỮ LIỆU ---
  // In ra Serial Plotter để so sánh
  Serial.print(z);
  Serial.print(",");     // Đường màu xanh: Nhiễu thô
  Serial.println(x_est); // Đường màu đỏ: Lọc cố định R=0.5

  delay(20); // Tốc độ lấy mẫu 50Hz để đồng bộ với các bản AI
}