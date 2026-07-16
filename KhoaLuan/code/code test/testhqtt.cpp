#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// --- THÔNG SỐ AI: HỒI QUY TUYẾN TÍNH ---
// Phương trình: R = 0.185983 * Variance + 1.347017
const float W_SENSITIVITY = 0.185983; // Hệ số w (độ nhạy)
const float B_BASE = 1.347017;        // Hệ số b (giá trị cơ bản)

// --- BIẾN BỘ LỌC KALMAN ---
Adafruit_MPU6050 mpu;
float x_est = 9.8;
float P = 1.0;
float Q = 0.01; // Nhiễu hệ thống (Process Noise)
float innovation_buffer[50];
int buf_idx = 0;

// Bộ lọc làm mượt cho tham số R (để đồ thị đẹp hơn)
float R_prev = B_BASE;
const float alpha_R = 0.15;

void setup()
{
  Serial.begin(115200);
  while (!mpu.begin())
  {
    Serial.println("Loi: Khong tim thay MPU6050!");
    delay(1000);
  }

  // Khởi tạo bộ đệm 50 mẫu
  for (int i = 0; i < 50; i++)
    innovation_buffer[i] = 0;

  Serial.println("Tho,Loc,R_TuyenTinh");
}

void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Đọc trục Z (Trục đo trọng lực)
  float z = a.acceleration.z;

  // --- BƯỚC 1: DỰ ĐOÁN (PREDICT) ---
  float x_pred = x_est;
  P = P + Q;

  // --- BƯỚC 2: TÍNH VARIANCE (PHƯƠNG SAI) ---
  float innovation = z - x_pred;
  innovation_buffer[buf_idx] = innovation * innovation;
  buf_idx = (buf_idx + 1) % 50;

  float variance = 0;
  for (int i = 0; i < 50; i++)
    variance += innovation_buffer[i];
  variance /= 50.0;

  // --- BƯỚC 3: TÍNH R THEO PHƯƠNG TRÌNH THÍCH NGHI ---
  // R = w * variance + b
  float R_raw = (W_SENSITIVITY * variance) + B_BASE;

  // Chốt chặn an toàn cho R
  float R_constrained = constrain(R_raw, 0.1, 15.0);

  // Làm mượt R để tránh đáp ứng quá gắt với nhiễu gai
  float R_adaptive = (alpha_R * R_constrained) + ((1.0 - alpha_R) * R_prev);
  R_prev = R_adaptive;

  // --- BƯỚC 4: CẬP NHẬT KALMAN (UPDATE) ---
  float K = P / (P + R_adaptive);
  x_est = x_pred + K * innovation;
  P = (1 - K) * P;

  // --- BƯỚC 5: XUẤT DỮ LIỆU ---
  // Định dạng này giúp Serial Plotter vẽ 3 đường tách biệt
  Serial.print(z);
  Serial.print(",");
  Serial.print(x_est);
  Serial.print(",");
  Serial.println(R_adaptive);

  delay(20); // Tần số lấy mẫu 50Hz
}