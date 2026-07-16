#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// --- CÔNG THỨC AI HỒI QUY ĐA THỨC ---
// R = (-0.006591 * var^2) + (0.414549 * var) + 1.031923
const float W2 = -0.006591;
const float W1 = 0.414549;
const float B = 1.031923;

// --- BIẾN BỘ LỌC KALMAN ---
Adafruit_MPU6050 mpu;
float x_est = 9.8;
float P = 1.0;
float Q = 0.01; // Nhiễu hệ thống (có thể chỉnh từ 0.001 đến 0.05)
float innovation_buffer[50];
int buf_idx = 0;

// Biến làm mượt R (Low-pass filter cho tham số R)
float R_prev = 1.0;
const float alpha_R = 0.1; // Hệ số làm mượt R (càng nhỏ càng mượt)

void setup()
{
  Serial.begin(115200);
  while (!mpu.begin())
  {
    Serial.println("Khong tim thay MPU6050!");
    delay(1000);
  }

  for (int i = 0; i < 50; i++)
    innovation_buffer[i] = 0;
  Serial.println("Gia_toc_tho,Gia_toc_loc,R_AI_HoiQuy");
}

void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Đọc trục Z (Trục chịu trọng lực 9.8)
  float z = a.acceleration.z;

  // --- BƯỚC 1: DỰ ĐOÁN (PREDICT) ---
  float x_pred = x_est;
  P = P + Q;

  // --- BƯỚC 2: TÍNH TOÁN BIẾN SỐ 'VAR' (PHƯƠNG SAI) ---
  float innovation = z - x_pred;
  innovation_buffer[buf_idx] = innovation * innovation;
  buf_idx = (buf_idx + 1) % 50;

  float var = 0;
  for (int i = 0; i < 50; i++)
    var += innovation_buffer[i];
  var /= 50.0;

  // --- BƯỚC 3: TÍNH R THEO CÔNG THỨC AI HỒI QUY ---
  // R = (W2 * var^2) + (W1 * var) + B
  float R_raw = (W2 * var * var) + (W1 * var) + B;

  // Chốt chặn an toàn (R không được âm và không nên quá lớn gây trễ nặng)
  float R_constrained = constrain(R_raw, 0.1, 10.0);

  // Làm mượt sự thay đổi của R để đường lọc không bị giật
  float R_adaptive = (alpha_R * R_constrained) + ((1.0 - alpha_R) * R_prev);
  R_prev = R_adaptive;

  // --- BƯỚC 4: CẬP NHẬT KALMAN (UPDATE) ---
  float K = P / (P + R_adaptive);
  x_est = x_pred + K * innovation;
  P = (1 - K) * P;

  // --- BƯỚC 5: XUẤT DỮ LIỆU ---
  Serial.print(z);
  Serial.print(",");
  Serial.print(x_est);
  Serial.print(",");
  Serial.println(R_adaptive); // Chia hoặc nhân nếu muốn quan sát dải R rộng hơn

  delay(20); // Tần số 50Hz
}