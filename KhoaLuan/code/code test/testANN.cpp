#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// --- THÔNG SỐ AI TỪ KẾT QUẢ HUẤN LUYỆN ---
const float X_MEAN = 3.086414, X_STD = 7.049765;
const float Y_MEAN = 1.921039, Y_STD = 2.280008;

// Trọng số ANN
const float W_ih[5] = {-0.25091909, -0.15009727, 0.36993671, 1.51961615, -3.93186994};
const float b_h[5] = {-0.68801096, -0.82694232, 0.95365461, -0.37355693, -0.98834846};
const float W_ho[5] = {-0.95882844, -0.49916183, 0.51496343, -0.12679810, -2.73286153};
const float b_o = 0.81900717;

// --- BIẾN BỘ LỌC KALMAN ---
Adafruit_MPU6050 mpu;
float x_est = 9.8;           // Ước lượng ban đầu (trọng lực)
float P = 1.0;               // Sai số ước lượng
float Q = 0.001;             // Nhiễu hệ thống (giữ thấp để mượt)
float innovation_buffer[50]; // Tăng lên 50 để khớp với chiến thuật ổn định nhất
int buf_idx = 0;

// Hàm kích hoạt ReLU
float relu(float x) { return x > 0 ? x : 0; }

// Hàm thực thi Mạng Nơ-ron (Forward Pass)
float run_ann(float input)
{
  // 1. Chuẩn hóa đầu vào (StandardScaler)
  float scaled_in = (input - X_MEAN) / X_STD;

  // 2. Tính toán Lớp ẩn (Hidden Layer) và Lớp đầu ra (Output Layer)
  float output = 0;
  for (int i = 0; i < 5; i++)
  {
    float hidden_node = relu(scaled_in * W_ih[i] + b_h[i]);
    output += hidden_node * W_ho[i];
  }
  output += b_o;

  // 3. Giải chuẩn hóa đầu ra (Inverse Transform)
  return (output * Y_STD) + Y_MEAN;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  if (!mpu.begin())
  {
    Serial.println("Khong tim thay MPU6050!");
    while (1)
      delay(10);
  }

  // Khởi tạo bộ đệm
  for (int i = 0; i < 50; i++)
    innovation_buffer[i] = 0;

  Serial.println("Gia_toc_tho,Gia_toc_loc,R_Adaptive");
}

void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Chọn trục đo (Ví dụ trục Z là trục chịu trọng lực)
  float z = a.acceleration.z;

  // --- BƯỚC 1: DỰ ĐOÁN (PREDICT) ---
  float x_pred = x_est;
  P = P + Q;

  // --- BƯỚC 2: TÍNH TOÁN ĐẶC TRƯNG CHO AI ---
  float innovation = z - x_pred;
  innovation_buffer[buf_idx] = innovation * innovation;
  buf_idx = (buf_idx + 1) % 50;

  float feature = 0;
  for (int i = 0; i < 50; i++)
    feature += innovation_buffer[i];
  feature /= 50.0; // Đây chính là Variance (Phương sai)

  // --- BƯỚC 3: AI QUYẾT ĐỊNH THAM SỐ R ---
  float R_ann = run_ann(feature);

  // Chốt chặn an toàn cho R (Tránh các giá trị âm hoặc quá cực đoan)
  R_ann = constrain(R_ann, 0.01, 10.0);

  // --- BƯỚC 4: CẬP NHẬT (UPDATE) ---
  float K = P / (P + R_ann); // Hệ số Kalman
  x_est = x_pred + K * innovation;
  P = (1 - K) * P;

  // --- BƯỚC 5: XUẤT DỮ LIỆU ---
  // Định dạng để xem trên Serial Plotter
  Serial.print(z);
  Serial.print(",");
  Serial.print(x_est);
  Serial.print(",");
  Serial.println(R_ann);

  delay(20); // Tốc độ 50Hz, phù hợp để quan sát thay đổi
}