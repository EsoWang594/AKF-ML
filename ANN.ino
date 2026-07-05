#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// [Giữ nguyên cấu trúc trọng số ANN của bạn]
const float w_input_to_hidden[5] = { -0.250919, -2.762200, 1.321214, 2.836193, -8.676693 };
const float b_hidden[5]          = { -0.688011, -1.299231, 1.217105, 1.127394, -3.463029 };
const float w_hidden_to_output[5] = { -0.958828, -1.600103, 1.132761, -0.527683, -6.108320 };
const float b_output              = 0.283017;

Adafruit_MPU6050 mpu;
const int BUTTON_PIN = 14; 
float x_est = 0.0, P_est = 1.0, Q = 0.001, R_adaptive = 1.0;
float var_buffer[10] = {0}; 
int var_idx = 0;

uint32_t ram_baseline = 0; // RAM sau khi khởi tạo xong
unsigned long previous_time = 0;
const unsigned long INTERVAL_MS = 20; // Tăng nhẹ để Serial không bị nghẽn

float update_Adaptive_KF_ANN(float z_measured) {
    float innovation = z_measured - x_est;
    float current_var = innovation * innovation;
    var_buffer[var_idx] = current_var;
    var_idx = (var_idx + 1) % 10;
    float x_scaled = 0;
    for(int i = 0; i < 10; i++) x_scaled += var_buffer[i];
    x_scaled /= 10.0;

    float h[5];
    for (int i = 0; i < 5; i++) {
        float net = (w_input_to_hidden[i] * x_scaled) + b_hidden[i];
        h[i] = (net > 0.0) ? net : 0.0;
    }
    R_adaptive = b_output;
    for (int i = 0; i < 5; i++) R_adaptive += w_hidden_to_output[i] * h[i];
    R_adaptive = constrain(R_adaptive, 0.001, 10.0);

    float P_pred = P_est + Q;
    float K = P_pred / (P_pred + R_adaptive);
    x_est = x_est + K * (z_measured - x_est);
    P_est = (1.0 - K) * P_pred;
    return x_est;
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT);
    Wire.begin();
    if (!mpu.begin()) while (1) delay(10);
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    
    // Đợi 2s để các tiến trình khởi tạo thư viện ổn định trước khi chốt RAM
    delay(2000);
    ram_baseline = ESP.getFreeHeap(); 
    Serial.println("System Ready. Press D14 to start.");
}

void loop() {
    if (digitalRead(BUTTON_PIN) == HIGH) {
        if (millis() - previous_time >= INTERVAL_MS) {
            previous_time = millis();

            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            float raw_az = (a.acceleration.z / 9.80665);

            unsigned long t_start = micros();
            float filtered = update_Adaptive_KF_ANN(raw_az);
            unsigned long t_end = micros();
            
            static int sample_count = 0;
            sample_count++;

            // Sử dụng printf để định dạng cố định số ký tự
            // %d: số nguyên, %.2f: float 2 chữ số, %.4f: float 4 chữ số
            Serial.printf("Mau thu: %04d | Tho: %6.2f | Loc KF: %6.2f | Time: %7.4f ms | RAM: %6.3f KB\n", 
                          sample_count, 
                          raw_az, 
                          filtered, 
                          (float)(t_end - t_start)/1000.0, 
                          (float)(ram_baseline - ESP.getFreeHeap())/1024.0);
        }
    }
}