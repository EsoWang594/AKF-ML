#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ====================================================================
// THAM SỐ HỒI QUY ĐA THỨC BẬC 2 (Đã được huấn luyện)
// ====================================================================
const float w_var_sq = -0.001062; 
const float w_var    = 0.065985;  
const float bias_R   = 0.386503;  

Adafruit_MPU6050 mpu;
const int BUTTON_PIN = 14; 

// Kalman Variables
float x_est = 0.0; 
float P_est = 1.0; 
float Q = 0.01;   
float R_adaptive = 1.0; 

uint32_t ram_baseline = 0;
unsigned long previous_time = 0;

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT);
    Wire.begin();

    if (!mpu.begin()) while (1) delay(10);
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    
    delay(2000); // Ổn định hệ thống
    ram_baseline = ESP.getFreeHeap();
    Serial.println("System Ready. Press D14 to start...");
}

float update_Adaptive_KF_HQDT(float z_measured) {
    // 1. Tính phương sai tức thời (không qua bộ lọc trung bình để giữ độ nhạy cao cho HQDT)
    float innovation = z_measured - x_est;
    float current_var = innovation * innovation;

    // 2. Tính R thích nghi bằng đa thức bậc 2: R = a*var^2 + b*var + c
    R_adaptive = (w_var_sq * current_var * current_var) + (w_var * current_var) + bias_R;
    R_adaptive = constrain(R_adaptive, 0.01, 50.0);

    // 3. Kalman Filter (Tuyến tính)
    float P_pred = P_est + Q;
    float K = P_pred / (P_pred + R_adaptive);
    
    x_est = x_est + K * (z_measured - x_est);
    P_est = (1.0 - K) * P_pred;

    return x_est;
}

void loop() {
    if (digitalRead(BUTTON_PIN) == HIGH) {
        if (millis() - previous_time >= 20) { // 50Hz
            previous_time = millis();

            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            // Giữ nguyên gia tốc Z (m/s^2)
            float raw_az = a.acceleration.z; 

            unsigned long t_start = micros();
            float filtered = update_Adaptive_KF_HQDT(raw_az);
            unsigned long t_end = micros();

            // Định dạng đầu ra cố định để vẽ biểu đồ và thống kê
            static int sample_count = 0;
            sample_count++;

            Serial.printf("Mau thu: %04d | Tho: %8.2f | Loc KF: %8.2f | Time: %7.4f ms | RAM: %6.3f KB\n", 
                          sample_count, 
                          raw_az, 
                          filtered, 
                          (float)(t_end - t_start)/1000.0, 
                          (float)(ram_baseline - ESP.getFreeHeap())/1024.0);
        }
    }
}