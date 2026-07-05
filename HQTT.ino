#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ====================================================================
// THAM SỐ HỒI QUY TUYẾN TÍNH (HQTT)
// ====================================================================
const float w_variance = 0.029170;  
const float bias_R     = 0.437255;  

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
    Serial.println("HQTT System Ready. Press D14 to start...");
}

float update_Adaptive_KF_HQTT(float z_measured) {
    // 1. Tính phương sai từ sai số cải tiến
    float innovation = z_measured - x_est;
    float current_var = innovation * innovation;

    // 2. HQTT: R = w*Var + bias
    R_adaptive = (w_variance * current_var) + bias_R;
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
            float filtered = update_Adaptive_KF_HQTT(raw_az);
            unsigned long t_end = micros();

            static int sample_count = 0;
            sample_count++;

            // Xuất dữ liệu thống kê
            Serial.printf("Mau thu: %04d | Tho: %8.2f | Loc KF: %8.2f | Time: %7.4f ms | RAM: %6.3f KB\n", 
                          sample_count, 
                          raw_az, 
                          filtered, 
                          (float)(t_end - t_start)/1000.0, 
                          (float)(ram_baseline - ESP.getFreeHeap())/1024.0);
        }
    }
}