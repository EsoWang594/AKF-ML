#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ====================================================================
// THAM SỐ TOÁN HỌC UKF CỐ ĐỊNH (Baseline)
// ====================================================================
float x_est = 0.0; 
float P_est = 1.0; 
float Q = 0.01;   
float R_fixed = 5.0; // R cố định

// Tham số điểm Sigma (Giữ nguyên cho tính nhất quán)
float alpha = 0.1;
float beta = 2.0;
float kappa = 0.0;
float lambda = alpha * alpha * (1.0 + kappa) - 1.0;
float Wm0 = lambda / (1.0 + lambda);
float Wc0 = (lambda / (1.0 + lambda)) + (1.0 - alpha * alpha + beta);
float Wmi = 1.0 / (2.0 * (1.0 + lambda));
float Wci = 1.0 / (2.0 * (1.0 + lambda));

Adafruit_MPU6050 mpu;
const int BUTTON_PIN = 14;
uint32_t ram_baseline = 0;
unsigned long previous_time = 0;

float update_UKF(float z_measured) {
    float x_predict = x_est; 
    float P_predict = P_est + Q;

    float sigma_margin = sqrt((1.0 + lambda) * P_predict);
    float sigma_pts[3] = {x_predict, x_predict + sigma_margin, x_predict - sigma_margin};
    float z_sigma[3] = {sigma_pts[0], sigma_pts[1], sigma_pts[2]};

    float z_mean = Wm0 * z_sigma[0] + Wmi * z_sigma[1] + Wmi * z_sigma[2];

    float Pyy = Wc0 * (z_sigma[0] - z_mean) * (z_sigma[0] - z_mean) +
                Wci * (z_sigma[1] - z_mean) * (z_sigma[1] - z_mean) +
                Wci * (z_sigma[2] - z_mean) * (z_sigma[2] - z_mean) + R_fixed;

    float Pxy = Wc0 * (sigma_pts[0] - x_predict) * (z_sigma[0] - z_mean) +
                Wci * (sigma_pts[1] - x_predict) * (z_sigma[1] - z_mean) +
                Wci * (sigma_pts[2] - x_predict) * (z_sigma[2] - z_mean);

    float K = Pxy / Pyy;
    x_est = x_predict + K * (z_measured - z_mean);
    P_est = P_predict - K * Pyy * K;
    return x_est;
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT);
    Wire.begin();
    if (!mpu.begin()) while (1) delay(10);
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    
    delay(2000);
    ram_baseline = ESP.getFreeHeap();
    Serial.println("Baseline UKF Ready. Press D14 to start...");
}

void loop() {
    if (digitalRead(BUTTON_PIN) == HIGH) {
        if (millis() - previous_time >= 20) {
            previous_time = millis();

            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            float raw_az = a.acceleration.z; // Giữ nguyên m/s^2

            unsigned long t_start = micros();
            float filtered = update_UKF(raw_az);
            unsigned long t_end = micros();
            
            static int sample_count = 0;
            sample_count++;

            Serial.printf("Mau thu: %04d | Tho: %8.2f | Loc UKF: %8.2f | Time: %7.4f ms | RAM: %6.3f KB\n", 
                          sample_count, raw_az, filtered, 
                          (float)(t_end - t_start)/1000.0, 
                          (float)(ram_baseline - ESP.getFreeHeap())/1024.0);
        }
    }
}