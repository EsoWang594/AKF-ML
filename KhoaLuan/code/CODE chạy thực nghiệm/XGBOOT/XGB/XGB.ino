#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>      
#include "xgb_model_code.h" 

// Khởi tạo đối tượng cảm biến theo thư viện Adafruit
Adafruit_MPU6050 mpu;

// Kế thừa cấu hình chân nút bấm
const int BUTTON_PIN = 14; // Chân D14 (GPIO14)

// ====================================================================
// KHAI BÁO CÁC THAM SỐ TOÁN HỌC CHO BỘ LỌC UKF
// ====================================================================
float x_est = 0.0;       
float P_est = 1.0;       
float Q = 0.01;          
float R_adaptive = 1.0;   

float alpha = 0.1;
float beta = 2.0;
float kappa = 0.0;
float lambda = alpha * alpha * (1.0 + kappa) - 1.0;

float Wm0 = lambda / (1.0 + lambda);
float Wc0 = (lambda / (1.0 + lambda)) + (1.0 - alpha * alpha + beta);
float Wmi = 1.0 / (2.0 * (1.0 + lambda));
float Wci = 1.0 / (2.0 * (1.0 + lambda));

const int WINDOW_SIZE = 15;
float window_buffer[WINDOW_SIZE];
int buffer_index = 0;
int current_elements = 0;

uint32_t initial_free_ram = 0;
unsigned long previous_time = 0;
const unsigned long INTERVAL_MS = 10; 

// ====================================================================
// HÀM ĐỌC DỮ LIỆU CẢM BIẾN THỰC TẾ (SỬ DỤNG ĐÚNG CHUẨN ADAFRUIT)
// ====================================================================
float read_raw_sensor() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Thư viện Adafruit mặc định trả về đơn vị m/s^2. 
    // Ta chia cho 9.80665 để quy đổi về đơn vị gia tốc trọng trường (g) đồng bộ với UKF.
    float az_g = a.acceleration.z / 9.80665; 
    
    return az_g;
}

// ====================================================================
// HÀM CẬP NHẬT PHƯƠNG AIS CỬA SỔ TRƯỢT
// ====================================================================
float update_moving_variance(float new_sample) {
    window_buffer[buffer_index] = new_sample;
    buffer_index = (buffer_index + 1) % WINDOW_SIZE;
    if (current_elements < WINDOW_SIZE) {
        current_elements++;
    }

    float sum = 0.0;
    for (int i = 0; i < current_elements; i++) {
        sum += window_buffer[i];
    }
    float mean = sum / current_elements;

    float sum_sq_diff = 0.0;
    for (int i = 0; i < current_elements; i++) {
        float diff = window_buffer[i] - mean;
        sum_sq_diff += diff * diff;
    }
    return sum_sq_diff / current_elements;
}

// ====================================================================
// THUẬT TOÁN ADAPTIVE UKF KẾ T HỢP DỰ BÁO NHÃN R TỪ XGBOOST
// ====================================================================
float update_Adaptive_UKF_XGBoost(float z_measured) {
    float current_variance = update_moving_variance(z_measured);

    double xgb_input[1] = { (double)current_variance }; 
    double xgb_prediction = score(xgb_input); 

    R_adaptive = (float)xgb_prediction;
    if (R_adaptive < 0.01) R_adaptive = 0.01;
    if (R_adaptive > 50.0) R_adaptive = 50.0;

    float x_predict = x_est; 
    float P_predict = P_est + Q;

    float sigma_margin = sqrt((1.0 + lambda) * P_predict);
    float sigma_pts[3];
    sigma_pts[0] = x_predict;
    sigma_pts[1] = x_predict + sigma_margin;
    sigma_pts[2] = x_predict - sigma_margin;

    float z_sigma[3];
    for(int i=0; i<3; i++) {
        z_sigma[i] = sigma_pts[i]; 
    }

    float z_mean = Wm0 * z_sigma[0] + Wmi * z_sigma[1] + Wmi * z_sigma[2];

    float Pyy = Wc0 * (z_sigma[0] - z_mean) * (z_sigma[0] - z_mean) +
                Wci * (z_sigma[1] - z_mean) * (z_sigma[1] - z_mean) +
                Wci * (z_sigma[2] - z_mean) * (z_sigma[2] - z_mean) + R_adaptive;

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
    
    // Cấu hình chân nút bấm D14 là INPUT
    pinMode(BUTTON_PIN, INPUT);
    
    Serial.println("Dang khoi tao cam bien MPU6050 (Adafruit)...");
    
    // Khởi tạo và kiểm tra kết nối phần cứng theo chuẩn Adafruit
    if (!mpu.begin()) {
        Serial.println("MPU6050 ket noi THAT BAI! Vui long kiem tra lai day SDA/SCL.");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 ket noi THANH CONG!");

    // Cấu hình dải đo tối ưu cho bài toán (Có thể tùy chỉnh nếu cần)
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    for(int i = 0; i < WINDOW_SIZE; i++) {
        window_buffer[i] = 0.0;
    }
    
    initial_free_ram = ESP.getFreeHeap();
    previous_time = millis();
}

void loop() {
    int buttonState = digitalRead(BUTTON_PIN);
    unsigned long current_time = millis();
    
    if (buttonState == HIGH) {
        if (current_time - previous_time >= INTERVAL_MS) {
            previous_time = current_time; 

            float raw_accZ = read_raw_sensor();
            
            unsigned long start_time = micros();   
            float filtered_accZ = update_Adaptive_UKF_XGBoost(raw_accZ); 
            unsigned long end_time = micros();     

            float execution_time_ms = (end_time - start_time) / 1000.0;
            uint32_t current_free_ram = ESP.getFreeHeap();
            float ram_consumed_kb = (float)(initial_free_ram - current_free_ram) / 1024.0;
            if (ram_consumed_kb <= 0.0) {
                ram_consumed_kb = (float)(sizeof(window_buffer) + 128) / 1024.0;
            }

            static int sample_count = 0;
            sample_count++;

            // IN DỮ LIỆU
            Serial.print("Mau thu: ");           Serial.print(sample_count);
            Serial.print(" | Tho (g): ");        Serial.print(raw_accZ, 4);
            Serial.print(" | Loc UKF (g): ");    Serial.print(filtered_accZ, 4);
            Serial.print(" | R thich nghi: ");   Serial.print(R_adaptive, 4);
            Serial.print(" | Thoi gian chay: "); Serial.print(execution_time_ms, 4); Serial.print(" ms");
            Serial.print(" | RAM chiem dung: "); Serial.print(ram_consumed_kb, 3);   Serial.println(" KB");
        }
    } else {
        static unsigned long last_notify = 0;
        if (millis() - last_notify > 2000) { 
            Serial.println("He thong dang TAM DUNG. Nhan giu nut bam tren D14 de KICH HOAT...");
            last_notify = millis();
        }
    }
}