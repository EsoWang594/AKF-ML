#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "gru_model_weights.h"

// ====================================================================
// ĐỊNH NGHĨA CHÂN NÚT BẤM & KHỞI TẠO CẢM BIẾN
// ====================================================================
const int BUTTON_PIN = 14;      // Chân D14 kết nối nút bấm
bool is_running = false;        // Trạng thái hệ thống (true = đang chạy, false = dừng)
bool last_button_state = LOW;   // Lưu trạng thái trước đó của nút bấm
unsigned long last_debounce_time = 0;
const unsigned long DEBOUNCE_DELAY = 50; // Thời gian chống nhiễu phím (50ms)

Adafruit_MPU6050 mpu;           // Khai báo đối tượng cảm biến MPU6050

// ====================================================================
// THAM SỐ TOÁN HỌC UKF & BUFFER CỬA SỔ TRƯỢT
// ====================================================================
float x_est = 0.0;       
float P_est = 1.0;       
float Q = 0.01;          
float R_adaptive = 1.0;   

float alpha = 0.1, beta = 2.0, kappa = 0.0;
float lambda = alpha * alpha * (1.0 + kappa) - 1.0;
float Wm0 = lambda / (1.0 + lambda);
float Wc0 = (lambda / (1.0 + lambda)) + (1.0 - alpha * alpha + beta);
float Wmi = 1.0 / (2.0 * (1.0 + lambda));
float Wci = 1.0 / (2.0 * (1.0 + lambda));

const int VAR_WINDOW = 15;
float var_buffer[VAR_WINDOW];
int var_index = 0, var_count = 0;
float gru_input_history[GRU_WINDOW_SIZE]; 

uint32_t initial_free_ram = 0;
static int sample_count = 0;

// ====================================================================
// HÀM ĐỌC CẢM BIẾN THỰC TẾ (THAY THẾ HÀM GIẢ LẬP)
// ====================================================================
float read_raw_sensor() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Trả về giá trị gia tốc thực tế theo trục Z (Đơn vị: m/s^2)
    // Nếu đặt cảm biến nằm im phẳng trên bàn, giá trị sẽ xấp xỉ ~9.81 hoặc -9.81 m/s^2 tùy chiều lật
    return a.acceleration.z; 
}

// Hàm tính phương sai
float update_moving_variance(float new_sample) {
    var_buffer[var_index] = new_sample;
    var_index = (var_index + 1) % VAR_WINDOW;
    if (var_count < VAR_WINDOW) var_count++;
    float sum = 0.0;
    for (int i = 0; i < var_count; i++) sum += var_buffer[i];
    float mean = sum / var_count;
    float sum_sq_diff = 0.0;
    for (int i = 0; i < var_count; i++) {
        float diff = var_buffer[i] - mean;
        sum_sq_diff += diff * diff;
    }
    return sum_sq_diff / var_count;
}

void push_to_gru_history(float new_variance) {
    for (int i = 0; i < GRU_WINDOW_SIZE - 1; i++) {
        gru_input_history[i] = gru_input_history[i + 1];
    }
    gru_input_history[GRU_WINDOW_SIZE - 1] = new_variance;
}

// Hàm dự báo GRU C++ thuần
float predict_gru(float* input_sequence) {
    float h[GRU_UNITS] = {0.0}; 
    for (int t = 0; t < GRU_WINDOW_SIZE; t++) {
        float xt = input_sequence[t];
        float z[GRU_UNITS], r[GRU_UNITS], h_tilde[GRU_UNITS];
        for (int i = 0; i < GRU_UNITS; i++) {
            float gate_z_net = W_gru[0][i] * xt + b_gru[0][i];
            for (int j = 0; j < GRU_UNITS; j++) gate_z_net += U_gru[j][i] * h[j] + b_gru[1][i];
            z[i] = 1.0 / (1.0 + exp(-gate_z_net)); 

            float gate_r_net = W_gru[0][i + GRU_UNITS] * xt + b_gru[0][i + GRU_UNITS];
            for (int j = 0; j < GRU_UNITS; j++) gate_r_net += U_gru[j][i + GRU_UNITS] * h[j] + b_gru[1][i + GRU_UNITS];
            r[i] = 1.0 / (1.0 + exp(-gate_r_net)); 

            float gate_h_net = W_gru[0][i + 2 * GRU_UNITS] * xt + b_gru[0][i + 2 * GRU_UNITS];
            for (int j = 0; j < GRU_UNITS; j++) gate_h_net += U_gru[j][i + 2 * GRU_UNITS] * (r[j] * h[j]) + b_gru[1][i + 2 * GRU_UNITS];
            h_tilde[i] = tanh(gate_h_net); 

            h[i] = (1.0 - z[i]) * h_tilde[i] + z[i] * h[i];
        }
    }
    float dense1_out[DENSE1_UNITS];
    for (int i = 0; i < DENSE1_UNITS; i++) {
        float sum = b_dense1[i];
        for (int j = 0; j < GRU_UNITS; j++) sum += h[j] * W_dense1[j][i];
        dense1_out[i] = (sum > 0.0) ? sum : 0.0; 
    }
    float final_output = b_dense2[0];
    for (int j = 0; j < DENSE1_UNITS; j++) final_output += dense1_out[j] * W_dense2[j][0];
    return final_output;
}

// Thuật toán lõi thích nghi GRU-UKF
float update_Adaptive_UKF_GRU(float z_measured) {
    float current_variance = update_moving_variance(z_measured);
    push_to_gru_history(current_variance);

    if (sample_count >= GRU_WINDOW_SIZE) {
        R_adaptive = predict_gru(gru_input_history);
    } else {
        R_adaptive = 1.0; 
    }
    if (R_adaptive < 0.01) R_adaptive = 0.01; 

    float x_predict = x_est; 
    float P_predict = P_est + Q;
    float sigma_margin = sqrt((1.0 + lambda) * P_predict);
    float sigma_pts[3] = {x_predict, x_predict + sigma_margin, x_predict - sigma_margin};

    float z_mean = Wm0 * sigma_pts[0] + Wmi * sigma_pts[1] + Wmi * sigma_pts[2];
    float Pyy = Wc0 * (sigma_pts[0] - z_mean) * (sigma_pts[0] - z_mean) +
                Wci * (sigma_pts[1] - z_mean) * (sigma_pts[1] - z_mean) +
                Wci * (sigma_pts[2] - z_mean) * (sigma_pts[2] - z_mean) + R_adaptive;

    float Pxy = Wc0 * (sigma_pts[0] - x_predict) * (sigma_pts[0] - z_mean) +
                Wci * (sigma_pts[1] - x_predict) * (sigma_pts[1] - z_mean) +
                Wci * (sigma_pts[2] - x_predict) * (sigma_pts[2] - z_mean);

    float K = Pxy / Pyy;
    x_est = x_predict + K * (z_measured - z_mean);
    P_est = P_predict - K * Pyy * K;

    return x_est;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    pinMode(BUTTON_PIN, INPUT); 
    
    // Khởi tạo giao tiếp I2C và cảm biến MPU6050 thật
    Wire.begin(); // Mặc định SDA=SDA_PIN, SCL=SCL_PIN của board ESP32
    if (!mpu.begin()) {
        Serial.println("[-] KHÔNG tìm thấy chip cảm biến MPU6050. Hãy kiểm tra dây nối!");
        while (1) { delay(10); }
    }
    Serial.println("[+] MPU6050 đã kết nối thành công.");
    
    // Cấu hình dải đo gia tốc (tùy chọn)
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    
    initial_free_ram = ESP.getFreeHeap();
    Serial.println("=================================================");
    Serial.println("HE THONG SAN SANG. NHAN NUT TAI D14 DE CHAY/DUNG.");
    Serial.println("=================================================");
}

void loop() {
    // ---- KHỐI KIỂM TRA VÀ CHỐNG NHIỄU NÚT BẤM (DEBOUNCE) ----
    int reading = digitalRead(BUTTON_PIN);
    if (reading != last_button_state) {
        last_debounce_time = millis();
    }

    if ((millis() - last_debounce_time) > DEBOUNCE_DELAY) {
        static bool button_pressed = false;
        if (reading == HIGH && !button_pressed) {
            is_running = !is_running;
            button_pressed = true;
            
            if (is_running) {
                Serial.println("\n>>> [START] He thong bat dau chay...");
            } else {
                Serial.println("\n>>> [STOP] He thong da tam dung.");
            }
        } else if (reading == LOW) {
            button_pressed = false;
        }
    }
    last_button_state = reading;

    // ---- KHỐI XỬ LÝ CHÍNH ----
    // ---- KHỐI XỬ LÝ CHÍNH ----
    if (is_running) {
        float raw_accZ = read_raw_sensor();
        
        unsigned long start_time = micros();   
        float filtered_accZ = update_Adaptive_UKF_GRU(raw_accZ); 
        unsigned long end_time = micros();    

        sample_count++;

        // --- ĐỊNH DẠNG CHO SERIAL PLOTTER ---
        // Chỉ in tên biến và giá trị cách nhau bằng dấu phẩy, không in chú thích
        Serial.print("Raw:");
        Serial.print(raw_accZ, 3);
        Serial.print(",");
        Serial.print("Filtered:");
        Serial.print(filtered_accZ, 3);
        Serial.print(",");
        Serial.print("R_Adaptive:");
        Serial.println(R_adaptive, 4); // Dòng cuối dùng println để xuống hàng

        delay(10); 
    }
}