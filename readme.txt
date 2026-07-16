# AKF-ML: Adaptive Kalman Filter Optimized by Machine Learning for IMU Sensors

[![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-ESP32-orange.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Language](https://img.shields.io/badge/Language-C%2B%2B%20%2F%20Python-blue.svg)](#)

Dự án này là mã nguồn chính thức của đề tài Khóa luận tốt nghiệp Cử nhân ngành Công nghệ Vật lý Điện tử và Tin học: **"Ứng dụng Machine Learning tối ưu hóa bộ lọc Kalman cho cảm biến IMU"**. 

Hệ thống triển khai thuật toán **Bộ lọc Kalman thích nghi (Adaptive Kalman Filter - AKF)** trên vi điều khiển **ESP32**, trong đó ma trận hiệp phương sai sai số đo $R$ được tối ưu hóa động (dynamic adaptation) theo thời gian thực dựa trên các mô hình **Machine Learning / Deep Learning** nhằm loại bỏ nhiễu rung động cho cảm biến IMU MPU6050 dưới các điều kiện môi trường khác nhau.

---

## 📌 Sơ đồ kiến trúc hệ thống

```text
┌───────────┐      ┌─────────────────────────┐      ┌───────────────────────────────┐
│  MPU6050  │ ───> │    ESP32 MCU (I2C)      │ ───> │ Feature Extraction            │
│  (Raw)    │      │  Acc & Gyro Acquisition │      │ (Sliding Window N=50 & Z-score)│
└───────────┘      └─────────────────────────┘      └───────────────────────────────┘
                                                                    │
┌─────────────────────────┐      ┌─────────────────────────┐        │
│ Optimized Attitude      │ <─── │ Adaptive Kalman Filter  │ <──────┘
│ Output (Roll & Pitch)   │      │ (Dynamic R Update)      │
└─────────────────────────┘      └─────────────────────────┘
🚀 Các tính năng chính
Thu thập dữ liệu thời gian thực: Thu nhận dữ liệu gia tốc (Accelerometer) và vận tốc góc (Gyroscope) từ cảm biến MPU6050 qua giao tiếp I2C trên ESP32 với tần số lấy mẫu cao.
Kỹ thuật trích lọc đặc trưng (Feature Engineering):
	Áp dụng phương pháp cửa sổ trượt (sliding window) kích thước N = 50 để tính toán phương sai dữ liệu thô ngay trên chip (on-the-fly).
	Chuẩn hóa dữ liệu bằng phương pháp Z-score phù hợp với tài nguyên nhúng giới hạn.
Huấn luyện mô hình đa dạng (Python): So sánh hiệu năng của nhiều kiến trúc mô hình từ cơ bản đến phức tạp:
	Hồi quy tuyến tính & Hồi quy đa thức (Linear & Polynomial Regression)
	Mạng nơ-ron nhân tạo (ANN)
	Cây quyết định tăng cường (XGBoost)
	Mạng nơ-ron hồi quy cổng lặp (GRU)
Triển khai nhúng tối ưu (Embedded C++): Chuyển đổi và nhúng trực tiếp các mô hình ML đã huấn luyện từ Python sang mã nguồn C++ chạy trực tiếp trên vi điều khiển ESP32 với độ trễ tính toán cực thấp.
Bộ lọc Kalman thích nghi: Điều chỉnh động hệ số nhiễu đo R dựa trên dự báo trạng thái môi trường rung lắc từ mô hình ML, giúp tối ưu hóa góc đầu ra cả về độ nhạy (độ trễ thấp) và độ ổn định (triệt tiêu nhiễu tốt).

📂 Cấu trúc thư mục dự án
Dự án được tổ chức phân tầng rõ ràng giữa phần thiết kế phần cứng, huấn luyện máy học (Python) và mã nguồn nhúng (C++):
AKF-ML/
├── code/
│   ├── CODE chạy thực nghiệm/        # Mã nguồn nhúng (Arduino C++) nạp trực tiếp vào ESP32
│   │   ├── ANN/                     # Lọc Kalman thích nghi tích hợp mạng nơ-ron nhân tạo
│   │   │   └── ANN.ino
│   │   ├── GRU/                     # Lọc Kalman thích nghi tích hợp mạng GRU (Deep Learning)
│   │   │   └── GRU/
│   │   │       ├── GRU.ino
│   │   │       └── gru_model_weights.h # Trọng số mạng GRU được chuyển đổi sang mảng tĩnh C++
│   │   ├── HQDT/                    # Lọc Kalman thích nghi bằng Hồi quy đa thức (Polynomial)
│   │   │   └── HQDT.ino
│   │   ├── HQTT/                    # Lọc Kalman thích nghi bằng Hồi quy tuyến tính (Linear)
│   │   │   └── HQTT.ino
│   │   ├── RCODINH/                 # Bộ lọc Kalman truyền thống với R cố định (Mốc đối chứng)
│   │   │   └── RCODINH.ino
│   │   └── XGBOOT/                  # Lọc Kalman thích nghi tích hợp thuật toán XGBoost
│   │       └── XGB/
│   │           ├── XGB.ino
│   │           └── xgb_model_code.h # Cấu trúc cây quyết định xuất từ Python
│   │
│   ├── Code Huấn luyện máy học/      # Nghiên cứu và xây dựng mô hình (Python)
│   │   └── huanluyenmayhoc&test.ipynb # Jupyter Notebook xử lý đặc trưng, train & test mô hình
│   │
│   └── code test/                   # Các chương trình C++ (.cpp) giả lập thuật toán trên máy tính
│       ├── test R co dinh.cpp
│       ├── testANN.cpp
│       ├── testhqdt.cpp
│       └── testhqtt.cpp
│
└── data/                            # Tập dữ liệu thực nghiệm thu thập từ cảm biến
    ├── tinh1.xlsx                   # Dữ liệu tĩnh thô ban đầu
    ├── tinh3...csv / rung 2...csv   # Dữ liệu góc và gia tốc thô trong các kịch bản tĩnh và rung lắc
    │
    ├── loc/                         # Dữ liệu đầu ra sau khi chạy qua các bộ lọc trên ESP32
    │   ├── ann/                     # Góc ra của bộ lọc AKF-ANN (tĩnh, rung nhẹ, rung mạnh)
    │   ├── hqdt/                    # Góc ra của bộ lọc AKF-HQDT
    │   ├── hqtt/                    # Góc ra của bộ lọc AKF-HQTT
    │   └── R co dinh/               # Góc ra của bộ lọc Kalman tiêu chuẩn R cố định
    │
    └── xulydulieu/                  # Các mã nguồn Python làm sạch và đóng gói dữ liệu
        ├── 300sample.py             # Trích xuất phân đoạn dữ liệu 300 mẫu để phân tích chuyên sâu
        ├── cleandata.ipy / cleandata2.py # Làm sạch dữ liệu, xử lý nhiễu ngoại lai
        ├── dataset_for_ml.csv       # Các phiên bản tập dữ liệu gán nhãn cuối cùng
        ├── dataset_for_ml_v2.csv    # phục vụ cho quá trình huấn luyện Machine Learning
        ├── dataset_for_ml_v3.csv
        ├── data_test_300_mau.csv    # Tập test 300 mẫu giả lập thuật toán
        ├── data_tong_hop.csv        # Tổng hợp dữ liệu từ các kịch bản thực nghiệm
        └── data_clean/              # Thư mục lưu trữ các file dữ liệu sạch sau xử lý
            ├── rung 2 - ...csv
            ├── rung manh1 - ...csv
            └── tinh3 - ...csvAKF-ML/
├── code/
│   ├── CODE chạy thực nghiệm/        # Mã nguồn nhúng (Arduino C++) nạp trực tiếp vào ESP32
│   │   ├── ANN/                     # Lọc Kalman thích nghi tích hợp mạng nơ-ron nhân tạo
│   │   │   └── ANN.ino
│   │   ├── GRU/                     # Lọc Kalman thích nghi tích hợp mạng GRU (Deep Learning)
│   │   │   └── GRU/
│   │   │       ├── GRU.ino
│   │   │       └── gru_model_weights.h # Trọng số mạng GRU được chuyển đổi sang mảng tĩnh C++
│   │   ├── HQDT/                    # Lọc Kalman thích nghi bằng Hồi quy đa thức (Polynomial)
│   │   │   └── HQDT.ino
│   │   ├── HQTT/                    # Lọc Kalman thích nghi bằng Hồi quy tuyến tính (Linear)
│   │   │   └── HQTT.ino
│   │   ├── RCODINH/                 # Bộ lọc Kalman truyền thống với R cố định (Mốc đối chứng)
│   │   │   └── RCODINH.ino
│   │   └── XGBOOT/                  # Lọc Kalman thích nghi tích hợp thuật toán XGBoost
│   │       └── XGB/
│   │           ├── XGB.ino
│   │           └── xgb_model_code.h # Cấu trúc cây quyết định xuất từ Python
│   │
│   ├── Code Huấn luyện máy học/      # Nghiên cứu và xây dựng mô hình (Python)
│   │   └── huanluyenmayhoc&test.ipynb # Jupyter Notebook xử lý đặc trưng, train & test mô hình
│   │
│   └── code test/                   # Các chương trình C++ (.cpp) giả lập thuật toán trên máy tính
│       ├── test R co dinh.cpp
│       ├── testANN.cpp
│       ├── testhqdt.cpp
│       └── testhqtt.cpp
│
└── data/                            # Tập dữ liệu thực nghiệm thu thập từ cảm biến
    ├── tinh1.xlsx                   # Dữ liệu tĩnh thô ban đầu
    ├── tinh3...csv / rung 2...csv   # Dữ liệu góc và gia tốc thô trong các kịch bản tĩnh và rung lắc
    │
    ├── loc/                         # Dữ liệu đầu ra sau khi chạy qua các bộ lọc trên ESP32
    │   ├── ann/                     # Góc ra của bộ lọc AKF-ANN (tĩnh, rung nhẹ, rung mạnh)
    │   ├── hqdt/                    # Góc ra của bộ lọc AKF-HQDT
    │   ├── hqtt/                    # Góc ra của bộ lọc AKF-HQTT
    │   └── R co dinh/               # Góc ra của bộ lọc Kalman tiêu chuẩn R cố định
    │
    └── xulydulieu/                  # Các mã nguồn Python làm sạch và đóng gói dữ liệu
        ├── 300sample.py             # Trích xuất phân đoạn dữ liệu 300 mẫu để phân tích chuyên sâu
        ├── cleandata.ipy / cleandata2.py # Làm sạch dữ liệu, xử lý nhiễu ngoại lai
        ├── dataset_for_ml.csv       # Các phiên bản tập dữ liệu gán nhãn cuối cùng
        ├── dataset_for_ml_v2.csv    # phục vụ cho quá trình huấn luyện Machine Learning
        ├── dataset_for_ml_v3.csv
        ├── data_test_300_mau.csv    # Tập test 300 mẫu giả lập thuật toán
        ├── data_tong_hop.csv        # Tổng hợp dữ liệu từ các kịch bản thực nghiệm
        └── data_clean/              # Thư mục lưu trữ các file dữ liệu sạch sau xử lý
            ├── rung 2 - ...csv
            ├── rung manh1 - ...csv
            └── tinh3 - ...csv

🛠️ Hướng dẫn cài đặt và sử dụng
1. Phần huấn luyện Machine Learning (Python)
Di chuyển vào thư mục xử lý dữ liệu để chuẩn bị dữ liệu sạch:
cd data/xulydulieu
python cleandata2.py
Mở và chạy file Notebook huấn luyện để thử nghiệm và xuất trọng số mô hình:


jupyter notebook "code/Code Huấn luyện máy học/huanluyenmayhoc&test.ipynb"

Sau khi huấn luyện, các mô hình/trọng số sẽ được chuyển thành các file tiêu đề mảng tĩnh C++ (ví dụ: gru_model_weights.h, xgb_model_code.h) để nạp vào ESP32.

2. Triển khai nạp Firmware ESP32 (Arduino C++)
Yêu cầu: Đã cài đặt Arduino IDE và cấu hình bo mạch ESP32.

Bước 1: Kết nối phần cứng giữa ESP32 và cảm biến IMU MPU6050:
Bước 2: Mở chương trình tương ứng với mô hình bạn muốn thực nghiệm trong thư mục code/CODE chạy thực nghiệm/ (Ví dụ: code/CODE chạy thực nghiệm/ANN/ANN.ino).

Bước 3: Biên dịch (Compile) và nạp (Upload) code lên bo mạch ESP32 của bạn.

Bước 4: Mở Serial Plotter hoặc Serial Monitor với Baudrate tương ứng cấu hình trong code để quan sát tín hiệu góc đầu ra đã được tối ưu hóa động.
📊 Kết quả thực nghiệm chính
Mô hình AKF-ML giúp cải thiện vượt trội khả năng triệt tiêu nhiễu tần số cao của cảm biến IMU khi gặp rung lắc động so với bộ lọc Kalman truyền thống sử dụng hệ số $R$ cố định:
Thời gian thực thi (Computational Latency): * Mô hình Hồi quy tuyến tính (HQTT): < 0.1 ms trên ESP32.

Mô hình Mạng nơ-ron nhân tạo (ANN): ~ 1.2 ms trên ESP32.

Độ chính xác (Accuracy Improvement): Giảm sai số bình phương trung bình Root Mean Squared Error (RMSE) của góc đầu ra lên tới 30% - 40% trong các kịch bản thực nghiệm rung lắc từ vừa đến mạnh. 


🎓 Thông tin đề tài
Sinh viên thực hiện: Phạm Đăng Quang

Giảng viên hướng dẫn: GVC. ThS. Hứa Thị Hoàng Yến

Đơn vị công tác: Khoa Vật lý – Vật lý Kỹ thuật, Trường Đại học Khoa học Tự nhiên, ĐHQG-HCM.

Chuyên ngành: Công nghệ Vật lý Điện tử và Tin học.

Năm hoàn thành: 2026