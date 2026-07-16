import pandas as pd
import os

file_list = [
    'tinh3  - 2026-03-04 - Recording 1.csv',
    'rung 2  - 2026-03-04 - Recording 1.csv',
    'rung manh1  - 2026-03-04 - Recording 1.csv'
]

list_100_cuoi = []

for file_name in file_list:
    if os.path.exists(file_name):
        # Đọc file gốc, bỏ 3 dòng đầu của Excel
        df = pd.read_csv(file_name, skiprows=3, header=None)
        
        # Lấy 100 dòng cuối cùng của file này
        # (Lưu ý: lấy trước khi bị lệnh cắt 100 dòng cuối ở bước gộp file tác động)
        df_last_100 = df.tail(100)
        
        list_100_cuoi.append(df_last_100)
        print(f"Đã lấy 100 dòng cuối từ: {file_name}")

# Gộp lại thành file test
if list_100_cuoi:
    df_test_hop = pd.concat(list_100_cuoi, ignore_index=True)
    df_test_hop.columns = ['Timestamp', 'AccX', 'AccY', 'AccZ', 'GyroX', 'GyroY', 'GyroZ']
    df_test_hop.to_csv('data_test_300_mau.csv', index=False)
    print("\n--- HOÀN THÀNH ---")
    print("File 'data_test_300_mau.csv' đã sẵn sàng (gồm 300 mẫu cuối của 3 file).")