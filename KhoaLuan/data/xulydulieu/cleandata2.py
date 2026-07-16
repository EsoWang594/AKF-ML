import pandas as pd
import numpy as np

# 1. Đọc file gốc
df = pd.read_csv('dataset_for_ml.csv')

# 2. Hàm dán nhãn V3: Chiến thuật "Siêu mượt"
def optimize_label_v3(v):
    # Vùng Tĩnh tuyệt đối: Giữ R nhỏ nhất để bám sát trọng lực lý thuyết
    if v < 0.001: 
        return 0.01
    
    # Vùng Nhiễu nền: Tăng nhẹ R để làm phẳng các dao động li ti
    elif v < 0.01:
        return 0.1
    
    # Vùng Rung vừa: Đây là vùng quan trọng nhất để thắng R=0.5
    # Chúng ta giữ R quanh dải 0.3 - 0.5
    elif v < 0.2:
        return 0.4
    
    # Vùng Rung mạnh: Chỉ tăng lên tối đa 1.0 (Thay vì 1.5 hay 5.0)
    # Mức 1.0 đủ để lọc nhiễu nhưng cực kỳ an toàn cho RMSE vì ít gây trễ pha
    else:
        return 1.0

# 3. Tạo cột nhãn mới (Sửa lỗi gọi hàm từ code cũ của bạn)
df['Label_R'] = df['Variance'].apply(optimize_label_v3)

# 4. Kiểm tra sự phân bổ nhãn V3
print("--- PHÂN BỔ NHÃN MỚI V3 ---")
print(df['Label_R'].value_counts().sort_index())

# 5. Lưu thành file mới riêng biệt
output_file = 'dataset_for_ml_v3.csv'
df.to_csv(output_file, index=False)

print(f"\n--- KẾT QUẢ ---")
print(f"Đã tạo xong file: {output_file}")
print(f"Lời khuyên: Hãy dùng file này để train mô hình Đa thức (Polynomial), "
      f"khả năng cao chỉ số RMSE sẽ cải thiện vượt bậc so với các bản trước.")