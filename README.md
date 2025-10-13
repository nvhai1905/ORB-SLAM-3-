# Hướng dẫn build Orb-Slam3 với Ubuntu 24.04

## 1. Giới thiệu
ORB-SLAM là một hệ thống Visual SLAM (Simultaneous Localization and Mapping bằng camera) sử dụng điểm đặc trưng ORB (Oriented FAST + Rotated BRIEF).

Mục tiêu của SLAM là trong khi di chuyển, hệ thống vừa xác định vị trí (localization) vừa xây dựng bản đồ (mapping) của môi trường.

ORB-SLAM nổi bật vì là hệ thống mã nguồn mở, hiệu quả, dùng “feature-based” (dựa vào việc phát hiện và theo dõi các đặc trưng hình ảnh) và có khả năng loop closure (phát hiện khi nó quay lại vị trí đã đi qua) để giảm lỗi tích lũy.
Một hệ thống ORB-SLAM thường có các module chạy song song:

Tracking (Theo dõi vị trí
Local Mapping (Bản đồ cục bộ)
Loop Closing & Global Optimization
Relocalization (Phục hồi khi mất theo dõi)

ORB-SLAM3 mạnh mẽ hơn 2 vì có thể  có thêm IMU, hoạt động trong những môi trường phức tạp hơn, Có thể tạo nhiều bản đồ khi bị mất tracking, rồi tự động gộp lại khi quay về khu vực cũ.Chính xác hơn ORB-SLAM2 từ 2–5 lần trên nhiều bộ dữ liệu.
## 2.Hướng dẫn cách build 
### Yêu cầu hệ thống
```bash
Ubuntu 24.04 
CMake 3.10 hoặc cao hơn
OpenCV (phiên bản 4.x)
Eigen3
Pangolin
```
### Bước 1: Cài đặt Dependencies
``` bash
sudo apt update
sudo apt install -y build-essential cmake git libcurl4-openssl-dev libssl-dev libusb-dev pkg-config libgtk-3-dev libglew-dev
```
### Bước 2:  Cài đặt các thư viện cần thiết
#### Eigen3
``` bash
sudo apt install -y libeigen3-dev
```
#### OpenCV
``` bash
sudo apt install -y libopencv-dev python3-opencv
```
#### Boost
``` bash
sudo apt install -y libboost-all-dev
```
#### Python3 Development
``` bash
sudo apt install -y python3-dev python3-setuptools libpython3-dev
```
#### Pangolin
``` bash
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
## Lỗi có thể gặp
‼️ Python3 not found

Giải pháp: Chạy `sudo apt install -y python3-dev` trước khi cmake

### Bước 3: Clone ORB-SLAM3
``` bash
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3
```
### Bước 4: Sửa C++ Standard
#### Mở file CMakeLists.txt:
``` bash
nano CMakeLists.txt
Bạn có thể mở trực tiếp file qua đường dẫn sửa cho dễ ^^
```
#### Thêm sau dòng `project(ORB_SLAM3)`:
``` bash
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
``` 

### Bước 5: Build Thirdparty Libraries
#### Build DBoW2
``` bash
cd ~/ORB_SLAM3/Thirdparty/DBoW2
mkdir build
cd build
cmake ..
make -j4
```
#### Build g2o
``` bash
cd ~/ORB_SLAM3/Thirdparty/g2o
mkdir build
cd build
cmake ..
make -j4
```
#### Build Sophus
``` bash
cd ~/ORB_SLAM3/Thirdparty/Sophus
mkdir build
cd build
cmake -DCMAKE_CXX_FLAGS="-Wno-error=array-bounds -Wno-array-bounds" ..
make -j4
```
Lỗi có thể gặp: `array subscript '__m128_u[0]' is partly outside array bounds`

Giải pháp: Thêm flag `-Wno-error=array-bounds -Wno-array-bounds`
### Bước 6: Build ORB-SLAM3
``` bash
cd ~/ORB_SLAM3
mkdir build
cd build
cmake ..
make -j4
```
### Lỗi có thể gặp:

---

**Error 1:** `mnFullBAIdx++ - use of operand of type 'bool'`

- **Nguyên nhân:** Variable `mnFullBAIdx` khai báo là `bool` nhưng được increment như số  
- **Giải pháp:** Mở `include/LoopClosing.h` và thay `bool mnFullBAIdx;` thành `int mnFullBAIdx;`

---

**Error 2:** `use of an operand of type 'bool' in 'operator++'`

- **Nguyên nhân:** Giống lỗi 1  
- **Giải pháp:** Giống lỗi 1  

---

**Error 3:** `No rule to make target 'libDBoW2.so'`

- **Nguyên nhân:** Thirdparty libraries chưa được build  
- **Giải pháp:** Build Thirdparty libraries trước (Bước 5)

---

**Error 4:** `'decay_t' is not a member of 'std'` *(Pangolin)*

- **Nguyên nhân:** C++ standard quá cũ (C++11)  
- **Giải pháp:** Thêm `set(CMAKE_CXX_STANDARD 17)` vào `CMakeLists.txt`

---

**Error 5:** <span style="color:red">Python3 not found</span> *(Pangolin)*

- **Nguyên nhân:** Python3 development files chưa cài  
- **Giải pháp:** `sudo apt install -y python3-dev libpython3-dev`

---

**Error 6:** `Boost not found`

- **Nguyên nhân:** Boost library chưa cài  
- **Giải pháp:** `sudo apt install -y libboost-all-dev`

---

### Bước 7: Kiểm tra Build Thành Công

Kiểm tra xem các file đã được tạo:
``` bash
ls -lh ~/ORB_SLAM3/lib/libORB_SLAM3.so
ls -lh ~/ORB_SLAM3/Examples/Monocular/mono_tum
ls -lh ~/ORB_SLAM3/Vocabulary/ORBvoc.txt
```
Nếu thấy các file này, build thành công!
### Bước 8:  Chạy với Dataset
``` bash
cd ~
mkdir -p datasets/TUM
cd datasets/TUM
wget https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz
tar -xzf rgbd_dataset_freiburg1_xyz.tgz
```
Chạy ORB-SLAM3:
``` bash
cd ~/ORB_SLAM3
./Examples/Monocular/mono_tum ./Vocabulary/ORBvoc.txt ./Examples/Monocular/TUM1.yaml ~/datasets/TUM/rgbd_dataset_freiburg1_xyz
```
### Tóm tắt các lỗi chính và giải pháp

| **Lỗi** | **Nguyên nhân** | **Giải pháp** |
|:--|:--|:--|
| `mnFullBAIdx++` error | Variable là `bool` nhưng increment như số | Thay `bool` thành `int` trong `LoopClosing.h` |
| `'decay_t' not found` | C++11 không có `std::decay_t` | Thêm `CMAKE_CXX_STANDARD 17` |
| `Python3 not found` | Thiếu Python dev files | `apt install python3-dev libpython3-dev` |
| `Boost not found` | Thiếu Boost library | `apt install libboost-all-dev` |
| `array-bounds` warnings | GCC 13 cảnh báo về array bounds | Thêm `-Wno-error=array-bounds` flag |
| `No rule to make target libDBoW2.so` | Thirdparty chưa build | Build DBoW2, g2o, Sophus trước |

### Xác nhận build thành công
``` bash
# Kiểm tra library
ls -lh ~/ORB_SLAM3/lib/libORB_SLAM3.so

# Kiểm tra executables
ls -lh ~/ORB_SLAM3/Examples/Monocular/ | grep -E "^-rwx"

# Kiểm tra vocabulary
ls -lh ~/ORB_SLAM3/Vocabulary/ORBvoc.txt

# Kiểm tra headers
ls -la ~/ORB_SLAM3/include/ | wc -l
```
