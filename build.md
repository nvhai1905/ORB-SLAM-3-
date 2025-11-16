# Hướng dẫn Build ORB-SLAM3 trên Ubuntu 20.04

## Giới thiệu
ORB-SLAM là một hệ thống Visual SLAM (Simultaneous Localization and Mapping bằng camera) sử dụng điểm đặc trưng ORB (Oriented FAST + Rotated BRIEF).

Mục tiêu của SLAM là trong khi di chuyển, hệ thống vừa xác định vị trí (localization) vừa xây dựng bản đồ (mapping) của môi trường.

ORB-SLAM nổi bật vì là hệ thống mã nguồn mở, hiệu quả, dùng “feature-based” (dựa vào việc phát hiện và theo dõi các đặc trưng hình ảnh) và có khả năng loop closure (phát hiện khi nó quay lại vị trí đã đi qua) để giảm lỗi tích lũy. Một hệ thống ORB-SLAM thường có các module chạy song song:

Tracking (Theo dõi vị trí Local Mapping (Bản đồ cục bộ) Loop Closing & Global Optimization Relocalization (Phục hồi khi mất theo dõi)

ORB-SLAM3 mạnh mẽ hơn 2 vì có thể có thêm IMU, hoạt động trong những môi trường phức tạp hơn, Có thể tạo nhiều bản đồ khi bị mất tracking, rồi tự động gộp lại khi quay về khu vực cũ.Chính xác hơn ORB-SLAM2 từ 2–5 lần trên nhiều bộ dữ liệu.

## Tổng quan
Hướng dẫn này giúp bạn cài đặt ORB-SLAM3 trên Ubuntu 20.04 từ đầu, bao gồm các lỗi thường gặp và cách khắc phục.

**Thời gian:** Khoảng 1-2 giờ  
**Yêu cầu:** Ubuntu 20.04, kết nối internet ổn định

---

## Bước 1: Cài đặt Dependencies

```bash
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update

sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev
sudo apt-get install libglew-dev libboost-all-dev libssl-dev
sudo apt install libeigen3-dev
```

---

## Bước 2: Cài đặt OpenCV 3.2.0

### ⚠️ Tại sao phải dùng OpenCV 3.2.0?

Ubuntu 20.04 có sẵn OpenCV 4.2, nhưng:
- ORB-SLAM3 được phát triển và test với OpenCV 3.2.0
- OpenCV 4.x có thay đổi API so với 3.x (có thể gây lỗi)
- Cài OpenCV 3.2.0 vào `/usr/local` không conflict với OpenCV 4.2 của hệ thống

### Clone và checkout phiên bản 3.2.0

```bash
cd ~
mkdir Dev && cd Dev
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 3.2.0
```

### Sửa file quan trọng cho compatibility

```bash
gedit ./modules/videoio/src/cap_ffmpeg_impl.hpp
```

Thêm các dòng sau vào **đầu file**:

```cpp
#define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
#define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER
#define AVFMT_RAWPICTURE 0x0020
```

Lưu và đóng file.

### Build OpenCV (tắt các module không cần thiết)

```bash
mkdir build
cd build

cmake -D CMAKE_BUILD_TYPE=Release \
      -D WITH_CUDA=OFF \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D BUILD_opencv_viz=OFF \
      -D BUILD_opencv_python2=OFF \
      -D BUILD_opencv_python3=OFF \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      ..

make -j3
sudo make install
```

### Kiểm tra cài đặt

```bash
pkg-config --modversion opencv
# Kết quả phải là: 3.2.0
```

---

## Bước 3: Cài đặt Pangolin

```bash
cd ~/Dev
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin 
git checkout 86eb4975fc4fc8b5d92148c2e370045ae9bf9f5d
mkdir build 
cd build 
cmake .. -D CMAKE_BUILD_TYPE=Release 
make -j3
sudo make install
```

---

## Bước 4: Build ORB-SLAM3

### Clone repository

```bash
cd ~/Dev
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git 
cd ORB_SLAM3
git checkout ef9784101fbd28506b52f233315541ef8ba7af57
```

### Sửa lỗi compile quan trọng

```bash
gedit ./include/LoopClosing.h
```

Tại **dòng 51**, thay đổi từ:

```cpp
Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
```

Thành:

```cpp
Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3> > > KeyFrameAndPose;
```

Lưu và đóng file.

### Build ORB-SLAM3

```bash
./build.sh
```

⚠️ **Lưu ý:** Nếu gặp lỗi compile, thử chạy lại `./build.sh` 2-3 lần (như tác giả đã test).

### Kiểm tra build thành công

```bash
cd ~/Dev/ORB_SLAM3

echo "=== Checking libraries ==="
ls lib/*.so 2>/dev/null && echo "✓ Main library OK" || echo "✗ Main library MISSING"
ls Thirdparty/DBoW2/lib/*.so 2>/dev/null && echo "✓ DBoW2 OK" || echo "✗ DBoW2 MISSING"
ls Thirdparty/g2o/lib/*.so 2>/dev/null && echo "✓ g2o OK" || echo "✗ g2o MISSING"

echo ""
echo "=== Checking executables ==="
ls Examples/Stereo/stereo_euroc 2>/dev/null && echo "✓ Stereo executable OK" || echo "✗ Stereo executable MISSING"
ls Examples/Monocular/mono_euroc 2>/dev/null && echo "✓ Mono executable OK" || echo "✗ Mono executable MISSING"
ls Examples/Monocular-Inertial/mono_inertial_euroc 2>/dev/null && echo "✓ Mono-Inertial executable OK" || echo "✗ Mono-Inertial executable MISSING"
ls Examples/Stereo-Inertial/stereo_inertial_euroc 2>/dev/null && echo "✓ Stereo-Inertial executable OK" || echo "✗ Stereo-Inertial executable MISSING"
```

Nếu tất cả đều OK, bạn đã build thành công! 🎉

---

## Bước 5: Test với Dataset (Tùy chọn)

### Tải dataset EuRoC

```bash
cd ~
mkdir -p Datasets/EuRoc
cd Datasets/EuRoc/
wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
mkdir MH01
unzip MH_01_easy.zip -d MH01/
```

### Chạy test Stereo

```bash
cd ~/Dev/ORB_SLAM3
./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo
```

### Chạy các mode khác

```bash
# Monocular
./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono

# Monocular + Inertial
./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi

# Stereo + Inertial
./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereoi
```

---

## Các Lỗi Thường Gặp và Cách Khắc Phục

### ❌ Lỗi 1: `fatal error: stdlib.h: No such file or directory`

**Mô tả lỗi:**
```
/usr/include/c++/9/cstdlib:75:15: fatal error: stdlib.h: No such file or directory
   75 | #include_next <stdlib.h>
```

**Nguyên nhân:** Module `viz` của OpenCV 3.2.0 không tương thích với GCC 9 trên Ubuntu 20.04. ORB-SLAM3 không cần module này.

**Giải pháp:** Tắt module viz khi build OpenCV

```bash
cd ~/Dev/opencv
rm -rf build
mkdir build
cd build

cmake -D CMAKE_BUILD_TYPE=Release \
      -D WITH_CUDA=OFF \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D BUILD_opencv_viz=OFF \
      ..

make -j3
sudo make install
```

---

### ❌ Lỗi 2: `cast between incompatible function types` (Python bindings)

**Mô tả lỗi:**
```
warning: cast between incompatible function types from 'PyObject* (*)(PyObject*, PyObject*, PyObject*)' 
to 'PyCFunction' [-Wcast-function-type]
make[2]: *** [modules/python3/CMakeFiles/opencv_python3.dir/build.make:180: ...] Error 1
```

**Nguyên nhân:** Python bindings của OpenCV 3.2.0 không tương thích hoàn toàn với Python 3 trên Ubuntu 20.04.

**Giải pháp:** Tắt Python bindings (ORB-SLAM3 không cần)

```bash
cd ~/Dev/opencv
rm -rf build
mkdir build
cd build

cmake -D CMAKE_BUILD_TYPE=Release \
      -D WITH_CUDA=OFF \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D BUILD_opencv_viz=OFF \
      -D BUILD_opencv_python2=OFF \
      -D BUILD_opencv_python3=OFF \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      ..

make -j3
sudo make install
```

**Lợi ích:** Build nhanh hơn nhiều và tránh lỗi không cần thiết.

---

### ❌ Lỗi 3: Lỗi compile ORB-SLAM3 liên quan đến Eigen

**Mô tả lỗi:**
```
error: 'const class std::pair<const KeyFrame*, g2o::Sim3>' has no member named 'first'
```

**Nguyên nhân:** Vấn đề với const correctness trong template Eigen.

**Giải pháp:** Đã được fix ở Bước 4 - sửa file `include/LoopClosing.h` dòng 51

Thay:
```cpp
Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
```

Thành:
```cpp
Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3> > > KeyFrameAndPose;
```

---

### ❌ Lỗi 4: `AV_CODEC_FLAG_GLOBAL_HEADER` not declared

**Mô tả lỗi:**
```
error: 'AV_CODEC_FLAG_GLOBAL_HEADER' was not declared in this scope
error: 'CODEC_FLAG_GLOBAL_HEADER' was not declared in this scope
```

**Nguyên nhân:** FFmpeg mới có thay đổi API, OpenCV 3.2.0 cần các define này.

**Giải pháp:** Đã được fix ở Bước 2 - thêm các define vào file `cap_ffmpeg_impl.hpp`

---

### ❌ Lỗi 5: `libjasper-dev` không tìm thấy

**Mô tả lỗi:**
```
E: Package 'libjasper-dev' has no installation candidate
```

**Nguyên nhân:** Ubuntu 20.04 đã loại bỏ libjasper-dev khỏi repository chính.

**Giải pháp:** Đã được fix ở Bước 1 - thêm repository xenial-security

```bash
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt-get install libjasper-dev
```

---

### ❌ Lỗi 6: Build thất bại lần đầu

**Mô tả:** `./build.sh` báo lỗi lần đầu tiên chạy.

**Nguyên nhân:** Dependencies (DBoW2, g2o) đôi khi build không đúng thứ tự.

**Giải pháp:** Chạy lại `./build.sh` 2-3 lần như hướng dẫn

```bash
cd ~/Dev/ORB_SLAM3
./build.sh
# Nếu lỗi, chạy lại
./build.sh
# Lần thứ 3 nếu cần
./build.sh
```

---

### ❌ Lỗi 7: Pangolin không tìm thấy OpenGL

**Mô tả lỗi:**
```
Could NOT find OpenGL
```

**Giải pháp:** Cài đặt thư viện OpenGL

```bash
sudo apt-get install libgl1-mesa-dev libglu1-mesa-dev
```

---

### ❌ Lỗi 8: Không tìm thấy `pkg-config`

**Giải pháp:**

```bash
sudo apt-get install pkg-config
```

---

## Thư Viện Bổ Sung Có Thể Cần

Nếu gặp lỗi khác, thử cài thêm các thư viện sau:

```bash
# Thư viện C/C++ cơ bản
sudo apt-get install build-essential
sudo apt-get install libc6-dev
sudo apt-get install linux-libc-dev

# OpenGL và visualization
sudo apt-get install libgl1-mesa-dev libglu1-mesa-dev
sudo apt-get install freeglut3-dev

# Video và image codecs
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install libv4l-dev libxvidcore-dev libx264-dev

# Thư viện nén
sudo apt-get install libpng-dev libjpeg-dev libtiff-dev

# Parallel processing
sudo apt-get install libtbb2 libtbb-dev

# Linear algebra
sudo apt-get install libeigen3-dev liblapack-dev libblas-dev

# Boost libraries
sudo apt-get install libboost-all-dev
```

---

## Cấu Trúc Thư Mục Sau Khi Cài Đặt

```
~/Dev/
├── opencv/              # OpenCV 3.2.0 source
│   └── build/
├── Pangolin/            # Pangolin source
│   └── build/
└── ORB_SLAM3/           # ORB-SLAM3
    ├── lib/
    │   └── libORB_SLAM3.so
    ├── Thirdparty/
    │   ├── DBoW2/lib/libDBoW2.so
    │   └── g2o/lib/libg2o.so
    ├── Examples/
    │   ├── Monocular/
    │   ├── Stereo/
    │   ├── Monocular-Inertial/
    │   └── Stereo-Inertial/
    └── Vocabulary/
        └── ORBvoc.txt

~/Datasets/              # Datasets (tùy chọn)
└── EuRoc/
    └── MH01/
```

---

## Sử Dụng ORB-SLAM3 trong Project Riêng

### Include paths

```cmake
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ~/Dev/ORB_SLAM3/include
    ~/Dev/ORB_SLAM3/Thirdparty/Sophus
)
```

### Link libraries

```cmake
target_link_libraries(${PROJECT_NAME}
    ~/Dev/ORB_SLAM3/lib/libORB_SLAM3.so
    ~/Dev/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
    ~/Dev/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
)
```

### Vocabulary file

Luôn cần file vocabulary:
```
~/Dev/ORB_SLAM3/Vocabulary/ORBvoc.txt
```

---

## Tips Quan Trọng

1. **Luôn dùng đúng commit version** như hướng dẫn để tránh lỗi
2. **Không bỏ qua bước sửa file** (cap_ffmpeg_impl.hpp và LoopClosing.h)
3. **Xóa thư mục build cũ** khi gặp lỗi: `rm -rf build && mkdir build`
4. **Kiểm tra OpenCV version** sau khi cài: `pkg-config --modversion opencv`
5. **Sử dụng `-j3` hoặc `-j4`** khi make để tránh quá tải RAM

---

## Tài Liệu Tham Khảo

- [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [OpenCV 3.2.0](https://github.com/opencv/opencv/tree/3.2.0)
- [Pangolin](https://github.com/stevenlovegrove/Pangolin)
- [EuRoC Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

---

**Tác giả:** Dựa trên hướng dẫn của Mauhing Yip  
**Cập nhật:** November 2024  
**Hệ thống test:** Ubuntu 20.04 LTS, GCC 9, OpenCV 3.2.0
