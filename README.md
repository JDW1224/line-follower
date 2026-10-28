# Line Follower

**2025년 12월 · 로봇 프로그래밍 · 4조**

카메라 영상에서 주행선을 검출하고 ROS 2 속도 명령을 발행하는 학업 프로젝트입니다. 수업에서 사용한 `line_follower` 패키지의 소스를 정리해 보관했습니다.

## 코드에서 확인한 기능

- OpenCV/GStreamer로 카메라 영상을 받아 `/camera/image_raw` 토픽으로 발행합니다.
- `camera_viewer`가 영상을 변환해 `/camera/image_upright` 토픽으로 전달합니다.
- 영상의 관심 영역(ROI), HSV 범위 및 이진화 결과를 이용해 주행선 중심을 구합니다.
- 횡방향 PID 제어로 `/cmd_vel`을 발행하며, `/odom` 자세 정보와 라인 유실 시 복구·회전 로직을 사용합니다.
- OpenCV 트랙바로 HSV·이진화·블러 설정을 조절하고 디버그 이미지를 발행합니다.
- 별도 `lane_detector.py`에는 Canny/Hough 기반 선 검출과 디버그 영상 기능이 있습니다.

속도 제한 상수에는 TurtleBot3 사양을 참고한 값이 사용됩니다. 실제 하드웨어 구성과 주행 성능 기록은 소스만으로 확인되지 않습니다.

## 구성

```text
src/linefollower.cpp                CMake에서 빌드하는 주행 제어 노드
src/camera_viewer.cpp               카메라 영상 변환 노드
src/line_follower_jdw_ver.cpp        곡선 검출을 포함한 별도 소스 버전
include/line_follower/              C++ 헤더
scripts/camera_publisher.py         GStreamer 카메라 입력
scripts/lane_detector.py            선 검출 디버그 노드
launch/line_follower.launch.py      카메라·영상 변환·주행 노드 실행
CMakeLists.txt / package.xml        ROS 2 빌드·패키지 정의
```

`line_follower_jdw_ver.cpp`는 현재 `CMakeLists.txt`의 실행 대상에 포함되지 않습니다. `lane_detector.py`는 개별 실행이 가능하며 기본 launch 파일에서는 비활성화되어 있습니다.

## 준비 환경

- ROS 2 및 colcon/ament_cmake를 사용할 수 있는 Linux 환경
- C++17 지원 컴파일러: 소스에서 `std::clamp` 사용
- `rclcpp`, `rclpy`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `tf2`, `tf2_geometry_msgs`, `cv_bridge`, `image_transport`
- OpenCV(C++·Python), NumPy, GStreamer 입력을 지원하는 OpenCV 빌드
- 카메라와 `/cmd_vel`, `/odom` 인터페이스를 제공하는 로봇 또는 별도 환경
- OpenCV GUI를 표시할 수 있는 데스크톱 환경

기본 카메라 파이프라인은 NVIDIA Jetson의 `nvarguscamerasrc`를 사용합니다. USB 카메라 등에서는 입력 파이프라인을 변경해야 합니다. 원래 사용한 ROS 2 배포판과 정확한 라이브러리 버전 기록은 포함되어 있지 않습니다. `image_transport`는 `CMakeLists.txt`에서 요구하지만 원본 `package.xml`에는 누락되어 있으므로 별도로 준비해야 합니다.

## 빌드와 실행

사용 중인 ROS 2 환경을 먼저 source하고 위 의존성을 준비합니다. 저장소 루트에서 패키지를 ROS 2 작업공간에 연결합니다.

```bash
mkdir -p ~/line_follower_ws/src
ln -s "$(pwd)" ~/line_follower_ws/src/line_follower
cd ~/line_follower_ws
colcon build --packages-select line_follower --cmake-args -DCMAKE_CXX_STANDARD=17
source install/setup.bash
```

기본 NVIDIA 카메라 파이프라인을 사용하는 경우 다음 launch 파일을 실행합니다. 이 launch에는 주행 명령을 발행하는 노드가 포함됩니다.

```bash
ros2 launch line_follower line_follower.launch.py
```

USB 카메라를 사용하는 경우 기본 launch 대신 카메라 파이프라인을 지정해 실행할 수 있습니다. 장치 경로, 해상도와 입력 형식은 해당 카메라 환경에 맞춰야 합니다.

```bash
ros2 run line_follower camera_publisher.py --ros-args -p pipeline:="v4l2src device=/dev/video0 ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink"
```

별도 터미널에서 작업공간 환경을 source한 뒤 필요한 노드를 실행합니다.

```bash
ros2 run line_follower camera_viewer
ros2 run line_follower line_follower
```

위 두 노드는 각각 별도 터미널에서 실행합니다. 선 검출 디버그 노드를 추가할 경우 `ros2 run line_follower lane_detector.py`를 사용합니다.

## 공개용 정리와 확인 범위

원본 소스와 빌드 정의를 보존하고 README 및 `.gitignore`를 추가했습니다. Python 문법과 패키지 구성을 확인했습니다. 이번 정리 과정에서는 ROS 2 빌드, 카메라 입력 및 실제 로봇 주행을 실행 검증하지 않았습니다.

## 출처와 라이선스

원본 `package.xml`의 `Apache-2.0` 라이선스 및 maintainer 표기를 보존했습니다. 원본에는 별도 라이선스 전문 파일이 포함되어 있지 않습니다.
