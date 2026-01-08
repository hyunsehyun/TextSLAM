# ROS2 Bag to TextSLAM Dataset Converter

ROS2 bag 파일을 TextSLAM 데이터셋 형식으로 변환하는 도구입니다. PaddleOCR을 사용하여 이미지에서 텍스트를 검출하고 인식합니다.

## 기능

- ROS2 bag에서 이미지 추출
- PaddleOCR을 사용한 텍스트 검출 및 인식
- TextSLAM 호환 형식으로 출력 생성

## 설치

### 1. Conda 환경 생성 (권장)

```bash
conda create -n navocr python=3.10
conda activate navocr
```

### 2. 의존성 설치

```bash
# PaddleOCR 및 PaddlePaddle 설치 (버전 호환성 중요)
pip install "paddleocr>=2.6.0,<3.0" "paddlepaddle>=2.5.0,<3.0"

# 기타 의존성
pip install rosbags opencv-python numpy
```

## 사용법

### 기본 실행

```bash
conda activate navocr

python3 rosbag_to_textslam.py \
    --bag_path /path/to/rosbag2_folder \
    --output_dir /path/to/output \
    --image_topic /camera/infra1/image_rect_raw
```

### 명령줄 인자

| 인자 | 필수 | 기본값 | 설명 |
|------|------|--------|------|
| `--bag_path` | O | - | ROS2 bag 디렉토리 경로 |
| `--output_dir` | O | - | 출력 디렉토리 경로 |
| `--image_topic` | X | `/camera/infra1/image_rect_raw` | 이미지 토픽 이름 |
| `--lang` | X | `en` | OCR 언어 (en, korean 등) |

### 실행 예시

```bash
# 예시 1: 기본 설정으로 실행
python3 rosbag_to_textslam.py \
    --bag_path ~/Downloads/rosbag2_2025_12_05-12_39_09 \
    --output_dir ~/ros2_ws/src/TextSLAM/output

# 예시 2: 다른 이미지 토픽 사용
python3 rosbag_to_textslam.py \
    --bag_path ~/Downloads/my_rosbag \
    --output_dir ~/TextSLAM/custom_dataset \
    --image_topic /camera/color/image_raw
```

## 출력 구조

실행 후 생성되는 파일 구조:

```
output_dir/
├── custom_indoor.yaml     # TextSLAM 설정 파일
├── Exper.txt              # 이미지 목록
├── images/
│   ├── [timestamp].png
│   ├── [timestamp].png
│   └── ...
└── text/
    ├── [timestamp]_dete.txt   # 텍스트 바운딩 박스 좌표
    ├── [timestamp]_mean.txt   # 인식된 텍스트 및 신뢰도
    └── ...
```

### 파일 형식

**Exper.txt**
```
1764905949.522972656 images/1764905949.522972656.png
1764905949.556406494 images/1764905949.556406494.png
```

**[timestamp]_dete.txt** (4점 바운딩 박스 좌표)
```
636.0,195.0,679.0,208.0,672.0,232.0,628.0,219.0
```

**[timestamp]_mean.txt** (텍스트, 신뢰도)
```
arena,0.9879826307296753
```

## TextSLAM 실행

변환 완료 후 TextSLAM 실행:

```bash
cd ~/ros2_ws/src/TextSLAM
./build/TextSLAM ./custom_dataset/coex_0108_1/coex_0108_1.yam
```

## 카메라 파라미터 수정

다른 카메라를 사용하는 경우, `rosbag_to_textslam.py`의 `CAMERA_PARAMS`를 수정하세요:

```python
CAMERA_PARAMS = {
    'fx': 432.17333984375,    # 초점 거리 x
    'fy': 432.17333984375,    # 초점 거리 y
    'cx': 431.1373291015625,  # 주점 x
    'cy': 237.8980712890625,  # 주점 y
    'k1': 0.0,                # 왜곡 계수
    'k2': 0.0,
    'p1': 0.0,
    'p2': 0.0,
    'k3': 0.0,
    'width': 848,
    'height': 480,
    'fps': 30.0,
    'rgb': 0                  # 0: BGR/Grayscale, 1: RGB
}
```

카메라 파라미터는 ROS2에서 확인 가능:
```bash
ros2 topic echo /camera/infra1/camera_info --once
```

## 문제 해결

### PaddlePaddle 버전 충돌
```
ImportError: cannot import name 'forward_complete_op_role'
```
해결: PaddlePaddle 버전을 맞춰서 재설치
```bash
pip uninstall paddlepaddle paddlepaddle-gpu paddleocr -y
pip install "paddleocr>=2.6.0,<3.0" "paddlepaddle>=2.5.0,<3.0"
```

### 토픽을 찾을 수 없음
```
Error: Topic '/camera/infra1/image_rect_raw' not found in bag.
```
해결: bag 파일의 토픽 목록 확인 후 올바른 토픽 지정
```bash
ros2 bag info /path/to/rosbag
```
