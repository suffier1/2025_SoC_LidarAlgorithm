# soc_lidar_real

## 개요
본 프로젝트는 LiDAR 기반 로봇 위치 추정 알고리즘 중 하나(Kalman Filter)를 ARM Cortex‑M 마이크로컨트롤러에 이식하여 Mbed OS 펌웨어로 실행하는 예제입니다. p5.js 시뮬레이터와 별개로, 실제 임베디드 환경(혹은 QEMU를 통한 가상 환경)에서 동작하는 펌웨어를 작성하고 빌드 및 에뮬레이션하는 과정을 담고 있습니다.

- **알고리즘**: Kalman Filter 기반 위치 추정
- **프레임워크**: Mbed OS 6
- **보드**: NUCLEO_F411RE (Cortex‑M4) / QEMU MPS2‑AN386 에뮬레이션

## 디렉터리 구조
```
soc_lidar_real/           # 프로젝트 루트
├─ mbed_app.json          # Mbed 프로젝트 설정
├─ mbed-os.lib            # Mbed OS 라이브러리 참조
├─ source/                # 소스 코드 디렉터리
│   ├─ main.cpp           # 메인 펌웨어 (UART 출력, 알고리즘 호출)
│   ├─ algorithms.h       # 알고리즘 헤더
│   └─ algorithms.cpp     # Kalman Filter 구현
└─ BUILD/                 # 빌드 아티팩트 (gitignore에 제외)
```

## 요구 환경
- Mbed CLI 1.x 또는 2.x
- GNU Arm Embedded Toolchain (arm-none-eabi-gcc)
- Python 3 (Mbed 도구 내부 사용)
- QEMU 시스템 ARM (Emulation)

## 1. 소스 코드 설명

### `algorithms.h`
Kalman Filter 인터페이스 선언:
```cpp
void updateKalmanFilter(float dt, float trueX, float trueY, float &estX, float &estY);
```

### `algorithms.cpp`
- `updateKalmanFilter()` 함수 내부에서 상태 예측(prediction)과 관측 업데이트(update) 수행
- 속도(`speed`), 노이즈 공분산(`Q`, `R`) 등을 하드코딩

### `main.cpp`
- `BufferedSerial pc(USBTX, USBRX, 115200);`로 UART 초기화
- `Timer timer;`로 시간 측정
- 루프 주기(`PERIOD`)에 따라:
  1. 실제 위치(`trueX`, `trueY`) 계산
  2. `updateKalmanFilter()` 호출
  3. `printf()` 형식으로 `timestamp,trueX,trueY,estX,estY` UART 전송
  4. `ThisThread::sleep_for(PERIOD)`로 타이밍 제어

## 2. 빌드 방법 (로컬)

```bash
# 프로젝트 루트로 이동
cd soc_lidar_real

# Mbed OS 업데이트 (최초 1회)
mbed deploy

# 빌드: NUCLEO_F411RE 보드를 대상으로
mbed compile -m NUCLEO_F411RE -t GCC_ARM
```

- 성공 시 `BUILD/NUCLEO_F411RE/GCC_ARM/soc_lidar_real.bin` 생성

## 3. QEMU 에뮬레이션

```bash
# 시스템 경로에 qemu-system-arm 추가 후 실행
qemu-system-arm \
  -M mps2-an386 \
  -cpu cortex-m4 \
  -kernel BUILD/NUCLEO_F411RE/GCC_ARM/soc_lidar_real.elf \
  -semihosting-config enable=on,target=native \
  -serial mon:stdio \
  -monitor none \
  -nographic
```

- UART 출력이 터미널에 실시간으로 표시됩니다

## 4. Git & GitHub

1. 로컬 Git 초기화:
   ```bash
   git init
   git add .
   git commit -m "Initial commit: soc_lidar_real"
   ```
2. GitHub 저장소 생성 후 원격 설정:
   ```bash
   git remote add origin https://github.com/<username>/soc_lidar_real.git
   git push -u origin master
   ```
3. `.gitignore`에 `BUILD/` 폴더 추가하여 빌드 결과 제외

## 5. 실행 결과 예시
```
1000,0.020,0.000,0.020,0.000  # timestamp(ms), trueX, trueY, estX, estY
2000,0.040,0.000,0.041,0.000
...
```



