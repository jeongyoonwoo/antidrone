# Anti-Drone Detection & Tracking System

24GHz FMCW 레이더와 카메라를 융합해 드론을 탐지·추적하고, Pan/Tilt 스테퍼 모터로 표적을 지향하는 시스템입니다.
레이더가 방위각·거리를 제공하고 카메라가 표적을 드론으로 식별하며, 두 정보를 융합해 추적 대상을 확정합니다.
신호처리(CORDIC 좌표변환, 칼만 추적 필터)와 모터 구동은 PYNQ-Z2(Zynq-7000) 상에서 PS/PL에 분담 구현했습니다.

동국대학교 전자전기공학부 캡스톤디자인(2026) 4인 팀 프로젝트이며,
본 저장소는 팀 저장소 [kimseojun89/capstone](https://github.com/kimseojun89/capstone)에서 포크한 것입니다.

**담당 범위(우정윤)** — FPGA 파트 전담 및 시스템 통합
호스트 PC 측 YOLOv8 학습·ByteTrack·GUI, 레이더 PPI, 하우징은 팀원 작업입니다.

---

## 시스템 구성

```
[HLK-LD2450]  --UART-->  +-----------------------------+
 24GHz FMCW               |  PYNQ-Z2 (xc7z020)         |
                          |   PS : UART 수신, 융합 판단,  |
[Camera] --USB--> [Host]  |        PID 제어             |
 YOLOv8x + ByteTrack  <-->|   PL : CORDIC, Kalman,     |
                          |        ULN2003 구동 IP      |
                          +-----------------------------+
                                       |
                                 [Pan/Tilt Stepper]
```

> **TODO**: 실제 블록다이어그램 이미지 1장과 시연 사진 1장을 `docs/`에 넣고 여기에 삽입

| 항목 | 사용 기술 |
|---|---|
| 레이더 | HLK-LD2450 (24GHz FMCW), UART 인터페이스 |
| FPGA | PYNQ-Z2 / Zynq-7000 (xc7z020), Vivado, Vitis HLS |
| 신호처리 | CORDIC 좌표변환 IP, 칼만 추적 필터 (오픈소스 기반 직접 설계·구현) |
| 영상 | YOLOv8x + ByteTrack (호스트 PC) |
| 구동 | ULN2003 스테퍼 구동 IP (PL) + PID 제어 (PS) |

---

## 주요 결과

- **추적 성공률 94.9%**
- **BRAM 사용률 85% → 15%** (Block RAM Tile 기준)
- **외부 처리 장치 2개 제거** — 모터 제어용 아두이노와 레이더 PPI용 라즈베리파이를 단일 SoC로 통합

### 최종 리소스 사용량 (xc7z020)

| 리소스 | 사용량 | 비율 |
|---|---|---|
| LUT | 31,244 / 53,200 | 58.7% |
| FF | 81,553 / 106,400 | 76.6% |
| BRAM | 21 / 140 | 15.0% |
| DSP | 173 / 220 | 78.6% |

---

## 설계 판단 기록

### MTI IP의 융합 지점 재배치

초기 설계에서는 MTI(Moving Target Indication) IP를 PL에 구현해 레이더 방위각과 영상 모션 정보를 PL 단에서 융합했습니다.
그러나 실측에서 두 정보의 융합이 불안정하게 동작했고, 융합 지점을 호스트의 YOLOv8 검출 경로로 옮기는 것으로 구조를 변경했습니다.
MTI IP는 블록 디자인에 남기되 펌웨어에서 비활성화했습니다.

이 재배치로 확보된 자원에 당초 별도 MCU로 처리할 예정이던 모터 제어를 통합했습니다.
라즈베리파이가 맡을 예정이던 기능은 Zynq PS(ARM)가 대체했고, 모터 제어는 PID를 PS에, ULN2003 구동 IP를 PL에 분담 배치했습니다.
결과적으로 추가 MCU 없이 단일 시스템으로 개발을 완료했습니다.

> 위 표의 최종 수치와 MTI 제거 전 리포트는 MTI 유무만 다른 비교가 아닙니다.
> 그 사이 코드 수정·개선으로 구성이 달라져 LUT·FF·DSP는 오히려 증가했으며, BRAM만 85%에서 15%로 감소했습니다.

### 알려진 이슈

- 표적이 한 번 포착된 뒤 사라지면 레이더 출력에 무효 값이 다수 발생해 PPI에 잡상이 표시되는 현상이 있었습니다. 시스템 통합 후 해소되었으나 원인은 확인 중입니다.
- UART 수신 값이 약 2초 지연되어 출력되던 현상은 Zynq UART 수신 FIFO(64바이트) 오버플로우로 인한 패킷 유실로 추정하고 있습니다.

---

## 저장소 구조

```
├── go.ps1                       실행 런처 (추적 / 캘리브레이션 / 사전 점검 / 모터 정지 메뉴)
├── Vitis/                       ★ HLS IP 소스 (C++)
│   ├── cordic_polar.cpp         CORDIC 극좌표 변환 (AXI4-Lite)
│   ├── kalman_filter.cpp        칼만 추적 필터 (AXI4-Lite)
│   └── uln2003_controller.cpp   28BYJ-48 듀얼 스테퍼 구동 (AXI4-Lite 제어 + GPIO 출력)
├── vivado_project/              ★ Vivado 프로젝트 (PYNQ-Z2, xc7z020)
│   ├── antidrone.xpr
│   ├── antidrone_wrapper.xsa    Vitis 플랫폼용 하드웨어 내보내기
│   └── antidrone.srcs/          블록 디자인(antidrone_bd), 핀 제약(antidrone.xdc, motor_pins.xdc)
├── antidrone/                   실행 시스템
│   ├── vitis_workspace/antidrone_app/src/ps_main.cpp
│   │                            ★ Zynq PS 베어메탈 펌웨어: 레이더 UART 파싱 → CORDIC/Kalman IP → Pan/Tilt PID
│   ├── cpp/                     호스트 추적 앱 (C++, OpenCV, ONNX Runtime)
│   │   ├── apps/                ptcamera_tracker(메인), detector_viewer, tracking_viewer, camera_smoke_test
│   │   └── include/, src/       YOLOv8 검출, ByteTrack 추적, 화면 표시, FPGA 시리얼 통신
│   ├── unified_gui.py           통합 GUI: 카메라 영상, 레이더 PPI, 텔레메트리, 3D 표시 (UDP 수신, 표시 전용)
│   ├── run_system.ps1           통합 실행: 비트스트림·ELF 적재, 사전 점검, 추적·캘리브레이션 실행
│   ├── flash.tcl                XSCT로 비트스트림과 ELF를 보드에 적재
│   └── models/drone_yolov8x/    YOLOv8x 모델 위치 (가중치 파일은 용량 때문에 저장소에 포함하지 않음)
├── scripts/
│   ├── pantilt_calibration/     카메라 내부 파라미터(ChArUco)와 픽셀 → 안테나 평면 좌표 변환 캘리브레이션
│   ├── build_app.py, build_app.tcl, run_build.bat   PS 펌웨어 빌드 (Vitis 2023.2)
│   ├── build_win.bat            호스트 C++ 앱 빌드 (Visual Studio Build Tools)
│   ├── recreate_platform.tcl    XSA에서 Vitis 플랫폼 재생성
│   └── gui_udp_sim.py           하드웨어 없이 GUI를 시험하는 가상 텔레메트리 송신기
├── data/                        캘리브레이션 데이터 (ChArUco 보드, 촬영 이미지, camera_intrinsics.yaml)
├── docs/                        시스템 개요, FPGA 작업 흐름, Pan/Tilt 캘리브레이션, ONNX/CUDA 적용 가이드
└── legacy/                      현재 파이프라인에서 쓰지 않는 파일 보관 (MTI 서브시스템, 구 PPI 뷰어, OpenVINO 문서 등)
```

★ 표시는 본인 담당(FPGA)입니다.

## 빌드 및 실행

> **TODO**: Vivado/Vitis 버전, 비트스트림 생성 절차, 호스트 실행 명령을 간단히 기재

---

## 향후 계획

- 안테나를 별도 설계·외주 제작하여 레이더 성능 개선
- 다중 표적 반사를 동시에 처리하고, 카메라가 각 표적을 드론/조류로 분류해 마킹한 대상만 추적하는 구조로 확장
- YOLOv8 경량 모델을 FPGA에 포팅하여 호스트 PC 없는 독립형 시스템 구현
