# Intelligent UWB Embedded Core · E1

G474 실행 프로파일·계약 검토·벤치마크는 [E2](E2.md)에 정리했다.
HUB 담당 개발자는 [연동 가이드와 IF 매핑표](INTEGRATION.md), [독립 C 예제](examples/minimal.c)부터 시작한다.

정본은 대시보드 커밋 `0fd403077da493c58cbea08588e457e0c`의 PDoA 연구 세션이다. E1의 C11 정적 라이브러리는 기존 STM32 C 빌드에 연결하기 쉽고 호스트에서 같은 코드를 실행할 수 있다. IEEE 754 binary64와 동일한 연산 순서를 사용하고 FMA 수축과 fast-math를 끈다. C++ 호출부는 C ABI를 사용할 수 있다.

| 파일 | 역할 |
| --- | --- |
| `uwb_core.h`, `uwb_core.c` | Observation·Range·Tracking Result 계약, 초기화, 고정 창, 관측과 거리 연결, 만료 조회 |
| `uwb_math.c` | 원형 통계·IIR, 가중 투표·군집, 링버퍼 |
| `uwb_engines.c` | Adaptive Chunking, V1 모드 전환, V2 추적 유지, V3 경계 판단 |
| `tests/` | JS 정본과 C의 비교 실행기, 합성 경계 검사, 독립 링크 크기 프로브 |

## 입력과 출력

| 입력 | 표현과 의미 |
| --- | --- |
| `raw_pdoa`, `pdoa_valid` | `int16_t` 원시값, 단위 rad × 2048. 유효 여부는 별도 플래그이며 원시값에는 센티널이 없다. |
| `sts_q`, `reason_flags` | `int16_t` 정수 STS q와 `uint16_t` 비트. 하위 3비트는 가중·엔진에서 제외하고, bit 3은 ADC 경고로 가중치를 적용한다. 가중치는 q를 최대 256으로 제한하며 엔진의 현재 품질 검사에는 입력 q를 사용한다. |
| `tick_ms`, `sequence` | 보드 측 `uint32_t` 밀리초·관측 번호. 동일 쌍은 중복이며, 번호 건너뜀은 Adaptive 누락에 반영한다. 둘 중 하나가 감소하면 JS처럼 새 epoch로 초기화한다. 32비트 래핑도 여기에 해당한다. |
| `arrival_ms`와 `now_ms` | 동일한 로컬 단조 시계의 `uint64_t` 밀리초. 32비트 플랫폼 시계는 어댑터가 확장한다. `arrival_ms`는 실제 관측의 수신 시각이며 결과 조회의 기준이다. |
| `center_mdeg`, `center_valid` | `int32_t` 밀리도 −180,000~180,000과 유효 플래그. 유효하지 않으면 값은 무시한다. 보드에서 센터를 실제로 변경·재확인한 경우 어댑터는 PF13 경로의 세션 리셋을 전달한다. |
| `UwbRangeObservation` | 같은 관측에 연결된 `uint32_t` 보드 tick·관측 번호, `int32_t` 거리 mm(유효 시 0~INT32_MAX), 유효 플래그. 관측마다 한 건을 받는다. PF13의 SN·원시 PDoA·유효 비트 대조는 어댑터가 담당한다. D 처리 시 최신 유효 TWR은 1,500 ms까지 참고한다. 500 mm 초과 변화는 급변으로 보고, V2 유지 지원에는 250 ms 이내의 TWR을 요구한다. |

Ipatov power/F1/누적값, STS power/누적값 및 상태 레지스터 `st0/st1`은 JS가 기록하는 진단값이다. 현재 판단에는 원시 PDoA·TWR·STS q·reason 비트와 이 입력에서 유도되는 분산·점유율·추세·누락을 쓴다.

호출자는 `UwbCore`를 정적으로 할당하고 `uwb_research_config()`로 얻은 설정을 `uwb_core_init`에 전달한다. 연구 프리셋은 200/400 ms, 합의율 0.55, 기본 CENTER 20°, Adaptive 30°/50°, q 하한 230이다. 설정 변경은 재초기화로 적용한다. 각 인스턴스의 호출은 직렬화한다. 128개 고정 관측 및 복구 창의 한계를 넘는 입력은 `UWB_CAPACITY_EXCEEDED`를 반환하며 이전 결과와 수신 시각을 유지한다. Adaptive의 600 ms·최근 128개 절단은 JS 규칙과 같다.

`uwb_core_decision`은 17개 후보 중 지정 후보의 순수 관측 결과를 제공한다. `uwb_core_read`는 현재 유효 방향·모드·처리 상태·사유 비트, 거리 및 유효 여부, 마지막 실제 관측의 tick·수신 시각·시퀀스를 복사한다. 거리는 현재 관측에 연결된 유효 값이며 만료 또는 미결합 시 `distance_valid=false`와 `-1`을 반환한다. 방향은 LEFT/CENTER/RIGHT/UNKNOWN이며, phase는 센터를 뺀 원형 위상, deviation은 기존 LR_SIGN을 적용한 상대 위상이다. 수치가 정의되지 않으면 NaN을 반환한다. 점수는 관측 일관성 지표다.

결과의 `origin`은 현재 관측, V2 제한 유지, V3 현재 군집과 이전 판단을 함께 사용한 히스테리시스를 구분한다. 폴링은 수신이 멈춰도 계속 호출한다. 마지막 실제 관측에서 **250 ms 초과** 시 `tag_lost`를 표시하며, 센터 미확인·현재 표본 무효는 별도 사유다. 초기 입력 대기는 RECOVERY이고, V2 유지 만료 시 UNKNOWN으로 돌아간다. 거리와 방향의 유효 플래그는 독립적이다. 이 구조는 어댑터가 읽는 C 계약이며 IF 패킷, 방위각, 전후방 및 조향 필드를 정의하지 않는다.

## 검증과 자원

```powershell
cmake -S embedded/uwb_core -B C:/Temp/uwb-host -DUWB_BUILD_HOST_REPLAY=ON -DCMAKE_BUILD_TYPE=Release
cmake --build C:/Temp/uwb-host --config Release
ctest --test-dir C:/Temp/uwb-host -C Release --output-on-failure
$capture = 'C:/HIL/PDOA_CAL_AG/LIVE_25HZ_20261119_134633'
node embedded/uwb_core/tests/compare.js C:/Temp/uwb-host/uwb_replay.exe "$capture/capture/COM5_chunks.jsonl" "$capture/capture_adaptive_20261119_150730/COM5_chunks.jsonl"
```

비교기는 두 원본 UART를 기존 PF13 파서와 JS 세션에 다시 입력한다. 두 캡처에는 동일한 최종 연구 프리셋을 적용하고, 파일 사이와 파서에서 지정한 epoch에서 초기화한다. 방향·모드·유지·경계 여부와 시간은 정확히 일치해야 한다. 위상 원형 차이 1e−8°, σ 3e−6°, 점수·점유율 1e−10을 수치 허용오차로 사용한다. σ의 영점 근처에서는 libm의 coherence 반올림 차이가 제곱근에서 확대된다. 합성 입력으로 ±180° 경계, 무효·센터 변경, ADC, TWR 급변, 시퀀스 누락·32비트 롤오버, 250/251 ms 및 재획득도 확인한다.

Windows GCC 16.1.0에서 원시 관측 364,150개와 합성 입력 176개를 재생해 불일치 0건을 확인했다. V1 모드 전환 9,832건, V2 제한 유지 8,406건, V3 BOUNDARY 2,449건이 포함됐다. 최대 수치 차이는 위상 2.28e−13°, σ 8.54e−7°, 점수 2.23e−16, 점유율 0이었다. 25 Hz 캡처 말미의 4,029개 관측과 합성 검사를 함께 재생한 호스트 평균은 약 4.6 µs였다. 이 시간은 관측 계산 호출만 측정한 값이다.

GNU Tools for STM32 14.3.1, `-Os`, hard-float, newlib-nano 기준이다. 별도 `tests/size_probe.c`를 코어 3개 번역 단위와 링크했다. 보드 시작 코드·HAL·RTOS 없이, 라이브러리 코드와 libm/libgcc 등 링크된 의존성을 포함한 수치다.

| 항목 | Cortex-M4 · G474RE | Cortex-M7 · F767ZI |
| --- | ---: | ---: |
| 코어 1개 정적 RAM | 12,496 B | 12,496 B |
| 결과 구조체 | 96 B | 96 B |
| 독립 프로브 Flash `text + data` | 27,776 B | 21,856 B |
| 관측 함수 자체 스택 `-fstack-usage` | 672 B | 656 B |
| 관측→Adaptive→군집 경로의 코어 프레임 합계 | 1,376 B | 1,328 B |

스택 합계에는 libm·libgcc와 호출 태스크·ISR 프레임이 포함되지 않는다. 동일 빌드는 `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16` 또는 `-mcpu=cortex-m7 -mfpu=fpv5-d16`, 공통 `-mthumb -mfloat-abi=hard -Os -std=c11 -fstack-usage -ffunction-sections -fdata-sections -ffp-contract=off -fno-fast-math --specs=nano.specs --specs=nosys.specs -nostartfiles -Wl,--gc-sections,-e,main -lm`로 재현할 수 있다. M4는 단정밀도 FPU에서 binary64 기본 연산에 소프트웨어 도우미를 사용하고, M7은 배정밀도 FPU를 지원한다. [STM32G474RE](https://www.st.com/en/microcontrollers-microprocessors/stm32g474re.html) · [STM32F767ZI](https://www.st.com/en/microcontrollers-microprocessors/stm32f767zi.html)

25 Hz의 주기는 40 ms다. 고정 기간 내 군집 검색은 표본 수에 대해 최대 제곱 비용이며, 실제 G474/F767 시간·스택 여유는 DWT 사이클 및 태스크 스택 고수위로 확인해야 한다. 앵커마다 독립 상태를 두면 2개는 24,992 B, 4개는 49,984 B의 코어 RAM이 필요하다. 같은 태스크에서 순차 처리할 경우 처리 스택과 코드 영역을 재사용할 수 있다.

수집 보드 이식 단계에서는 IF-0/IF-1 확정 후 어댑터의 관측 연결·단조 시계·센터 리셋 처리를 연결하고, 실제 타깃 빌드에서 시간·메모리와 수신 타이밍을 계측한다.
