# 단일 PDoA 앵커 연동 가이드

이 가이드는 HUB 담당자가 E2 `e6e07c1825095580188f98ad5ffa4ed15ee49dc8`의 **double 기반 V3 최소 프로파일**을 이식할 때 사용한다. 앵커 → HUB 입력은 연구용 **IF-0 R1**(앵커 `5f4204b`의 `docs/contracts/IF0_BYTE_CONTRACT_20261120.md`), HUB → 컨트롤 보드 출력은 BBX의 **실차 시험용 0x244 v0.2**를 따른다.

## 소스와 빌드

아래 경로는 이 가이드와 같은 `embedded/uwb_core/` 디렉터리를 기준으로 한다.

| 인계 파일 | 역할 |
|---|---|
| [uwb_core.c](uwb_core.c), [uwb_math.c](uwb_math.c), [uwb_engines.c](uwb_engines.c) | 필수 C11 소스 3개 |
| [uwb_core.h](uwb_core.h) | 필수 공개 헤더. HUB 호출부가 포함한다. |
| [uwb_internal.h](uwb_internal.h) | 필수 내부 헤더. 코어 빌드에 사용한다. |
| [CMakeLists.txt](CMakeLists.txt), [examples/minimal.c](examples/minimal.c) | 선택 사항. 호스트 라이브러리 빌드와 독립 실행 예제. |

기존 빌드에 소스를 직접 넣을 때는 Core 번역 단위에 `UWB_V3_ONLY=1`을 정의한다. CMake는 `-DUWB_V3_ONLY=ON`을 사용한다. GCC 기준 C11, IEEE 754 binary64, `-ffp-contract=off -fno-fast-math`, libm 링크를 유지한다. G474의 E2 측정 빌드는 `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Os`와 섹션 GC를 사용했다. ABI 설정은 통합 프로젝트와 일치시킨다. C++ 호출부는 공개 헤더의 C ABI를 사용할 수 있다.

리포지터리 루트에서 독립 예제를 빌드하고 실행할 수 있다. CMake와 호스트 C 컴파일러가 필요하다.

```powershell
$build = Join-Path $env:TEMP 'uwb-v3-example'
cmake -S embedded/uwb_core -B $build -DUWB_V3_ONLY=ON -DUWB_BUILD_EXAMPLE=ON -DCMAKE_BUILD_TYPE=Release
cmake --build $build --config Release
ctest --test-dir $build -C Release --output-on-failure -V
```

[독립 예제](examples/minimal.c)는 디코딩과 CAL 연결이 끝난 가상 관측을 넣고, 같은 확장 관측 번호로 PDoA와 TWR을 결합한 뒤 V3를 조회한다. 초기 재획득, LEFT·거리 출력, 250/251 ms 경계, 센터 세대 초기화를 확인한다. 예제의 `seq`는 `result.sequence`(확장 RF 관측 순번)이며 0x244의 발행 번호는 송신 어댑터에서 관리한다. 예제는 C API 호출까지만 보여준다.

## 호출 순서

1. 단일 앵커 스트림마다 `UwbCore` 인스턴스 하나를 정적 저장소에 둔다. `uwb_research_config()`를 `uwb_core_init()`에 전달한다. 코어 호출은 한 태스크에서 직렬화한다.
2. 신규 실제 교환에만 `uwb_core_observe()`를 호출한다. `UWB_ACCEPTED` 다음에 같은 교환의 `uwb_core_observe_range()`를 한 번 호출한다. 거리가 무효여도 `valid=false`로 결합할 수 있다. 이 호출 순서를 지켜야 현재 교환 거리가 결과에 연결되고 다음 관측의 TWR 조건에 사용된다.
3. `uwb_core_read(&core, UWB_V3, now_ms, &result)`로 결과를 읽는다. 거리와 좌우 유효성은 따로 확인한다. `UWB_DUPLICATE`, `UWB_UNPAIRED`, `UWB_BAD_INPUT`, `UWB_CAPACITY_EXCEEDED`는 어댑터 계수에 남기고, 거부된 입력으로 신선도를 갱신하지 않는다. 원인 분석용 `uwb_core_decision()`은 시각 만료가 적용되지 않는 관측 결과다.
4. 입력이 끊겨도 같은 HUB 단조 시계로 `uwb_core_read()`를 주기 호출한다. 마지막으로 받아들인 관측 도착 뒤 **250 ms 초과** 시 `tag_lost=true`이고 거리·좌우가 무효다. 아직 관측을 받지 않은 초기에는 RECOVERY다. 새 무효 관측이 들어오거나 센터가 무효면 모드는 LOST여도 `tag_lost=false`일 수 있다. 받아들인 무효 관측도 도착 시각을 갱신한다. `read()`는 추정기 상태를 갱신하지 않는다. 관측 tick 간격도 250 ms를 초과하거나 추정기가 무효 관측으로 LOST에 들어갔으면 기존 최근 유효 3표본·60 ms 재획득 경로를 사용한다. HUB 도착 지연에 따른 별도 epoch 초기화는 확정된 계약을 따른다.

## IF-0 R1 입력 매핑

앵커 AID `0x2001`은 **OBS_RAW 0x224**, **CAL_STATE 0x234**를 보낸다. 두 프레임 모두 **Classic CAN 500 kbps, 표준 11비트 ID, DLC 8, Little Endian**이다. OBS는 새 POLL 교환당 1회 큐에 넣고, CAL은 부팅·변경 시와 1 Hz로 보낸다.

### OBS_RAW 0x224

| 바이트 | R1 필드 | C API 입력 |
|---|---|---|
| B0 | `sn` (u8) | 수신 측에서 u32로 확장한 뒤 `UwbObservation.sequence`와 `UwbRangeObservation.sequence`에 같은 값을 준다. |
| B1 | `flags` | b0 `PHASE_VALID`, b1 `RANGE_VALID`, b2 `Q_FAIL`, b3 `STS0_FAIL`, b4 `STS1_FAIL`, b5 `ADC_WARN`, b6 `CAL_VALID`, b7 `CAL_GEN_LSB`. `(flags >> 2) & 0x0F`를 `.reason_flags`로 전달한다. b5는 실제 ADC 경고가 있었고 거절 사유가 없는 관측이다. Core에서는 경고 가중치를 적용한다. |
| B2–3 | `pdoa_raw` (i16 LE, rad×2048) | `.raw_pdoa`에 그대로 복사한다. b0이 켜져 있고 값이 `0x7FFF`가 아닐 때 `.pdoa_valid=true`로 둔다. b0이 꺼져도 읽은 원시 위상은 진단용으로 들어올 수 있다. |
| B4 | `sts_q` (u8) | `.sts_q`에 정수로 전달한다. `0xFE`는 254 포화, `0xFF`는 무효이므로 q=0으로 정규화해 게이트를 통과하지 않게 한다. 앵커의 Q_FAIL은 q<230이고 E2 Core 수용 조건은 **q>230**이다. |
| B5–6 | `dist_raw_mm` (u16 LE, mm) | **같은 교환의 필터 전 원시 DS-TWR**을 `UwbRangeObservation.distance_mm`로 준다. b1이 켜져 있고 값이 `0xFFFF`가 아닐 때 `.valid=true`다. `0xFFFE`는 65534 mm 이상 포화이므로 수신값 65534와 포화 여부를 보존한다. HOLD·기각·FINAL 미수신은 거리 무효다. |
| B7 | `obs_age_ms` (u8, 1 ms) | 0–253은 `t_obs = RX 시각 − obs_age_ms`로 복원한다. `0xFE`는 254 ms **이상**, `0xFF`는 미상이다. 두 코드는 정확한 RF 시각을 제공하지 않는다. |

- **순번.** 직전 수용 SN에 대해 `d = (uint8_t)(sn - last_sn)`을 계산한다. 0은 중복, 1–127은 수용하여 확장 순번에 d를 더하고 누락은 d−1로 기록, 128–255는 지연·역순으로 폐기한다. 정상 255→0은 d=1이다. 첫 프레임은 새 기준으로 수용하며, 중복·역순으로 도착 신선도를 갱신하지 않는다.
- **시각과 거리 결합.** `.arrival_ms`는 HUB 단조 시계의 실제 RX 시각(u64 ms), `.tick_ms`는 복원한 관측 시각을 Core 시간축의 u32 ms로 표현한 값이다. `now_ms`는 `.arrival_ms`와 같은 시계를 사용한다. 복원치는 POLL 진단부터 앵커 CAN 큐 투입까지의 경과만 보정하므로 큐 이후 지연은 남는다. IF-0에는 별도 TWR 완료 tick이 없으므로 동일 OBS에서 꺼낸 PDoA·TWR에 같은 `.tick_ms`를 써서 결합한다. 이는 Core 어댑터의 교환 시각 표현이다. 포화·미상 age, 시각 언더플로 또는 복원 tick 역행의 Core 입력 처리는 HUB 담당자가 정한다. 이 경우 계산 불가를 표시하고 추정값을 실제 RF 시각으로 기록하지 않는다.

### CAL_STATE 0x234

| 바이트 | R1 필드 | C API 입력·상태 |
|---|---|---|
| B0 | `cal_state` | 0 INVALID, 1 CENTER_ONLY, 2 CENTER_SIGN. 이전 값과 비교해 변경 시 추정기를 리셋한다. |
| B1–3 | `phi0_mdeg` (i24 LE, mdeg) | `0x7FFFFF`는 무효. 부호 확장한 i32를 단위 변환 없이 `.center_mdeg`에 넣는다. 범위는 ±180000 mdeg다. 예를 들어 `DC B1 FF`는 −20004 mdeg다. |
| B4–5 | `generation` (u16 LE) | 보정 세대의 하위 16비트다. 센터 캡처 시작과 초기화 때 증가하며 저장 시 유지된다. 세대가 바뀌면 같은 센터값도 새 보정으로 처리한다. |
| B6 | `flags` | b0 NVM_MISMATCH, b1 CAPTURE_ACTIVE, b2 SRC_NVM, b3 CAL_VALID, b4 DIRTY, b5–6 LR_SIGN, b7 예약(0). 부호 코드는 0 미정, 1 +1, 2 예약, 3 −1이다. E2 Core와 일치하는 코드는 3이다. |
| B7 | `boot_id` | 1–255. 이전 값에서 바뀌면 앵커 재시작으로 보고 SN 추적과 Core 상태를 초기화한다. |

- **CAL 연결.** 수신한 CAL의 유효 비트·기준값 범위·부호를 확인한다. 마지막 CAL 수신 후 3초 미만이고 OBS b6이 켜져 있으며 OBS b7이 `generation & 1`과 같을 때 해당 기준값으로 `.center_valid=true`를 전달한다. 그 외에는 방향을 무효로 두고 일치하는 CAL을 기다린다. 거리 유효성은 OBS b1로 별도 처리한다. CAL이 끊기면 새 OBS가 없어도 이전 방향의 발행을 중단한다.
- **리셋.** `generation`, `phi0_mdeg`, `LR_SIGN`, `cal_state`, `boot_id` 중 하나라도 바뀌면 `uwb_core_reset()`을 호출한다. 재부팅이면 SN 확장 상태도 초기화한다. 같은 `boot_id`가 다시 나올 수 있으므로 R1 §4의 SN 역행 지속 또는 T_rf 이상 OBS 공백 뒤 복귀 조건도 적용한다.
- **보정과 부호는 Core에서 한 번 적용한다.** 입력에는 원시 `pdoa_raw`와 `phi0_mdeg`를 각각 전달한다. LR_SIGN 코드는 Core와의 호환 확인에 사용한다. Core는 `wrap(raw / 2048 × 180 / π − center_mdeg / 1000)`로 센터를 빼고 `deviation_deg`에 −1 부호를 적용한다. IF-1 송신부는 이 편차를 cdeg로만 변환한다. 입력에서 센터를 미리 빼거나 입력·출력에서 부호를 다시 곱하면 판단 방향이 달라진다.

## IF-1 실차 시험용 v0.2 출력 매핑

기준 문서는 `BBX_PDoA_TestCAN_0x244_v0.2_20261120.md` §1–6과 `컨트롤보드_개발요청_PDoA실차시험_20261120.md` §1–2다. **Classic CAN 2.0B, 표준 11비트 ID 0x244, DLC 8, 500 kbps(개발용), Little Endian**이다. 목표 송신 주기는 25 Hz(40 ms)다.

| 바이트 | v0.2 필드 | C API와 송신 매핑 |
|---|---|---|
| B0–1 | `dist_mm` (u16 LE, mm) | `result.distance_mm`. `result.distance_valid`이고 0–65534 mm 범위일 때 유효하다. 무효면 `0xFFFF`. |
| B2–3 | `phase_cdeg` (i16 LE, 0.01°) | 유효한 `result.decision.deviation_deg`에 `round(100 × deviation_deg)`를 적용한다. 전송 범위는 −18000…17999이며 원형 경계의 +18000은 동치값 −18000으로 표현한다. `LAT_VALID=0`이면 `0x7FFF`. |
| B4 | `seq` (u8) | HUB 발행 번호. 새 결과 생성 시에만 +1(255→0), 같은 결과 재송신 시 유지한다. `result.sequence`는 RF 입력 순번이다. |
| B5 | `status` | b0–1 `lat`, b2–3 `src`, b4 `DIST_VALID`, b5 `LAT_VALID`, b6 `TAG_LOST`, b7 `RESTARTED`. |
| B6 | `quality` (u8) | 유효한 `decision.score`(0–1)를 ×100 하여 정수 0–100으로 변환한다. 관측 일관성 점수이며 `LAT_VALID=0`이어도 유효할 수 있다. 점수가 유효하지 않으면 `0xFF`(예: 관측 없음, TAG_LOST). `uwb_core_read()`의 무효화 후에도 이전 점수가 남을 수 있으므로 유한 여부만으로 TAG_LOST의 점수를 유효 처리하지 않는다. |
| B7 | `rf_age` (u8, 4 ms 단위) | 해당 결과를 지지한 실제 RF 관측부터 송신까지 경과한 4 ms 구간 수다. 40 ms는 10, 1016 ms는 `0xFE`, 1020 ms 이상 또는 관측 없음은 `0xFF`다. |

- **방향과 위상.** Core enum은 `UNKNOWN/LEFT/CENTER/RIGHT = 0/1/2/3`, 전송 `lat`는 `UNKNOWN/CENTER/LEFT/RIGHT = 0/1/2/3`이다. Core enum 순서로 `{0, 2, 1, 3}`을 적용한다. `deviation_deg`에는 센터 보정과 부호가 반영돼 있다. 양수는 LEFT 쪽, 음수는 RIGHT 쪽이며 실차 장착 기준 부호 확인이 남아 있다. 값의 의미는 센터 기준 상대 위상이다.
- **출처.** 유효 방향에서 `UWB_CURRENT_OBSERVATION` → `OBSERVED(1)`, `UWB_CONTINUITY_HOLD` → `HELD(2)`, `UWB_BOUNDARY_REFERENCE` → `REFERENCE(3)`이다. `NONE(0)`은 방향 무효에 사용한다. V3의 REFERENCE는 현재 경계 군집의 위상과 이전 판단을 함께 사용한다. 현재 군집만으로 판정한 BOUNDARY 결과는 OBSERVED다. HELD는 V2의 제한 시간 유지 결과이며 V3 최소 프로파일에는 없다.
- **유효성과 재시작.** `DIST_VALID`와 `LAT_VALID`는 독립이다. `LAT_VALID`는 `result.direction_valid`와 전송 가능한 상대 위상을 함께 요구한다. 방향 무효면 `lat=UNKNOWN`, `src=NONE`, `phase_cdeg=0x7FFF`다. `result.tag_lost`가 참이면 `TAG_LOST`를 세우고 두 유효 비트를 모두 내리며 거리에도 무효값을 넣는다. 수신 중 방향 보류만으로 `TAG_LOST`를 세우지 않는다. `RESTARTED`는 HUB가 재부팅 또는 추정기 리셋 이후의 구간을 표시한다.
- **RF 시각과 재송신.** OBSERVED는 결과를 지지한 관측, REFERENCE는 현재 군집에 쓰인 가장 최근 실제 관측, HELD는 유지값을 만든 마지막 지지 관측의 시각을 사용한다. HUB가 송신 시각과 같은 시간축으로 기준 시각을 관리한다. 코어의 `last_arrival_ms`는 가장 최근 수용 입력의 **도착** 시각으로 무효 표본도 포함하며, 공개 결과에는 군집에 쓰인 최신 RF 관측의 시각이 별도 필드로 제공되지 않는다. 동일 결과를 재송신할 때는 `seq`와 RF 기준 시각을 유지하고 `rf_age`만 다시 계산한다.

## HUB 담당 개발 요청사항

1. IF-0 수신 경로에서 표준 ID `0x224`·`0x234`와 DLC 8을 확인하고 RX 단조 시각을 기록한다. CAL 캐시와 확장 SN을 유지하고, 수용한 OBS만 PDoA→동일 교환 TWR→V3 조회 순서로 처리한다.
2. CAL 수신 만료, 세대 변경, 앵커 재시작과 나이를 복원할 수 없는 OBS를 처리한다. 거리·방향 유효성을 독립적으로 유지하고, RF 관측 공백 확인을 위해 새 입력이 없을 때도 Core를 조회한다.
3. IF-1 경로에서 `0x244` v0.2의 독립 발행 순번과 지지 RF 관측 기준 시각을 관리한다. IF-0 수신과 IF-1 시험 송신은 모두 Classic CAN 500 kbps다. 연동 확인 시 IF-0 원본과 이어서 발행한 IF-1 원본을 같은 HUB 시각축에 남기면 SN 순환, CAL 전환, 원시 거리 결합 및 상대 위상 부호를 대조할 수 있다.

## 실행 예산과 검증 범위

- 25 Hz 연구 조건은 앵커의 **새 교환마다 1회**, 약 40 ms 간격이다. 인터럽트는 수신 시각 기록과 큐 적재에 사용하고, 소유 태스크가 순번 순서로 코어를 실행한다. 발행·폴링 주기는 입력 주기와 독립적으로 관리한다. 128개 고정 창을 넘으면 오류를 반환한다.
- G474 최소 프로파일의 E2 교차 링크 기준 **코어 정적 RAM 12,496 B, 결과 96 B, 독립 프로브 Flash 26,448 B**다. 실제 통합 링크는 어댑터·태스크·공유 libm을 포함한다. MCU 사이클·스택 고수위·CAN 큐 지연은 아직 실측이 필요하다. [E2 측정 조건](E2.md)
- E1의 JS 대조는 원시 UART **364,150관측 + 합성 176입력**에서 불일치 0건, E2 전체·최소 비교는 **25 Hz 4,029관측 + 합성 176입력**에서 불일치 0건이었다. 이 결과는 기존 시간축·센터값을 사용한 비교다. E2에서 cdeg 센터 양자화가 방향을 바꾼 사례가 있었으며, R1은 i24 mdeg로 원래 정밀도를 보존한다. [검증 방법](README.md) · [E2 계약 검토](E2.md)
- 앵커 `5f4204b`의 500 kbps 연구 빌드는 R1 호스트 패킹 검증 4개 구성을 통과했다. 실제 IF-0 수신 데이터에서 순번·CAL 전환·복원 tick 및 25 Hz 캡처를 확인하는 것은 HUB 통합 단계에 남아 있다. 단일 앵커의 상대 위상과 좌우 판단은 연구용이며, 실차 부호와 물리 좌우는 별도 검증 대상이다. 전후방은 구분하지 못한다.
