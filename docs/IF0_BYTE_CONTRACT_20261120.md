# IF-0 바이트 계약 — PDoA 앵커 → UWB 허브 (연구, 2026-11-20)

- 상태: **연구 계약 R1**. DRAFT v2 §2.2 IF-0 공통안을 그대로 쓰되, E2 검토 중 통합에 필요한 항목(센터 정밀도, ADC 경고 의미, seq 랩·boot_id·재시작, generation)만 반영했다. v1/v2 문서는 고치지 않는다.
- 범위: 단일 태그 + 단일 PDoA 앵커, CH9 DS-TWR, 25 Hz, 내부 버스 Classic CAN 500 kbps. 다중 앵커, CIR, 새 알고리즘은 다루지 않는다.
- 앵커는 **좌우(LR) 판정 결과를 보내지 않는다.** 원시 관측과 이미 있는 보정 상태만 전달하고, 판정은 허브 Core가 한다.
- 구현 위치: 02 저장소 브랜치 `codex/if0-pdoa-obs-25hz`
  - 패킹: `Core/Inc/product_pdoa_if0.h`
  - 연결: `Core/Src/main.c`의 `PRODUCT_EXPERIMENTAL_PDOA_IF0_CAN` 블록(기본 0)
  - 시험: `tests/test_product_pdoa_if0.c`, `tests/verify_pdoa_if0.py`
- 500 kbps는 IF-0 송신이 켜진 연구 앵커에만 적용한다. 출하 및 IF-0 비활성 빌드는 250 kbps다. 연구 앵커의 레거시 프레임도 같은 FDCAN1에서 500 kbps로 전송된다.
- 펌웨어 구성: 25 Hz 연구 앵커 구성에 `-DPRODUCT_EXPERIMENTAL_PDOA_IF0_CAN=1`을 더한다. 빌드 도구는 `tools/pdoa_if0/build_if0_research.py --role anchor --if0 1`. 출하 빌드에서는 이 코드가 전처리 단계에서 빠진다.

500 kbps 연구 이미지의 USB-CAN 검증에는 `hil_if0_capture.py --bitrate 500000`을 명시한다. 보드 역할과 포트는 검증 시점에 다시 식별한다.

## 1. 공통

| 항목 | 값 |
|---|---|
| 버스 | Classic CAN 500 kbps, 11비트 표준 ID, DLC 8 고정 (`Core/Src/fdcan.c`: PCLK1 170 MHz, 클럭 분주 DIV1, 명목 분주 17, 1+15+4 TQ, SJW 1 TQ, 샘플 지점 80%, 자동 재전송 꺼짐, TX FIFO 순서 유지) |
| 바이트 순서 | 리틀 엔디언 |
| ID | OBS_RAW `0x223 + (AID & 0x0F)`, CAL_STATE `0x233 + (AID & 0x0F)` |
| 허용 AID | 하위 니블 0x1–0xC만 허용한다. 그 밖의 AID에서는 IF-0 송신이 꺼지고 부팅 로그에 `[IF0] enabled=0`이 찍힌다. |
| 현재 앵커 | AID 0x2001 → **OBS 0x224, CAL 0x234** |
| 레거시 | `0x200 + AID` DLC 4 거리 프레임은 내용과 주기 모두 그대로다(`verify_pdoa_if0.py`로 호출부 불변 확인). |
| UART | D/P 행은 그대로다(P 문장 불변 검사). 주기 행은 새로 넣지 않았고, 부팅 시 `[IF0] ...` 한 줄만 추가된다. |

## 2. OBS_RAW `0x223+(AID&0xF)` — POLL 교환 1건당 정확히 1회

| 바이트 | 필드 | 형식 | 단위·스케일 | 무효·포화 |
|---|---|---|---|---|
| B0 | `sn` | u8 | POLL SN(교환 식별자). P 행 sn, 레거시와 같은 교환 번호. 0xFF → 0x00으로 순환 | — |
| B1 | `flags` | 비트 | §2.1 | — |
| B2–3 | `pdoa_raw` | i16 LE | `dwt_readpdoa()` 원시값, **rad × 2048** (phase_rad = raw / 2048) | 위상을 읽지 못했을 때만 0x7FFF |
| B4 | `sts_q` | u8 | `dwt_readstsquality()` 값. 음수 → 0 | 0xFF 무효, 0xFE(=254) 포화 |
| B5–6 | `dist_raw_mm` | u16 LE | **같은 SN 교환**에서 새로 계산된 원시 DS-TWR 거리, mm (필터 전 `raw_input_mm`) | 0xFFFF 무효, 0xFFFE 포화(≥65534 mm) |
| B7 | `obs_age_ms` | u8 | POLL 진단 취득 시각 → 이 프레임을 CAN 큐에 넣은 시각, **1 ms** 단위 (HAL_GetTick 차이, ±1 ms 양자화) | 0xFF 모름, 0xFE ≥254 ms 포화 |

### 2.1 OBS flags (B1)

| 비트 | 이름 | 의미 |
|---|---|---|
| b0 | PHASE_VALID | `product_pdoa_observation_valid()` 결과(펌웨어 PDoA 유효 판정과 같음). 0이어도 B2–3에는 읽은 원시값이 실린다(진단용). |
| b1 | RANGE_VALID | B5–6이 이 교환에서 새로 계산된 거리. 기각, HOLD, FINAL 미수신이면 0이고 dist는 0xFFFF |
| b2 | Q_FAIL | D 행 reason bit0: `sts_q < 230` |
| b3 | STS0_FAIL | D 행 reason bit1 |
| b4 | STS1_FAIL | D 행 reason bit2 |
| b5 | ADC_WARN | D 행 reason bit3. **거절 사유(b2–b4)가 없고 STS 상태에 실제 ADC count 경고 비트가 있는** 관측이다. 경고가 있어도 위상은 허용됐다는 뜻(`product_pdoa_sts_diagnostic.h:20-27`). 거절 사유가 있으면 0 |
| b6 | CAL_VALID | OBS 생성 시점의 CAL 스냅숏이 유효한 기준값(φ0)을 가짐(= CAL_STATE B6 b3) |
| b7 | CAL_GEN_LSB | CAL_VALID=1일 때 `generation & 1`, 아니면 0. CAL_STATE와 OBS를 짝짓는 epoch 표시 |

- b2–b5 = `product_pdoa_sts_reason(sts_q >= 230, st0, st1) << 2`이다. D 행과 같은 식·같은 인자다(`main.c` `product_pdoa_if0_send_obs`).
- 허브의 q 게이트(E1의 q>230 등)는 수신 측에서 B4로 다시 판정한다. 앵커는 q를 가공하지 않고 그대로 보낸다.

### 2.2 OBS 송신 시점

- POLL을 받으면 위상·q·STS를 읽는다(`product_pdoa_anchor_capture_poll`). 이어 같은 교환의 FINAL 처리(성공 또는 실패/시간초과)가 끝난 뒤, 다음 루프 시작의 `product_pdoa_anchor_flush_poll`에서 OBS 1프레임을 큐에 넣는다(`main.c:10451-10453`, 호출 `main.c:3778`).
- 거리 결합: `hr_anc`는 POLL마다 `raw=-1`로 초기화되고(`main.c:11014-11015`) FINAL 계산 시 `raw_input_mm`로 채워진다(`main.c:11670`). `hr_anc.sn == sn`이고 `raw > 0`일 때만 RANGE_VALID=1이다. P 행 결합식(`main.c:3820`)과 같다.
- 송신 순서: 같은 교환의 레거시 0x201(FINAL 처리 중) → (변경이 있으면) CAL_STATE → OBS_RAW. TX FIFO가 순서를 지킨다.
- 재송신, HOLD, 타이머 송신은 없다. FIFO가 가득 차는 등으로 송신이 실패하면 그 프레임은 버리고 RAM 카운터(`g_if0.obs_tx_fail`)만 올린다. 수신 측은 sn 공백으로 손실을 판단한다.
- 따라서 **OBS_RAW 도착 = 신규 RF 관측**이다(v2 O-1).

## 3. CAL_STATE `0x233+(AID&0xF)` — 부팅 즉시 + 1 Hz + 변경 즉시

| 바이트 | 필드 | 형식 | 단위·의미 | 무효 |
|---|---|---|---|---|
| B0 | `cal_state` | u8 | 0 INVALID, 1 CENTER_ONLY, 2 CENTER_SIGN (`pdoa_phys_state_t`, `product_pdoa_physical_cal.h:32-34`) | — |
| B1–3 | `phi0_mdeg` | **i24 LE** | 센터 기준 위상 φ0, **m°(0.001°)**, 범위 ±180000 | 0x7FFFFF (기준값 없음 또는 범위 밖) |
| B4–5 | `generation` | u16 LE | 보정 세대(NVM u32)의 하위 16비트 | — |
| B6 | `flags` | 비트 | §3.1 | — |
| B7 | `boot_id` | u8 | 부팅마다 새로 만드는 0이 아닌 값(1–255) | 0은 쓰지 않음 |

### 3.1 CAL flags (B6)

| 비트 | 이름 | 의미 |
|---|---|---|
| b0 | NVM_MISMATCH | 저장 레코드와 RAM이 어긋남(기존 상태 그대로) |
| b1 | CAPTURE_ACTIVE | 센터/부호 캡처 진행 중 |
| b2 | SRC_NVM | 기준값 출처가 유효 NVM V4 레코드(CRC 정상, UID 일치, state ≥ 1). 0이면 RAM이거나 기준값 없음 |
| b3 | CAL_VALID | φ0가 유효하고 표현 범위 안. OBS b6과 같은 값 |
| b4 | DIRTY | RAM 보정이 아직 저장되지 않음 |
| b5–6 | LR_SIGN | 0 = 미정, 1 = +1, 3 = −1, 2 = 예약. 현재 펌웨어의 컴파일 상수 `PRODUCT_PDOA_LR_SIGN (-1)`(`product_pdoa_lr.h:11-17`) → 코드 3 |
| b7 | — | 0 (예약) |

- 기준값 선택 우선순위는 펌웨어 `product_pdoa_lr_reference()`와 같다: 유효 NVM → 아니면 캡처 중이 아닌 RAM 센터 → 없으면 무효. 호스트 시험 `test_cal_from_physical`에서 두 결과가 같은지 대조한다.
- 송신 조건:
  - 부팅 직후 1회(`product_pdoa_if0_init`)
  - 이전에 보낸 8바이트와 다르면 즉시
  - 같아도 마지막 송신 후 1000 ms가 지나면 1회
- 점검 시점: 교환 중에는 매 루프 OBS 송신 직전(최소 20 ms 간격). 태그가 200 ms 이상 없으면 POLL 대기 루프에서 50 ms 간격.
- 송신이 실패하면 다음 점검에서 다시 보낸다(CAL만 해당, 상태 프레임이라 중복 무해).

## 4. 순번·재시작·세대 규칙 (수신 측 필수)

1. **sn 순환 해제**: 직전 수용 sn을 `last`로 두고 `d = (u8)(sn − last)`를 계산한다.
   - `d = 0`: 중복이므로 버린다.
   - `d = 1..127`: 수용한다. 누락은 `d − 1`개이고, u32 확장 번호에 `d`를 더한다.
   - `d = 128..255`: 늦게 왔거나 순서가 뒤바뀐 프레임이므로 버린다.
   - 참조 구현: `pdoa_if0_sn_unwrap()`.
2. **재시작 감지(RESTARTED)**: CAL_STATE B7 `boot_id`가 바뀌면 앵커가 재부팅된 것이다.
   - sn 추적을 초기화하고(다음 OBS를 첫 프레임으로 수용) 추정기를 리셋한다.
   - 앵커는 부팅 직후 OBS보다 먼저 CAL_STATE를 보낸다.
   - boot_id가 우연히 같은 값으로 다시 나올 확률은 약 1/255다. 보조 판단 기준: sn 역행(`d ≥ 128`)이 계속되거나 OBS가 T_rf 이상 끊긴 뒤 다시 들어오면 재시작으로 본다.
3. **세대(generation) 규칙**:
   - 펌웨어는 **센터 캡처를 시작할 때**와 **보정 초기화(clear) 때** generation을 1 올린다(`product_pdoa_physical_cal.c:62-65, 251-255`). 저장(save)할 때는 그대로다.
   - 같은 φ0로 다시 캡처해도 세대가 바뀐다.
   - 수신 측은 CAL_STATE의 `generation`, `phi0_mdeg`, `LR_SIGN`, `cal_state`, `boot_id` 가운데 하나라도 바뀌면 좌우 추정 상태를 리셋한다.
4. **OBS ↔ CAL 짝짓기**:
   - OBS의 CAL_VALID=1이고 `CAL_GEN_LSB == (마지막 CAL_STATE generation & 1)`일 때만 그 φ0를 써서 보정한다.
   - 둘이 다르면(새 CAL_STATE를 아직 못 받음) LAT_VALID=0으로 두고 다음 CAL_STATE를 기다린다.
   - CAL_STATE가 3 s(1 Hz 기준 3회) 이상 오지 않으면 보정 무효로 본다.
5. **관측 시각**: `t_obs = 허브 RX 시각 − obs_age_ms`(0xFF는 보정 불가, 0xFE는 254 ms 이상).
   - 계산되는 범위는 앵커 내부의 POLL 진단 → CAN 큐 구간뿐이다. CAN 중재와 전송 시간(≤0.27 ms/프레임)은 포함되지 않는다.
6. **유효성 사용**:
   - 위상 사용 조건: `PHASE_VALID=1` ∧ `CAL_VALID=1` ∧ 허브 q 게이트.
   - 거리 사용 조건: `RANGE_VALID=1`일 때만.
   - 두 비트를 서로 대신 쓰지 않는다.
   - `ADC_WARN=1`은 유효 관측이며, 통계나 품질 표시에만 쓴다.
7. 허용 범위 밖 ID(AID 니블 0, 0xD–0xF로 접힌 값)는 무시한다.

## 5. DRAFT v2와의 차이 (E2 통합 항목만)

| 항목 | v2 | R1 (이 계약) | 이유 |
|---|---|---|---|
| CAL φ0 | B1–2 `phi0_cdeg` i16 (0.01°) | B1–3 `phi0_mdeg` i24 (0.001°) | 센터 정밀도(E2): NVM의 mdeg 값을 손실 없이 전달. 시험: −20.004° → −20004 왕복(0xFFB1DC) |
| CAL generation | B3–4 | B4–5 (하위 16비트) | φ0 확장에 따른 이동 |
| CAL lr_sign | B5 i8 | B6 b5–6 코드 | 바이트 확보 |
| CAL flags | b0 NVM_MISMATCH, b1 CAPTURE_ACTIVE, b2 UID_MATCH | b2 = SRC_NVM(UID 일치 포함 유효 NVM), b3 CAL_VALID, b4 DIRTY 추가 | 기준값 출처를 펌웨어 우선순위와 같게 |
| CAL B7 | `seq` | `boot_id` | v2 §N6 (UWB Q-4) |
| OBS b5 | ADC_ALLOWED | ADC_WARN(뜻은 같음: 실제 경고 + 거절 없음) | 의미 명확화 |
| OBS b7 | 0 | CAL_GEN_LSB | 세대 전환 짝짓기 |
| OBS obs_age | 5 ms 단위 | **1 ms 단위** (0xFE 포화 = 254 ms) | 5 ms 양자화 지적(E2) |
| dist 무효값 | 0xFFFF / 0x7FFF 미정(§N1) | **0xFFFF** (0xFFFE 포화) | 0x240·0x244와 같은 관례 |

## 6. 시험 벡터 (`tests/test_product_pdoa_if0.c`와 같음)

| 프레임 | 입력 | 바이트 B0..B7 (hex) |
|---|---|---|
| OBS 정상 | sn 0x5A, raw −1234, PHASE·RANGE 유효, q 241, reason 0, dist 2345 mm, CAL_VALID, gen 7(홀수), age 3 ms | `5A C3 2E FB F1 29 09 03` |
| OBS 전부 무효 | sn 0x5A, 위상·q·거리·age 모름, CAL 무효 | `5A 00 FF 7F FF FF FF FF` |
| CAL 정상 | state 1, φ0 27.231° (27231 m°), gen 0x00012345, SRC_NVM, CAL_VALID, sign −1, boot_id 0xA7 | `01 5F 6A 00 45 23 6C A7` |
| CAL 기준값 없음 | state 0, φ0 무효, gen 0, NVM_MISMATCH·CAPTURE_ACTIVE·DIRTY, sign 미정, boot_id 0xA7 | `00 FF FF 7F 00 00 13 A7` |

## 7. 버스 부하 (25 Hz, 앵커 1기, 500 kbps)

- 최악 비트 스터핑 기준 프레임 길이: DLC 4 = 95비트, DLC 8 = 135비트(최대 270 µs).
- 최악 부하: 레거시 0x201 25×95 + OBS 25×135 + CAL 1×135 = **5885 bps ≈ 1.18 %**.
- 스터핑을 뺀 공칭 부하: 25×79 + 25×111 + 111 = 4861 bps ≈ 0.97 %.
- 교환 1건(40 ms)의 최대 버스 점유: 레거시 + OBS + CAL = 365비트 = 0.73 ms.
- CAL이 변경 때문에 추가로 나가는 경우는 캡처나 저장을 할 때 몇 프레임뿐이다.

## 8. 기존 250 kbps 빌드 검증 기록 (2026-11-20, 보드 미플래시)

- 호스트 시험(MSYS2 gcc, `-Wall -Wextra -Werror`), `tests/verify_pdoa_if0.py`: 구성 4종(physical/lr/adc = 111, 110, 001, 000) PASS.
  - 확인 항목: 정확한 바이트, 무효값, 포화, reason 대응, 같은 교환 결합, i24, 세대 순환, sn 1000프레임 순환 해제, `product_pdoa_lr_reference()` 대조.
  - 정적 검사: P 문장 불변, 레거시 `can_tx_msg` 호출부 불변, main.c는 추가만 있고 기본값 0.
- 펌웨어 빌드 결과는 커밋 메시지와 완료 보고서에 적는다. 정리하면 다음과 같다.
  - IF0=0 빌드는 검증된 25 Hz 이미지 8cceff03과 바이트가 같다.
  - IF0=1 빌드는 성공했다.
  - 출하 빌드는 c96f7ef9…와 바이트가 같다.

## 9. 500 kbps 빌드 검증 (보드 미플래시)

- FDCAN 초기화 오브젝트에서 IF-0 연구 앵커 분주 17, IF-0 비활성 앵커·TAG·출하 분주 34를 확인했다.
- IF-0 호스트 패킹 4개 구성이 통과했고 UART P 행과 기존 CAN 송신 호출부 비교가 일치했다.
- 재현 빌드에서 IF-0 비활성 앵커, TAG, 출하 이미지가 각 기준 이미지와 바이트 단위로 일치했다.
- 500 kbps 실제 버스 송수신 검증은 별도 보드 적용 이후에 진행한다.
