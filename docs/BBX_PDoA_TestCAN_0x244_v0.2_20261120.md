# BotBox+ PDoA+TWR 단일 앵커 추종 실차 시험 — CAN 인터페이스 (HUB → 컨트롤 보드)

**실차 시험용 v0.2 — 본부장 검토 반영** · 2026-11-20 · 작성 BBX · 근거 지시 t42u, t43u · v0.1 대비 변경: seq 의미, rf_age 기준, HELD/REFERENCE 정의, 시간 값 표기(8바이트 배치는 그대로)
범위: HUB가 보내는 **0x244 프레임 1개**만 정의한다. 기존 0x240/0x241 계약은 바꾸지 않으며 0x245는 쓰지 않는다. **조향·감속·정지·재출발 정책은 컨트롤 보드 담당 개발자가 정한다.** 이 문서는 디코딩과 신선도 판단에 필요한 사실만 정한다.

## 1. 프레임 개요
| 항목 | 값 |
|---|---|
| 형식 | Classic CAN 2.0B, 표준 11비트 ID, **FD 아님** |
| 비트레이트 | **500 kbps** (개발용) |
| ID / DLC | **0x244** / 8 |
| 송신 주기 | 목표 25 Hz (40 ms). 주기는 보증하지 않음(§7) |
| 바이트 순서 | Little Endian |

## 2. 바이트 배치
| 바이트 | 필드 | 형식·단위 | 무효값 / 범위 |
|---|---|---|---|
| B0–1 | `dist_mm` | u16, mm, TWR 거리 | **0xFFFF** = 무효. 0–65534 |
| B2–3 | `phase_cdeg` | i16, 0.01°, 센터 기준 처리된 PDoA 상대 위상(부호 적용) | **0x7FFF** = 무효. −18000…+17999 (±180.00°) |
| B4 | `seq` | u8, **새 결과가 생성될 때만 +1**, 255 → 0 순환. 같은 결과 재송신은 같은 값 | — |
| B5 | `status` | 아래 비트 | — |
| B6 | `quality` | u8, 0–100, 관측 일관성 점수(확률 아님) | **0xFF** = 무효 |
| B7 | `rf_age` | u8, **이 결과를 뒷받침하는 실제 RF 관측**부터 이 프레임 송신까지 경과, **4 ms 단위** (10 = 40 ms) | **0xFF** = ≥ 1020 ms 또는 관측 없음 (0xFE = 1016 ms) |

**B5 `status`**
| 비트 | 이름 | 값 |
|---|---|---|
| b0–1 | `lat` | 0 UNKNOWN · 1 CENTER · 2 LEFT · 3 RIGHT |
| b2–3 | `src` | 0 NONE · 1 OBSERVED · 2 HELD · 3 REFERENCE (§4) |
| b4 | `DIST_VALID` | 1 = `dist_mm` 유효 |
| b5 | `LAT_VALID` | 1 = `lat`·`phase_cdeg` 유효 (**알고리즘 조건 충족일 뿐, 물리 검증 아님**) |
| b6 | `TAG_LOST` | 1 = 태그 RF 신호 상실 |
| b7 | `RESTARTED` | 1 = HUB 재부팅 또는 추정기 리셋 직후 구간(길이: 연구용 참고값 약 1 s). 이전 결과와 연속성 없음 |

## 3. 송신 측(HUB) 보장 사항
1. 거리와 좌우는 **독립**이다. DIST_VALID=1 ∧ LAT_VALID=0, DIST_VALID=0 ∧ LAT_VALID=1 모두 정상 조합이다.
2. `TAG_LOST=1` ⇒ DIST_VALID=0, LAT_VALID=0, lat=UNKNOWN, src=NONE, dist=0xFFFF, phase=0x7FFF.
3. `LAT_VALID=0` ⇒ lat=UNKNOWN, src=NONE, phase=0x7FFF.
4. `DIST_VALID=0` ⇒ dist=0xFFFF. 유효 여부의 기준은 **상태 비트**이고 무효값은 보조 표시다.
5. 정의되지 않은 값(phase가 0x7FFF가 아니면서 범위 밖, quality 101–254)은 송신하지 않는다. 수신되면 프레임 손상 또는 버전 불일치를 뜻한다.

## 4. src: OBSERVED / HELD / REFERENCE (E2 Core 기준)
기준: Codex E2 커밋 `e6e07c1`(브랜치 `codex/uwb-embedded-e1`, 2026-11-20 10:25 KST)¹. 
| src | 의미 | `rf_age` 기준 |
|---|---|---|
| OBSERVED | 현재 관측이 직접 뒷받침하는 결과 | 그 결과를 뒷받침한 관측 |
| **HELD** | **V2 유지 추정**: 새 관측이 방향을 직접 정하지 못할 때, 마지막으로 지지 조건을 충족한 관측의 방향을 **제한된 시간 동안 유지**한다. 유지 허용 시간은 Core 내부에서 80/160/240 ms(품질 저하/기본/안정)로 정하며, 넘으면 UNKNOWN이 된다² | 유지값이 나온 **마지막 지지 관측**. 유지 중에는 계속 커진다 |
| **REFERENCE** | **V3 이력 참조**: **현재 관측 군집**의 위상이 CENTER/측면 경계 근처(히스테리시스 구간)에 있을 때, 그 군집과 **이전 판단을 함께** 써서 방향을 정한다. 과거 방향을 그냥 유지하는 것이 아니며, 위상 값은 현재 군집 값이다³ | 현재 군집에 쓰인 **가장 최근 실제 관측** |
- 새 프레임 송신이나 이력 참조는 관측 시각을 갱신하지 않는다. 같은 결과를 재송신하면 `rf_age`만 커진다.
- 참고: Core의 V3 전용 빌드 프로파일(`UWB_V3_ONLY`)은 V2를 계산하지 않으므로 그 빌드에서는 HELD가 나오지 않는다⁴.

## 5. 부호·의미
- `phase_cdeg` **양수 = LEFT, 음수 = RIGHT**(센터 기준 상대 위상에 부호 −1 적용)⁵. **[미확인 — 실차 장착 후 재확인]**: 부호 −1은 책상 시험(11/06 AE)으로 정한 값이다.
- 상대 위상은 **물리적 방위각이 아니다**. lat 판정 문턱은 HUB Core 내부 값이며 이 인터페이스에 포함하지 않는다.
- **전후방 판단은 지원하지 않는다.** 태그가 뒤에 있어도 앞과 같은 값이 나올 수 있다.
- HUB 구현 메모: Core `UwbDirection`(0 UNK, 1 LEFT, 2 CENTER, 3 RIGHT) → `lat = {0,2,1,3}[dir]`, `phase_cdeg = round(100 × deviation_deg)`.

## 6. 시퀀스·신선도 (디코딩 사실)
- `d = (u8)(seq − 직전 seq)`: **d = 0 → 같은 결과의 재송신(오류 아님)** · d = 1 → 다음 새 결과 · 2–127 → 그 사이 d−1개 결과를 받지 못함 · 128–255 → 이전 결과가 늦게 도착한 것(역순).
- 결과의 나이는 `rf_age × 4 ms`로 알 수 있다. 프레임이 계속 와도 seq가 그대로이고 rf_age가 커지면 새 RF 관측이 없는 것이다.
- **연구용 참고값**(결정값 아님, 수신 측 개발자가 정함): 0x244 미수신 200 ms · RF 관측 공백 250 ms · HELD 상한 160 ms · RESTARTED 구간 약 1 s.

## 7. 패킷 예시 (스크립트로 인코딩 후 역디코딩 검증, 시간 순서대로)
| 상황 | dist | phase | seq | B5 | q | rf_age | 바이트 (B0…B7) |
|---|---|---|---|---|---|---|---|
| 정상 CENTER | 1500 mm | +2.50° | 0x10 | 0x35 (C, OBSERVED, DV, LV) | 85 | 40 ms | `DC 05 FA 00 10 35 55 0A` |
| LEFT | 2000 mm | +35.00° | 0x11 | 0x36 (L, OBSERVED, DV, LV) | 78 | 40 ms | `D0 07 AC 0D 11 36 4E 0A` |
| LEFT 재송신 (같은 결과) | 2000 mm | +35.00° | **0x11** | 0x36 | 78 | **80 ms** | `D0 07 AC 0D 11 36 4E 14` |
| RIGHT | 1800 mm | −30.00° | 0x12 | 0x37 (R, OBSERVED, DV, LV) | 80 | 40 ms | `08 07 48 F4 12 37 50 0A` |
| HELD LEFT | 2050 mm | +34.00° | 0x13 | 0x3A (L, HELD, DV, LV) | 45 | 120 ms | `02 08 48 0D 13 3A 2D 1E` |
| REFERENCE LEFT (경계 근처) | 2100 mm | +19.00° | 0x14 | 0x3E (L, REFERENCE, DV, LV) | 70 | 40 ms | `34 08 6C 07 14 3E 46 0A` |
| UNKNOWN (거리만 유효) | 1700 mm | 무효 | 0x15 | 0x10 (UNK, NONE, DV) | 30 | 40 ms | `A4 06 FF 7F 15 10 1E 0A` |
| LOST | 무효 | 무효 | 0x16 | 0x40 (TAG_LOST) | 무효 | 260 ms | `FF FF FF 7F 16 40 FF 41` |

## 8. 버스·ID 주의
- ID 0x244의 하위 니블 4는 출고 `321ccc1`의 좌/우/후 슬롯(니블 1/2/3) 디스패치에 걸리지 않는다⁶.
- HUB FDCAN 출력 포트는 현재 **FD+BRS 모드 500k**로 설정돼 있다 → **Classic 500k로 설정해야 함 [미확인 — 미구현]**.
- 컨트롤 보드 기준: 출고 `321ccc1` CAN3 = 250 kbps, kaist-vlm `aaa6ffc` CAN3 = 500 kbps. 어느 쪽을 기반으로 할지는 담당 개발자가 정한다.

## 9. 현재 기술적 한계
- LAT_VALID는 알고리즘 조건일 뿐 **물리적 좌우는 미검증**(본부장 판정 보류).
- 태그 자세 영향(PF18/PF19, 11/17)은 세션 원본만 있고 **문서화 전**.
- **25 Hz 실제 CAN 송수신은 미측정**(현행 출력 약 5 Hz).
- **HUB에 0x244 구현 없음**(현재 0x240/0x241만).
- 기존 3앵커와 PDoA 앵커의 **동시 레인징 성능 미측정**.
- **위상 부호(+ = LEFT)는 실차 미확인**.

## 부록. 제품화 보류 목록 (이번 시험 범위 아님)
IF-0/IF-1 전체 계약(DRAFT v2 보존) · 명세 v0.12 · 0x245 진단 프레임 · 처리 상태·DEGRADED·RANGE_JUMP·CENTER_CAL_VALID 비트 · 전후방 판단 · 출고 250k와 개발 500k 통일 · 하트비트·CFG·고장 코드 · Qmin/Hmax/타임아웃 확정 · 모드(단일 PDoA/3앵커/Hybrid) 전환 · 앵커 재부팅 감지 · 위상 부호 실차 확정.

---
근거 (uwb-visualization 저장소, `C:\Users\ryan\dev\embedded\UWB_STM32\uwb-visualization-e1`, `git show e6e07c1`; 작업트리 미커밋 변경은 인용하지 않음)
¹ `embedded/uwb_core/E2.md:1-3` (E2 = HUB G474 이식 준비, 기준 E1 `447c6d1`).
² `embedded/uwb_core/uwb_engines.c:315-337` (V2 유지: `grace_ms = quality_drop ? 80 : stable ? 240 : 160` `:321`, `origin = UWB_CONTINUITY_HOLD` `:329`, `held_ms = age` `:330`), 조회 시 만료 `uwb_core.c:310-312`.
³ `embedded/uwb_core/uwb_engines.c:371-412` (V1 결과에서 시작 `:373`, 현재 군집 편차로 판정 `:395-399`, 히스테리시스 구간에서 이전 판단 사용 `UWB_BOUNDARY_REFERENCE` `:400-402`, 위상 = 현재 군집 `:409`). E2.md:48 "V3의 현재 군집 + 이력 참조를 V2 HELD와 구별".
⁴ `embedded/uwb_core/E2.md:14`, `uwb_engines.c:258` `#if !UWB_V3_ONLY`.
⁵ 앵커 연구 `b858d92` `Core/Src/product_pdoa_lr.c:55-58`, `Core/Inc/product_pdoa_lr.h:11-19` (`PRODUCT_PDOA_LR_SIGN (-1)`); Core `uwb_math.c:14,27-33` 같은 방향.
⁶ botbox-stm32 `321ccc1` `Core/App/manager/uwb/uwb_manager.cpp:855-866`. 비트레이트: `321ccc1`/`aaa6ffc` `Core/Src/can.c:107`, HUB `14627cb` `Core/Src/fdcan.c:81`.
