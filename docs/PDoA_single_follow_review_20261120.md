---
title: 단일 PDoA 앵커 좌우 추종 — 컨트롤 보드 측 읽기 전용 검토
date: 2026-11-20
author: BBX (본부장 요청 t33u)
evidence_level: OBSERVED (코드·문서 읽기 전용, 보드·버스 실측 없음)
status: 제안(PROPOSAL) — 통신 계약은 UWB 담당 봇과 합의 전까지 확정·구현하지 않음
---

# 단일 PDoA 앵커 좌우 추종 검토 (2026-11-20)

표기: **[확인]** 코드·문서 file:line 또는 쪽 근거 · **[추정]** 근거로부터의 추론 · **[미확인]** 확인 못 함 · **제안** = 합의 전 초안 수치/형식.
모든 저장소는 `git show`/`git archive`(로컬 원격 추적 ref, 마지막 fetch 2026-11-20 08:40 KST)로만 읽었다. checkout·commit·push·브랜치 생성·보드 접근 없음.

## 0. 결론 (먼저)

1. **단일 PDoA 추종 모드는 "별도 결과 소스"로 추가 가능하다. 단, 붙일 곳은 kaist-vlm 계열의 IF-1 수신부(`UwbManager`/`ExternalFollow`)가 맞고, 출고 계열(origin/main)에는 그대로 붙이면 안 된다.** [확인·추정]
   - origin/main(`321ccc1`, v1.2.2, 제품 기준본 Set A와 동일)은 CAN3 모든 표준 ID를 받고 **ID 하위 4비트(1/2/3)로 좌·우·후 앵커 슬롯을 고른다** (`uwb_manager.cpp:855-866`). PDoA V2 `0x211–0x213`, `0x241`, 향후 하트비트 `0x701`은 모두 앵커 슬롯을 덮어쓴다. UWB 태스크도 190 ms 주기라 25 Hz를 살리지 못한다.
   - origin/kaist-vlm(`aaa6ffc`, 11/16)은 이미 **온보드 3앵커 알고리즘을 제거**하고 수집 보드(botbox-uwb-hub)의 `0x240`만 **정확한 ID 비교**로 받으며(`uwb_manager.cpp:116-125`), UWB 태스크 20 ms, ISR 수신 시각 기준 신선도, 새 seq 기준 600 ms 타임아웃 구조다. 같은 모양의 `PdoaFollow` 소스를 병렬로 두기 쉽다.
2. **기존 `UWB_RESULT_A 0x240`으로는 단일 PDoA 좌우 추종을 실을 수 없다.** direction 3/4(좌/우)는 "태그가 옆(MID)"이라는 뜻이고 kaist-vlm 수신부는 이를 **제자리 회전**으로 바꾼다(`uwb_external_follow.hpp:173-180`). bearing은 LUT 보정 통과 후에만 유효(명세 p7)이며 PDoA는 전후방을 보장하지 않는다. → **신규 프레임이 필요**하다(제안 ID `0x244`, §5).
3. **"Intelligent Adaptive UWB V3"는 현재 펌웨어가 아니라 PC 연구 도구(uwb-visualization, JavaScript)에만 있다** (`0fd4030`, 11/20 09:20 KST). 25 Hz 앵커 기준선도 N=1 TWR·UART 관측 PASS이며 **CAN 송신은 5 Hz 제한·PASS 범위 제외**다. 따라서 "수집 보드가 25 Hz로 좌우 결과를 CAN에 낸다"는 전제는 아직 어느 펌웨어에도 없다. [확인]
4. **버스 속도 불일치가 선결 과제다.** origin/main CAN3 250 kbps, kaist-vlm CAN3 500 kbps, 허브 FDCAN1·2 공칭 500 kbps(FD+BRS 설정), 명세·앵커 제품 250 kbps. [확인]
5. **안전 원칙(제안)**: 단일 PDoA 모드에서는 **후진·제자리 회전 금지, 전진+완만 조향만**, 오래된/HELD 초과 방향으로 조향 금지(직진 감속), 새 seq 200 ms 미수신·TAG_LOST·LOST는 정지, 비상정지·범퍼·초음파 경로와 최종 지령권은 컨트롤 보드에 그대로. 3앵커 경로는 한 줄도 바꾸지 않는다.

---

## 1. 근거로 쓴 자료 (경로·SHA-256·커밋)

### 1.1 참고 문서
| 문서 | 경로 (공유 PC) | SHA-256 |
|---|---|---|
| 01 레벨구조도 개정6 | `C:\Users\ryan\Desktop\0. 개발 문서\01_UWB_측위_펌웨어\모듈화\01_UWB_추종_모듈화_레벨구조도_개정6.pdf` (볼트 사본 동일 해시) | `47ecd17ea97661cdeb54cd34672c84809a40ecceb02946ac964fb20187b016e9` |
| 02 통신 인터페이스 명세서 v0.11 | `C:\Users\ryan\Desktop\0. 개발 문서\01_UWB_측위_펌웨어\모듈화\02_UWB_추종_모듈_통신_인터페이스_명세서_v0.11.pdf` (botbox-uwb-hub `docs/`·kaist-vlm `docs/`·볼트 사본 동일 해시) | `b0f33e56552c94751e065a01b7550825cb6bf8dc33e8d0b67d29080306aea61d` |
| if4_status_protocol.md (draft 1, 11/19) | `C:\Users\ryan\Desktop\0. 개발 문서\03_KAIST_모듈화_현장실사\if4_status_protocol.md` | `8680760633b208ba1cc40669cbbddc7e391cdf0605277914ac1119347a770c2f` |
| 제품 수치 기준본 v1.1 (읽기만) | `Desktop\Notepad\TheRobotics\02_프로젝트\봇박스제품\제품 수치 기준본.md` | `056ad53e2f8e8feae0fa074a2eee80cdef22984175774bdc9d2e2e14ba1e5b42` |
| UF1 A 조사 (재검증) | `C:\Users\ryan\projects\봇박스\UF1_UWB수집흐름_20261110\UF1_A_survey_20261110.md` | `37d3eb37749b4aaee5e5215af1f93a3fdbe358695bf27fb006f82d4187aa49c7` |

PDF는 `pdftotext -layout`로 추출, 쪽 번호는 PDF 쪽.

### 1.2 botbox-stm32 (`C:\Users\ryan\dev\embedded\botbox-stm32`, fetch 안 함)
| ref | SHA | 날짜(KST) | 비고 |
|---|---|---|---|
| origin/main | `321ccc16799cdb637690740213e3c08126c5fab0` | — | v1.2.2 (`Core/App/version.h`), 온보드 3앵커, 기준본 Set A "origin/main 동일" |
| origin/kaist-vlm | `aaa6ffc4f79bacd18914d1a68fa0e8f0dfa1c427` | 11-16 14:09 | **이전 알려진 `4a55c2d`에서 14커밋 전진**. `104b2c9` 온보드 UWB 제거·IF-1 0x240 수신, `d099678` CAN3 500 kbps |
| origin/feat/if4-nav-state | `9d32fbec9470f2a6bc64c00030b5e741ccb261fc` | 11-18 14:59 | kaist-vlm 위 1커밋: NAV_STATE_A/B, 0x240/0x241 IF-4 릴레이. "Not yet verified on hardware" |
| origin/feat/jetson-rec-v2 | `e9d7013f3a3c1a5f62cee82cc34cd1fa38dfc7a2` | — | Jetson 브리지 |
| origin/feat/jetson-if4-rx | `cc5ae6a39e58483757ed9ec45f0d64d95fef1cad` | — | 문서 |
| origin/feat/kaist-jetson-link | `29932259abd86fe1f3675b26d894fa8f9dd15f40` | — | 문서 |
| origin/jr-japan | `306bb5ace6f673a0db6091b774d36b259bdac6e3` | — | JR, 범위 밖 |
| 로컬 main | `d0677a6112593e50cae9d8e78c8225ff1ff8c21a` | 10-12 | origin/main 대비 ahead 23 / behind 48 (기준본 Set B 계열) |
| 로컬 feat/uwb-can-v2-consumer | `7b38467bf900beca5148769342e5f0dcb44ba50f` | 10-01 | PDoA V2 소비자, kaist-vlm의 조상 |
| `4a55c2d` (이전 kaist-vlm) | `4a55c2dc771b00bc6f9e1bddb320438d71a3ba8d` | — | UF1 A 조사 기준. origin/kaist-vlm의 조상 [확인 `merge-base --is-ancestor`] |
| 별도 클론 `botbox-stm32-kaist-rec` | feat/kaist-rec-trigger `d56011a`, kaist-vlm `321ccc1`(behind 28) | — | 참고만 |

### 1.3 기타 저장소
| 저장소 | ref / SHA | 비고 |
|---|---|---|
| botbox-uwb-hub 로컬 클론 `C:\Users\ryan\projects\KAIST\J2_if4_rx_20261110\botbox-uwb-hub` | main = origin/main = `14627cb466b7f52aabeafe4ac9e8fc13dcb80853` (HEAD detached 동일) | 수집 보드(G474) 펌웨어 |
| UWB_STM32/02_stm32-dw3000-g474re (앵커 제품) | HEAD `codex/hold-window-25hz` = main `a5400ef78cc3f110adec2b8be69dbf3379cb921d` (11-12) | `product_pdoa_can.*`, 25 Hz 기준선 |
| UWB_STM32/uwb-visualization | main `0fd403077da493c58cbea24ba68508588e457e0c` (11-20 09:20) | Intelligent UWB V1/V2/V3 (JS). 작업 트리에 미커밋 수정 다수(엔진 파일은 아님) |
| UWB_STM32 루트 | main `4a7cacd` | `NEXT_STEPS.md`: Physical Multi-Node CAN DEFERRED, PDoA tuning FROZEN, L4 actual motion BLOCKED |

---

## 2. 항목별 검토

### 항목 1. 현행 BotBox+ UWB CAN 수신 구조

**A. origin/main `321ccc1` (출고 계열)**
- 비트레이트 250 kbps [확인]: `Core/Src/can.c:106-111` CAN3 Prescaler 12, BS1 15 tq, BS2 2 tq(18 tq). `Core/Src/main.c:169-171,191-192` HSE 8 MHz/PLLM 4×PLLN 216/P 2 = 216 MHz, AHB /4, APB1 /1 → 54 MHz. 54 MHz/12/18 = 250 kbps [계산].
- 필터: ID/MASK 32비트, ID 0, 마스크 `0xFFF0<<5` → 레지스터 비트 31–21(STID)이 마스크에 들어가지 않아 **표준 ID는 전부 통과** [확인 `controller/can_controller.cpp:17-27`].
- ISR: `interrupt_callback_impl.c:145-161` CAN3 FIFO0 → `uwb_rx_cplt()`; `cpp_bridge.cpp:30-39`가 StdId·8바이트를 큐에 넣음. **IDE/RTR·DLC 검사 없음, 수신 시각 없음** [확인].
- 큐: `tasks.cpp:105` 깊이 10. UWB 태스크 `tasks.cpp:313-331`, 주기 `tasks.h:28` **190 ms**, 우선순위 11 [확인].
- 디스패치: `manager/uwb/uwb_manager.cpp:855-866` **`nodeID & 0x0F`가 1/2/3이면 좌/우/후 슬롯**. PDoA 분기 없음 [확인].
- 레거시 파서: `controller/uwb_can_controller.hpp:12-30`(0xAA, int16 거리, XOR), `uwb_can_controller.cpp:131-170` — start/checksum 불일치 → packetBroken, −1 → tagLost, 매 수신마다 `_lastRxTick` 갱신 [확인].
- 타임아웃: `receive_timeout_ms` 기본 **10000 ms** (`configuration/firmware_configuration.cpp:32`), 판정 `uwb_manager.cpp:95-150`(어느 한 센서라도 타임아웃이면 정지) [확인].
- **결과**: `0x211–0x213`(PDoA V2, start 0xAC)·`0x241`·`0x701`·`0x251` 등 하위 니블 1–3 프레임은 해당 앵커 슬롯을 **깨진 패킷으로 덮어쓰고 수신 시각까지 갱신**한다 → 좌/우/후 신호손실 판정 흔들림, 타임아웃 은폐 [확인 코드 경로 / 주행 영향은 추정].

**B. `4a55c2d` (UF1 A 조사 기준, 재검증)**
- PDoA 분기 `(id & 0xFFF0)==0x210` 후 나머지는 하위 니블 디스패치 [확인 `uwb_manager.cpp:962-979`; UF1 문서의 `963-979`는 1줄 차이, 내용 동일].
- PDoA V2 파서(0xAC, ver 2, phase_cdeg·distance·quality_r·seq) [확인 `uwb_can_controller.hpp:28,60-110`], PDoA 수신 타임아웃 750 ms (`:51`), 레거시 고정창 `UWB_RECEIVE_TIMEOUT 1300`·PDoA 전방근거 500 ms (`uwb_manager.hpp:18-19`), PDoA 조향 권한은 3센서 신선 + 전후차 ≤ −450 mm일 때만 (`uwb_pdoa_authority.hpp:26-50`) [확인].
- UF1 A의 "S2 금지(0x701·0x241이 좌측 슬롯 덮어씀)" 결론은 **4a55c2d·origin/main에 대해 여전히 유효** [확인]. 단 **origin/kaist-vlm `aaa6ffc`에는 해당 없음**(아래 C).

**C. origin/kaist-vlm `aaa6ffc` (개발 계열, IF-1 컨트롤 보드)**
- CAN3 **500 kbps** [확인 `Core/Src/can.c:107-111` Prescaler 6; `main.c:174-175,196-197` AHB /1, APB1 /4 → 54 MHz; 54/6/18 = 500 kbps]. 결정 근거: `docs/review/2026-11-10-uwb-migration-verification.md:10` "명세 250 kbps와 다르지만 사용자 결정".
- 필터는 그대로 전체 통과(`can_controller.cpp:17-27`), ISR에서 **표준 데이터 프레임만**, **ISR 수신 시각** 스탬프, 큐 가득 참 카운트 [확인 `cpp_bridge.cpp:36-61`].
- 디스패치: **`nodeID == 0x240`만** 처리, 나머지 무시 [확인 `manager/uwb/uwb_manager.cpp:116-125`]. 0x240 디코더는 예약값 거부 [확인 `controller/uwb_result_v011.hpp:100-132`].
- UWB 태스크 **20 ms** (`tasks.h:36`), 큐 10 (`tasks.cpp:137`), 루프 `tasks.cpp:345-362` [확인].
- 신선도: 새 seq만 타이머 갱신, 600 ms, 읽는 시점 재확인 [확인 `uwb_external_follow.hpp:28-39,92-120`, `uwb_manager.cpp:202-210`].
- feat/if4-nav-state `9d32fbe`: 같은 수신부에서 0x240/0x241을 IF-4로 릴레이 [확인 `uwb_manager.cpp:120-128`, `kaist/if4_link.cpp:658-663`].

**D. 수집 보드 botbox-uwb-hub `14627cb`**: 앵커 입력 `CAN_PORT_1`(`App/uwb_task.cpp:30`), 출력 `CAN_PORT_2`(`App/uwb_output.cpp:10`), 두 포트 공칭 500 kbps (`botbox-uwb-hub.ioc:79,95`, FDCAN 24 MHz `:332`), 0x240을 클래식 프레임으로 송신(`uwb_output.cpp:274-285`), 0x241 꺼짐(`:20`), **0x701 하트비트 송신 없음**(app_main 하트비트는 LED 토글, `App/app_main.c:67-84`) [확인]. PDoA 입력·처리 코드 없음 [확인: `App/` 내 pdoa 파서 없음, BEARING_SRC 항상 0 `uwb_output.cpp:38`].

### 항목 2. 기존 3앵커 추종 알고리즘 (origin/main `321ccc1` 기준, hub `14627cb`에 이식본 있음)

- **신호 판정** `uwb_manager.cpp:95-150`: 좌·우·후 타임아웃(10 s) 하나라도 → `SENSOR_TIMEOUT` 정지. tagLost/broken/invalid → signalLost.
- **모드 결정** `decide_mode` `:252-313`: L·R 정상 → DEFAULT(후방 정상) / FRONT_ONLY(후방 손실); L·R 손실·B 정상 → BACK_ONLY; L+B → LEFT_BACK; R+B → RIGHT_BACK; L만 → LEFT_ONLY; R만 → RIGHT_ONLY; 3개 손실 → STOP. 후방 UWB 끔이면 FRONT_ONLY 고정(`:155-166`).
- **전환 히스테리시스** `apply_mode_transition` `:339-426`: 버퍼 7 중 4 (`firmware_configuration.cpp:34-35`), 관찰 1500 ms(`:36`) 후 현행 유지/최신 채택.
- **거리**: 전방 = √((L²+R²)/2 − d²/4), d=400 mm (`uwb_manager.cpp:448-460`, `firmware_configuration.cpp:39`). LPF 기본 꺼짐(`firmware_configuration.cpp:33` `distance_filter_enabled 0`; 필터 함수 `uwb_can_controller.hpp:69-75`).
- **방향** `calculate_direction` `uwb_move_maker.cpp:143-161`: |front−back| < MID 280(후진 중 140) → MID, > 450 → FORWARD/BACKWARD, 사이 = 이전 유지.
- **좌우** `calculate_turn` `:31-43`: |L−R| < 100 CENTER, > 150 좌/우, 사이 유지. `calculate_back_turn` `:45-57` 동일 100/150.
- **회전비** `update_turn_ratio_from_angle` `:81-141`: 각도 EMA α0.40·스텝 15°, 데드존 5°, 풀스케일 60°, 비율 EMA α0.35, 방향 전환 시 250 ms 동안 100% (`firmware_configuration.cpp:65-76`).
- **MOVE/STOP** `calculate_move` `:191-282`: 속도단별 시작/근접정지 1250/1500/1750 · 1000/1250/1500, 후진 1500/1750/2000 · 1250/1500/1750, 상한 8500/8750, MID는 ×3/2. 3샘플 다수결 + STOP→MOVE 즉시 재개(`:166-185`).
- **apply_*** `uwb_manager.cpp:506-672`. LEFT/RIGHT_ONLY는 최근 신뢰 움직임(`single_only_history_timeout_ms` 1700 ms, `firmware_configuration.cpp:37`)이 그럴듯할 때만 진행, 아니면 **최근 움직임 유지** (`keep_recent_motion` `:736-749`). → **오래된 방향으로 최대 1.7 s 조향 가능한 구조** [확인]. 단일 PDoA 모드에서는 이 패턴을 쓰지 않아야 함(요구사항 "오래된 관측 조향 금지").
- **출력** `output_manager.cpp:861-913`: FORWARD+turn → 전진/전진좌/전진우, BACKWARD+backTurn(좌↔우 반전 매핑), MID → 제자리 회전. 초음파 정지는 **패널 SAFETY 모드에서만** (`:870-883`), `US_AVOIDANCE` 비활성(`:5`). 거리별 속도 상한 1500 mm→1단, 3000 mm→2단(`:1000-1032`, `firmware_configuration.cpp:94-95`). 제자리 회전 속도 = 속도단값 × 100/77/52 % (`output_manager.cpp:27-36,1058-1063`, `firmware_configuration.cpp:80-82,91-93`) → 1500 / ≈2117 / ≈2129 카운트. **"제자리 회전은 1단만 다르고 2·3단은 같다"는 공유 사실과 일치** [확인·계산].
- **정지 램프**: UWB STOP은 브레이크가 아니라 감속 램프(1단 50, 2·3단 100 카운트/20 ms) [확인 `output_manager.cpp:1181-1220`, `firmware_configuration.cpp:87-89`] → 3단 4095에서 0까지 약 820 ms [계산].
- "1300 ms validity"는 origin/main이 아니라 `4a55c2d` 고정창(`uwb_manager.hpp:18`)과 명세 "현행 1300 ms"(p13·p15) 표기다. 출고 기본값은 10 s, 기준본 표의 "0.3/1.3/10 s"(기준본 §1 UWB 신호 끊김 행)와 일치 [확인].

### 항목 3. 단일 PDoA 추종 모드 추가 가능성

**판단: 가능(kaist-vlm 계열에 소스 분리형으로). origin/main에 직접 추가는 비권장.** [추정, 근거 아래]

| 기준 | origin/main | kaist-vlm `aaa6ffc` |
|---|---|---|
| ID 디스패치 | 하위 니블 → 새 프레임이 앵커 슬롯 오염 위험 | 정확한 ID 비교 → 새 ID 추가 안전 |
| 25 Hz 처리 | 190 ms 태스크(25 Hz × 0.19 s ≈ 4.75 프레임/주기/ID) | 20 ms 태스크, 새 seq마다 1회 처리 |
| 신선도 | 슬롯별 마지막 수신 tick, 10 s | ISR 시각·새 seq·읽기 시점 재검사 |
| 3앵커 | 온보드 | 수집 보드(hub)가 계산해 0x240 송신 |
| 버스 | 250 kbps | 500 kbps (앵커와 불일치 위험) |

**훅 위치 (제안, 구현 안 함)**
1. 순수 로직 헤더 `manager/uwb/uwb_pdoa_follow.hpp` 신설 — `uwb_external_follow.hpp`와 같은 형태(HAL 없음, 시간 인자). 디코드·새 seq 신선도·정책 상태기계(§7·§8)를 담고 `ExternalFollowInput` 호환 구조를 낸다.
2. `UwbManager`에 소스 선택 1개만 추가: `follow_source = THREE_ANCHOR_IF1(기본) | PDOA_SINGLE`.
   - 컴파일 게이트 `BOTBOX_UWB_PDOA_SINGLE_FOLLOW_ENABLE` 기본 **0** + 설정 필드(기본 0). 둘 다 켜져야 동작.
   - **AUTO 진입 시점에만 소스 확정**, 주행 중 전환 거부(현 `AutoSourceSelector`의 REJECTED_MOVING 패턴과 동일 원칙).
   - `on_receive`: `0x240`은 기존 경로 그대로, 신규 ID는 PDoA 객체로. `update`: 소스가 PDOA일 때만 PDoA 평가, 아니면 기존 코드 경로 무변경.
3. `OutputManager`는 기존 getter(`get_direction/turn/turn_ratio/move`, `front_distance`)만 소비하므로 수정 최소. PDoA 모드는 **FORWARD만** 내고 BACKWARD·MID를 내지 않으므로 후진 추종·제자리 회전이 구조적으로 불가 [추정].
4. 회피 로직용 좌우 거리 차는 kaist-vlm에서 이미 0 처리(`uwb_manager.hpp:55-56`) → PDoA 모드도 동일.
5. origin/main에 꼭 넣어야 한다면 선행 조건: 디스패치를 정확 ID(0x201/0x202/0x203) 비교로 바꾸고 UWB 태스크 주기 분리 — **기존 출고 동작 변경이라 별도 승인 대상** [추정].

**주의 [확인]**: kaist-vlm은 온보드 3앵커를 제거했다(`docs/review/2026-11-10-uwb-migration-verification.md:36`). 이 계열에서 "3앵커 유지"는 허브의 0x240 경로 유지를 뜻한다. 또 허브 2센서(BEARING_VALID=0)에서는 kaist-vlm이 **직진만** 한다(같은 문서 `:26`).

### 항목 4. UWB_RESULT_A 0x240 / B 0x241 재사용 범위

- 0x240 필드(명세 p6): distance u16, bearing i16(0.01°, 정면 0·좌 +, 미보정 0x7FFF), direction(0 미정·1 전방·2 후방·3 좌측·4 우측), quality 0–100, status(b0 VALID, b1 BEARING_VALID, b2 TAG_LOST, b3 DEGRADED, b4 BEARING_SRC, b5–7 ANCHOR_MASK), seq. **배치는 v0.6 이후 고정**(p7) [확인].
- **재사용 가능한 것**: distance + VALID + TAG_LOST + seq (거리·생존 신호). 이것만으로는 좌우 추종 불가.
- **재사용하면 안 되는 것** [확인·추정]:
  - direction 3/4: 의미가 "태그가 옆(MID)"이고 kaist-vlm은 제자리 회전으로 매핑(`uwb_external_follow.hpp:173-180`). PDoA LEFT(센터 대비 위상 부호)와 의미가 다르다.
  - direction 1(전방): PDoA는 전후방을 보장하지 않으므로 넣을 근거가 없다(명세 p7 "PDoA 위상은 각도가 아니라 부호 있는 투영값"; V3 화면 문구 "센터 기준 상대 위상 · 전후방 식별 아님" `src/pdoa_intelligent_engine.js:145`).
  - bearing + BEARING_SRC=1: LUT 보정 통과 후에만(p7). 현재 PDoA는 물리 방위각 미보장.
  - 결론: 단일 PDoA를 0x240으로 내면 direction=0(UNKNOWN)이어야 하고, kaist-vlm은 **즉시 정지**(`uwb_external_follow.hpp:214-216`) → 추종 불가.
- 0x241(태그 좌표)도 BEARING_VALID 전제(p7) → 재사용 불가.
- **알려진 충돌** [확인]:
  - 하위 니블: `0x241`·`0x701`(=0x700+노드1, 명세 p11)·`0x211–0x213` → origin/main·4a55c2d 좌/우/후 슬롯 오염(위 항목 1).
  - status 비트 배치: v0.6(`4a55c2d controller/uwb_result_contract.hpp:22-29`: b2 DEGRADED, b3 TAG_LOST, b4–6 앵커) vs v0.11(`kaist-vlm controller/uwb_result_v011.hpp:26-32`, hub `uwb_output.cpp:34-42`: b2 TAG_LOST, b3 DEGRADED, b4 BEARING_SRC, b5–7 앵커). kaist-vlm에서 v0.6 코드는 삭제됨.
  - IF-4 릴레이는 0x240/0x241만 통과(`if4_link.cpp:658-663`), 명세 규칙은 0x240·0x250만(p10 규칙 1). 신규 ID는 릴레이되지 않음.

### 항목 5. 신규 PDoA 추종 결과 패킷 — **제안(PROPOSAL, 합의 전)**

**후보 ID: `0x244` `UWB_PDOA_FOLLOW_A`** (선택 진단 프레임 `0x245` `UWB_PDOA_FOLLOW_B`)
- 하위 니블 4/5 → origin/main·4a55c2d 슬롯 디스패치(1–3)에 걸리지 않음 [확인 `uwb_manager.cpp:858-865`].
- kaist-vlm은 정확 비교라 무시 [확인 `uwb_manager.cpp:117`].
- 명세 예약 ID(0x201–0x203, 0x210–0x21F, 0x240, 0x241, 0x250)와 겹치지 않음 [확인 p5]. 0x242/0x243은 하위 니블 2/3이라 **피함**.
- 0x240보다 우선순위 낮음(ID 큼). D3PC 0x301보다 높음 — 25 Hz × 1프레임이라 영향 미미 [추정].

**DLC 8, 리틀 엔디언 (제안)**
| 바이트 | 비트 | 필드 | 값 | 수신 측 규칙(제안) |
|---|---|---|---|---|
| 0–1 | — | `distance_mm` u16 | TWR 필터 거리, 0–65534, `0xFFFF` 무효 | VALID=1일 때만 사용 |
| 2 | 0–3 | `lat_class` | 0 UNKNOWN · 1 LEFT · 2 CENTER · 3 RIGHT (센터 기준 상대 위상, 물리 방위 아님) | 4–15 → 프레임 거부 |
| 2 | 4–7 | `proc_state` | 0 LOST · 1 RECOVERY · 2 DEGRADED · 3 ROBUST · 4 NORMAL · 5 FAST | 6–15 → 거부 |
| 3 | — | `quality` u8 | 0–100 관측 품질 점수 (확률 아님) | >100 거부 |
| 4 | 0–3 | `track_state` | 0 NONE · 1 TRACKING · 2 SOFT_HOLD · 3 DEGRADED_TRACKING · 4 RECOVERY · 5 LOST | 6–15 거부 |
| 4 | 4–5 | `lat_source` | 0 NONE · 1 OBSERVED(이번 관측) · 2 HELD(제한 유지 추정) · 3 예약 | 3 거부 |
| 4 | 6–7 | `front_back` | 0 UNKNOWN · 1 FRONT_CONFIRMED · 2 BACK_CONFIRMED · 3 예약 | 단일 PDoA는 항상 0 예상. 1이 오려면 근거 정의 합의 필요 |
| 5 | — | `lat_age_5ms` u8 | lat_class를 뒷받침한 **마지막 실제 관측**부터 송신까지 경과, 5 ms 단위. 0xFE 포화(≥1270 ms), 0xFF 없음 | HELD 상한 판정에 사용 |
| 6 | 0 | `VALID` | 거리 사용 가능 | |
| 6 | 1 | `LAT_VALID` | 수집 보드가 lat_class를 조향에 써도 된다고 판단 | 0이면 조향 안 함 |
| 6 | 2 | `TAG_LOST` | 태그 상실 | 1이면 즉시 정지 |
| 6 | 3 | `DEGRADED` | 품질·일관성 저하 | 속도 상한 |
| 6 | 4 | `CENTER_CAL_VALID` | 센터(0점) 보정 로드됨 | 0이면 LAT_VALID도 0이어야 함 |
| 6 | 5 | `RANGE_JUMP` | TWR 급변 감지 | 조향 보류 |
| 6 | 6–7 | 예약 | 0 | 0 아니면 거부(또는 무시 — 합의) |
| 7 | — | `seq` u8 | 새 결과마다 +1 (25 Hz) | 새 seq만 신선도 갱신, 공백 카운트 |

선택 `0x245` (주행 미사용, IF-4·로그용, 제안): `deviation_cdeg` i16(센터 대비 위상 편차), `sigma_cdeg` u16, `rule_set_id` u8(예: IAU-V1-20261119 → 번호), `raw_lat`/`v1_lat` u8, 예약, `seq`(A와 동일).

설계 근거: Intelligent UWB 출력 6가지(거리·LEFT/CENTER/RIGHT/UNKNOWN·품질·추적 상태·관측/유지 구분·처리 상태)를 1:1로 담고, 컨트롤 보드가 "오래된·불신 방향 금지"를 스스로 판정하도록 **age·source·LAT_VALID를 분리**했다. 명세의 "유효성은 상태 비트로만, age는 시점 정보" 원칙(p2)과의 관계는 합의 사항 — 이 제안은 **age를 조향 허용 상한 판정에도 쓰는 예외**를 둔다(유지 추정의 신선도는 status 비트로 표현이 안 되므로).

### 항목 6. 25 Hz 결과 수신과 제어 주기

| 항목 | origin/main | kaist-vlm | 근거 |
|---|---|---|---|
| Output(최종 지령) 주기 | 20 ms | 20 ms | main `tasks.cpp:344-351`; kvlm `tasks.cpp:384` |
| UWB 태스크 | 190 ms | 20 ms | main `tasks.h:28`; kvlm `tasks.h:36` |
| 입력(GPIO) | 30 ms | 30 ms | `tasks.cpp:309` / kvlm `:341` |
| 초음파 | 60 ms | 60 ms | `tasks.cpp:340` / kvlm `:372` |
| RTOS tick | 1 kHz | — | main `FreeRTOSConfig.h:66` |
| UWB 큐 | 10 | 10 | main `tasks.cpp:105`; kvlm `:137` |

- 25 Hz = 40 ms. kaist-vlm: 수신 → UWB 태스크(≤20 ms) → Output(≤20 ms) ≈ **≤40 ms** [추정, 이주 문서 `:32`와 같은 계산]. origin/main: 최대 190+20 ms, 25 Hz 결과 5개 중 4개는 처리되지 않고 덮어씀 [추정].
- **큐 깊이 10 (origin/main 190 ms)**: 0x244 25 Hz + PDoA V2 25 Hz + 레거시 거리 25 Hz면 190 ms당 ≈14프레임 → 넘침 위험 [추정]. kaist-vlm 20 ms면 주기당 ≤2–3 [추정].
- **CAN 부하**: 8바이트 표준 프레임 ≈135 bit(스터핑 최악 포함) 가정 시 25 Hz 1개 = 3.4 kbps(250k의 1.35 %), 25 Hz 3개 = 10.1 kbps(4.1 %), 기존 5 Hz 4개 = 2.7 kbps [계산]. 명세 설계 한도 50 %/버스(p4) 대비 여유 충분.
- **필터 재튜닝 필요** [추정]: kaist-vlm의 이동 버퍼(3샘플)·회전비 EMA는 "결과 1개당 1회" 진행(`uwb_manager.cpp:147-153`)이고 190 ms 기준으로 튜닝됨. 25 Hz면 3샘플 = 120 ms(현행 570 ms). 같은 시간상수를 원하면 α0.35 → ≈0.087, α0.40 → ≈0.10 [계산]. PDoA 모드는 연속 각도가 없으므로 회전비는 고정값 사용 제안(§7).
- **앵커·허브 현실** [확인]: 25 Hz 실기 PASS는 1TAG·1ANCHOR TWR·UART 관측 범위이고 "실제 CAN 송신 성능 … 제외" (`docs/validation/uwb_ch9_25hz_rr_n1_baseline_20261112.md`, PASS 범위 단락). 프리셋은 "25Hz / CAN 5Hz", 200 ms CAN 제한 (`CMakePresets.json:69-71,83-85`). 허브 결과 주기는 190 ms(`App/configuration/firmware_configuration.hpp:45`).

### 항목 7. 일시적 UNKNOWN 감속·정지 정책 — **제안 수치 [추정]**

참고 실측(연구 로그, 실주행 아님) [확인 `docs/PDOA_BOUNDARY_RESEARCH.md`, uwb-visualization `0fd4030`]: 3,995행(약 160 s, 25 Hz) 중 UNKNOWN V1 20.18 % → V3 6.73 %, LEFT→UNKNOWN→LEFT 89 → 18회. 연구 규칙: lostGap 250 ms, 재획득 3표본·60 ms, 반전 확인 80 ms, 유지 grace 160 ms(최대 240 ms) (`src/pdoa_intelligent_engine.js:4`, `src/pdoa_tracking_continuity.js:4`). "관측 점수는 정답 확률 아님 · LOS/NLOS 미검증" (`pdoa_intelligent_engine.js:63`).

컨트롤 보드 정책(제안, 단일 PDoA 모드 전용):
| 상황 | 동작 | 제안 값 |
|---|---|---|
| 조향 허용 조건 | VALID ∧ LAT_VALID ∧ CENTER_CAL_VALID ∧ ¬RANGE_JUMP ∧ 새 seq ∧ quality ≥ Qmin ∧ (source=OBSERVED ∨ (source=HELD ∧ lat_age ≤ Hmax)) ∧ proc_state ∈ {FAST, NORMAL, ROBUST} | Qmin 40 [추정], Hmax **160 ms** |
| CENTER | 직진 FORWARD | — |
| LEFT/RIGHT | FORWARD_LEFT/RIGHT, **고정 회전비** | 내륜 70 % (완만) [추정] |
| 방향 반전 L↔R | 연속 2개 새 seq 동일 반대 방향일 때만 반영, 그 사이 직진 | 80 ms (25 Hz 2프레임) |
| UNKNOWN 또는 조향 불허 (거리 VALID, 새 seq 있음) | **조향 즉시 중립(직진)**, 속도 1단 상한으로 감속, 이전 방향 유지 금지 | 0–400 ms "직진 감속" 구간 |
| UNKNOWN 지속 | 감속 정지(일반 램프) | 400 ms 초과 시 STOP [추정] |
| 재출발 | 같은 lat로 OBSERVED 새 seq 3개 연속 + 최소 정지 유지 | 3프레임(≈120 ms), 정지 최소 300 ms [추정] |
| proc_state DEGRADED/RECOVERY | 조향 안 함(직진 감속 규칙 적용) | — |
| DEGRADED 비트 | 속도 1단 상한 | — |

근거: "부분 차폐에서 불필요한 정지·출발 최소화" → 짧은 UNKNOWN은 정지 대신 **직진 감속**으로 흡수; "오래된·불신 방향 조향 금지" → 조향은 OBSERVED 또는 160 ms 이내 HELD만; 기존 `keep_recent_motion` 1.7 s 유지(`uwb_manager.cpp:736-763`) 패턴은 PDoA 모드에 사용 금지.
리스크 [추정]: 태그가 실제로 옆/뒤로 돌아간 상태에서 직진 감속 400 ms 동안 최대 ≈0.11 m(1단 1.0 km/h, 기준본 §1) 진행. 1단 외 속도 시험 금지 권고.

### 항목 8. TAG_LOST·통신 단절 안전 정지 — **제안**

| 조건 | 동작 | 제안 값 | 근거 |
|---|---|---|---|
| TAG_LOST=1 또는 proc_state=LOST 또는 track_state=LOST | 즉시 STOP(새 seq 기다리지 않음, 중복 seq에도 반영) | 0 ms | kaist-vlm 선례 `uwb_external_follow.hpp:197-218`(판정 순서) |
| 새 seq 미수신 | STOP + 진단 비트 | **200 ms**(25 Hz 5프레임) [추정] | 명세 p13 "1300 ms보다 단축, 후보 300 ms, 최종 체크 6" |
| 같은 seq 반복 | 타이머 갱신 안 함 → 200 ms 후 정지 | — | `uwb_external_follow.hpp:101-110` |
| 프레임 거부(예약값) | 무시 + 카운트, 신선도 갱신 안 함 | — | `uwb_result_v011.hpp:97-132` 패턴 |
| 판정 시점 | Output 읽기 시점에 신선도 재확인 | — | `uwb_manager.cpp:202-210` |
| 수집 보드 하트비트(0x701, 명세 p11) | 단일 PDoA 모드의 정지 판단에는 **사용하지 않음**(seq 타임아웃이 더 짧음). 500 ms 미수신 = 노드 오프라인 진단만 | 500 ms | 명세 p13 |
| 링크 단절 2 s 이상 | AUTO 재진입 전까지 래치(자동 재출발 안 함) [제안] | 2000 ms | — |
| 정지 방식 | TAG_LOST·통신 단절은 일반 감속 램프보다 빠른 정지 필요 검토(현재 3단 램프 ≈820 ms) | 실차 제동 시험으로 결정 | `output_manager.cpp:1181-1220` |
| 부저 | 기존 uwb_lost_buzzer_delay 1000 ms 재사용 | — | main `firmware_configuration.cpp:113`, kvlm `:83` |

기존 값과의 관계: 출고 10 000 ms(`firmware_configuration.cpp:32`), 개발 고정창 1300 ms(`4a55c2d uwb_manager.hpp:18`), PDoA 750 ms(`4a55c2d uwb_can_controller.hpp:51`), kaist-vlm 600 ms(`uwb_external_follow.hpp:32`, 5 Hz 3주기). PDoA 모드는 **자체 타임아웃**을 가지며 3앵커 설정값을 상속하지 않는다. 정지 전 이동 ≈ 속도×타임아웃: 1단(1.0 km/h ≈ 0.28 m/s) 200 ms → ≈0.06 m, 3단(2.4 km/h ≈ 0.67 m/s) 200 ms → ≈0.13 m + 램프 [계산, 속도는 기준본 §1 📏 실측 필요 값].

### 항목 9. IF-4로 Jetson 전달 — **제안**

현황 [확인]:
- 프레임: UART4 921600 8N1, `A5 5A | VER | MSG_ID LE | LEN | PAYLOAD | CRC-16/CCITT-FALSE`, VER 1 = LEN 0–8, VER 2 = LEN 0–64 (`if4_status_protocol.md:27-38`; 코드 `feat/if4-nav-state kaist/if4_frame.hpp:11-33`).
- 사용 ID: 0x702 HB(10 Hz), 0x704 Jetson HB, 0x420 OPS_STATE, 0x430 REC_STATUS, 0x410/0x411 NAV(50 Hz), 0x240/0x241 릴레이(AUTO에서만) (`if4_status_protocol.md:58-73`; `kaist/if4_msgs.hpp:10-18`).
- 0x422 CTRL_STATUS(draft, VER 2 LEN 48)에 follow 필드: uwb_result_reason(34), follow_stop_reason(35), follow 비트(36), follow_front/back_mm(38–41), turn_ratio(42) (`if4_status_protocol.md:156,179-185`). **문서 상태: "Firmware not yet implemented"** (`:3`).
- 릴레이 필터는 0x240/0x241만 (`if4_link.cpp:658-663`).

제안:
1. **0x244(·0x245)를 같은 MSG_ID로 그대로 릴레이**, VER 1 LEN 8, 1순위 큐(0x240과 같은 규칙: 수신 후 루프 1회 안). 부하 25 Hz × 16 B ≈ 0.4 kB/s (기존 ≈2 kB/s, 여유 92 kB/s; `if4_status_protocol.md:87`) [계산].
2. **컨트롤 상태**: CTRL_STATUS `layout 2`로 필드 추가 또는 신규 `0x426 PDOA_FOLLOW_CTRL`(VER 2, 제안 MSG_ID). 내용: follow_source(3앵커/PDoA), 정책 상태(FOLLOW/STRAIGHT_COAST/SOFT_STOP/STOP/LATCHED), 사유 코드, 적용한 0x244 seq, 적용 시점 lat_age, 적용 lat·회전비, 속도 상한, 재출발 카운트·stop/go 카운트. 결정 변화 시 + 2 Hz.
3. NAV_STATE_A `uwb_seq/uwb_age`는 명세상 0x240 기준(p9). PDoA 모드에서 0x244 seq를 가리키게 할지, 0xFF 유지할지 Jetson 팀 합의 필요.
4. UWB는 AUTO에서만 수신·릴레이(`if4_status_protocol.md:17,290-309`) — 벤치 S0/S1에서 수동 모드 관측이 필요하면 별도 요청 사안.

### 항목 10. 실차 시험 최소 개발 범위 — **제안**

| 단계 | 내용 | 켜는 것 | 꺼 두는 것 | 합격 기준(제안) |
|---|---|---|---|---|
| P0 계약 | UWB 봇과 §11 질문 합의(ID·배치·주기·비트레이트·앵커 주소·V3 실행 위치) | — | 구현 전부 | 서면 합의 |
| P1 호스트 | `uwb_pdoa_follow.hpp` 순수 로직 + `tests/uwb_if1` 형식 호스트 테스트(골든 프레임, UNKNOWN 버스트, HELD 경계 160/161 ms, 반전, 200/201 ms 타임아웃, 중복 seq, seq 랩). 연구 로그(3,995행 `estimates.jsonl`) 오프라인 재생으로 stop/go 횟수 산출 | 호스트만 | 펌웨어 빌드 반영 | 테스트 PASS, 오래된 방향으로 조향 0건 |
| S0 버스 관찰 | 벤치: 수집 보드가 0x244 25 Hz 송신, USB-CAN으로만 수신·기록 | 수집 보드 | 메인보드 연결 | 주기 40±5 ms, seq 공백율, ID 충돌 없음 |
| S1 그림자 | 메인보드 PDoA 모드 빌드 + **UWB 거리 확인 모드**(AUTO 판단만, 모터 지령 안 냄: main `output_manager.cpp:582-583,1267-1270`, kvlm `:638,1349`) + UART7 로그 `[PDOA-SH]` | 판단·로그 | 모터 지령 | 판단 지연 ≤40 ms, 정책 상태 전이 로그 |
| S2 바퀴 띄움 | D3PC 0x301/0x302 캡처, 바퀴 띄운 상태 | 모터(공중) | 지면 주행 | 정지 ≤ 타임아웃+40 ms, 좌우 부호 일치 |
| S3 실차 | 1단 속도, 직선 통로, E-stop 관찰자, 패널 **SAFETY 모드**(초음파 정지가 SAFETY에서만 동작 `output_manager.cpp:870-883`) | 1단 전진 추종 | 2·3단, 후진, 제자리 회전, US_AVOIDANCE, 0x241 | 반대 방향 조향 0, 분당 stop/go 횟수, 차폐 시 정지 지연 |

항상 유지: 3앵커 경로 무변경, ESTOP·범퍼(`tasks.cpp:281-291` main / `:313-322` kvlm 인터럽트 즉시 정지), 초음파, 최종 지령권(OutputManager 우선순위 — 명세 p13 순서) [확인]. 실차 전 PF9·범퍼 사전 점검(UWB_STM32 `NEXT_STEPS.md` WP34 기록 참조).

---

## 11. UWB 담당 봇과 합의해야 할 질문
1. **V3 실행 위치**: 현재 JS(PC)에만 있음. 수집 보드 펌웨어(허브 G474? 앵커 자체?) 이식 주체·일정은?
2. **물리 구성**: 단일 PDoA 앵커가 기존 3앵커를 대체하나, 4번째 노드로 추가되나? 추가라면 그 앵커의 레거시 거리 ID(`0x200 + short_addr&0x0F`, 02 `Core/Src/main.c:8047`)와 PDoA V2 ID(0x211–0x213)가 좌/우/후 슬롯과 겹치지 않게 주소 배정 필요.
3. **비트레이트**: 250(명세·앵커·origin/main) vs 500(kaist-vlm·허브). 시험 버스 하나로 통일 필요.
4. **패킷**: ID 0x244/0x245 수용 여부, §5 비트 배치, 예약값 처리(거부/무시), seq 규칙(새 결과마다 +1, 중복 송신 시 동일 seq).
5. **주기·지연**: 25 Hz CAN 송신 실제 가능 여부(현 프리셋 CAN 5 Hz), 관측→송신 상류 고정 지연(명세 CFG 0x0005) 값.
6. **LAT_VALID 정의**: 수집 보드가 어떤 조건에서 1로 두는가(CENTER 보정 로드, 품질 하한, 반전 확인 등). 컨트롤 보드 조건과 이중화 범위.
7. **HELD 상한**: 수집 보드 grace(160/240 ms)와 컨트롤 보드 Hmax(160 ms 제안) 일치 여부, lat_age 정의(마지막 실제 관측 기준).
8. **front_back**: 단일 PDoA에서 FRONT_CONFIRMED를 낼 근거가 있는가(없으면 항상 0). 거리 추세(전진 중 거리 증가 → 후방 의심) 같은 보조 근거를 수집 보드가 줄지, 컨트롤 보드가 볼지.
9. **TAG_LOST 정의**: TWR 실패 몇 회/몇 ms에서 1로 하는가, LOST(proc)와의 관계.
10. **quality 스케일**: 0–100 의미와 권장 하한.
11. **하트비트 0x701**: 송신 시작 시점(origin/main 슬롯 오염 문제, UF1 A S2 금지와 연결).
12. **IF-4**: Jetson 팀과 0x244 릴레이·NAV `uwb_seq` 의미 합의 (BBX ↔ Jetson 별도).

## 12. [미확인] 목록
- 오늘 시험할 BotBox+ 실차에 올라간 펌웨어 버전·설정(기준본 §4 "출고 실제 펌웨어 버전" 미확인과 동일). origin/main인지 kaist-vlm 계열인지.
- 실차 CAN3에 연결된 앵커들의 실제 비트레이트(250/500)와 주소.
- 단일 PDoA 앵커 장착 위치·안테나 방향, 센터(0점) 보정 상태(V3 연구 센터 27.231°는 연구 구간 값).
- 허브(`14627cb`)가 PDoA·25 Hz·0x244를 낼 계획/일정, 허브 보드가 CAN1만인지 CAN1·CAN2 모두인지(UF1 A 조사 미확인 유지).
- 25 Hz CAN 실송신 성능(02 기준선 PASS 범위 밖), 물리 다중 노드 CAN(`NEXT_STEPS.md` DEFERRED).
- 실차 제동거리·정지거리(기준본 0.1 m는 📏), 200 ms 타임아웃 최종값(명세 체크 6).
- V3의 인체 차폐·금속 반사·실제 방향 정확도(연구 문서 스스로 "판정하기 어렵다").
- 현재 위상 PDoA에서 전후방 대칭(앞-왼쪽과 뒤-왼쪽 동일 부호) 여부의 정량값 [추정: 2소자 PDoA는 전후 대칭].
- kaist-vlm 0x240 수신부의 실보드 HIL(이주 문서 §4 B–E 미착수).
- if4_status_protocol draft 1의 펌웨어 반영 시점.

## 13. 작업 기록
- 쓰기: 이 폴더(`C:\Users\ryan\projects\봇박스\PDoA단일추종_검토_20261120\`)만. 읽기용 `git archive` zip을 이 폴더의 `_tmp\`에 임시 생성해 박스로 복사한 뒤 삭제.
- 저장소·보드·볼트·STATUS.md 수정 없음. 통신 계약 확정·구현 없음.
