# AutonomousMobileDrone
To enable the implementation of autonomous drones using actual hardware.

# eVTOL飛行制御・シミュレーションシステム 仕様書

**Document ID:** EVTOL-SPEC-001  
**Version:** 1.0  
**対象:** 個人開発・GitHub公開用  
**開発形態:** PC上のシミュレーション  
**実機:** 使用しない

## クイックスタート

```bash
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m pytest tests -q
python simulation/run_sim.py
```

macOS で Xcode ライセンス未同意の場合は、Command Line Tools を指定する。

```bash
export DEVELOPER_DIR=/Library/Developer/CommandLineTools
cmake -S . -B build -DCMAKE_CXX_COMPILER="$DEVELOPER_DIR/usr/bin/c++"
```

シミュレーションの CSV とグラフは `results/` に出力される。

出力されるグラフ
各シナリオについて、次の PNG が作られます。


|ファイル|シナリオ|
|results/TEST-SCENARIO-001-takeoff.png|離陸（0 m → 100 m）|
|results/TEST-SCENARIO-002-hover.png|ホバリング|
|results/TEST-SCENARIO-003-descend.png|降下（100 m → 50 m）|
|results/TEST-SCENARIO-004-disturbance.png|外乱|

---

## 1. 目的

本システムは、小型eVTOL（electric Vertical Take-Off and Landing）を想定した飛行制御システムをソフトウェア上で構築し、以下の技術を習得・検証することを目的とする。

- 飛行制御
- 制御則設計
- フィードバック制御
- 飛行ダイナミクスの基礎
- シミュレーション
- 試験自動化
- C/C++による制御ソフトウェア開発
- Pythonによるシミュレーション・評価
- GitHubによるソフトウェア開発管理

本システムは実機の飛行を目的とせず、飛行制御技術を安全なシミュレーション環境で検証する。

---

# 2. システム概要

システムは以下の機能で構成する。

```text
┌──────────────────────────────┐
│       Mission / Target       │
│        目標高度・速度          │
└──────────────┬───────────────┘
               ↓
┌──────────────────────────────┐
│       Flight Controller      │
│          C / C++             │
│                              │
│  ・制御誤差計算               │
│  ・PID制御                    │
│  ・推力指令計算               │
└──────────────┬───────────────┘
               ↓
┌──────────────────────────────┐
│       eVTOL Dynamics         │
│          Python              │
│                              │
│  ・機体状態計算               │
│  ・高度変化                   │
│  ・外乱モデル                 │
└──────────────┬───────────────┘
               ↓
┌──────────────────────────────┐
│          Sensors             │
│                              │
│  ・高度                       │
│  ・速度                       │
│  ・加速度                     │
└──────────────┬───────────────┘
               │
               └──── Feedback ───→ Controller
```

---

# 3. 開発対象

## 3.1 対象機体

本システムでは、垂直離着陸可能な電動航空機を簡略化したモデルを使用する。

実際のeVTOLの機体形状、推進系、航空力学特性を完全に再現するものではない。

V1.0では、機体の鉛直方向の運動を主対象とする。

---

# 4. システム境界

## 4.1 対象範囲

V1.0では以下を対象とする。

- 高度制御
- 垂直速度制御
- 推力制御
- 飛行状態監視
- 外乱入力
- センサ値取得
- シミュレーション
- 自動試験
- 制御性能評価

## 4.2 対象外

V1.0では以下を対象外とする。

- 実機飛行
- 実機モーター制御
- 実機通信
- GPS
- カメラ
- LiDAR
- 画像認識
- SLAM
- 実際の航空機認証
- 実際のeVTOLの空力モデル

---

# 5. 機能要求

## REQ-F-001：目標高度設定

システムは、目標高度を設定できなければならない。

例：

```text
Target Altitude = 100 m
```

---

## REQ-F-002：高度取得

システムは、シミュレーション上の機体高度を取得できなければならない。

単位：

```text
m
```

---

## REQ-F-003：高度誤差計算

システムは、目標高度と現在高度から高度誤差を計算しなければならない。

```text
Error = Target Altitude - Current Altitude
```

---

## REQ-F-004：制御入力計算

システムは、高度誤差に基づいて機体への推力指令を計算しなければならない。

V1.0ではPID制御を使用する。

```text
u(t) = Kp・e(t)
     + Ki・∫e(t)dt
     + Kd・de(t)/dt
```

---

## REQ-F-005：推力制限

システムは、推力指令に上下限を設定しなければならない。

```text
0 ≤ Thrust ≤ Maximum Thrust
```

---

## REQ-F-006：機体状態更新

シミュレーションモデルは、推力指令を入力として機体の高度・速度を更新しなければならない。

---

## REQ-F-007：フィードバック制御

システムは、現在の機体状態を制御器へフィードバックし、継続的に制御入力を更新しなければならない。

---

## REQ-F-008：外乱入力

システムは、外乱をシミュレーションへ入力できなければならない。

例：

```text
External Force = -100 N
```

外乱によって高度が変化した場合、制御器は目標高度へ復帰することを目指す。

---

## REQ-F-009：飛行状態判定

システムは、現在の飛行状態を判定しなければならない。

V1.0では以下を定義する。

```text
GROUND
TAKEOFF
CLIMB
HOVER
DESCEND
LANDING
```

---

## REQ-F-010：ログ出力

システムはシミュレーション中の以下のデータを記録しなければならない。

- 時刻
- 目標高度
- 現在高度
- 高度誤差
- 垂直速度
- 推力
- 外乱
- 飛行状態

---

# 6. 非機能要求

## REQ-NF-001：再現性

同一の入力条件で実行した場合、同等のシミュレーション結果が得られること。

---

## REQ-NF-002：自動実行

シミュレーションおよびテストはコマンドラインから自動実行できること。

---

## REQ-NF-003：テスト可能性

制御器、機体モデル、センサモデルなどの主要コンポーネントを個別にテストできること。

---

## REQ-NF-004：可視化

シミュレーション結果をグラフで確認できること。

最低限、以下を表示する。

```text
高度 - 時間
速度 - 時間
推力 - 時間
高度誤差 - 時間
```

---

## REQ-NF-005：ソースコード管理

ソースコードはGitによって管理し、GitHub上で公開可能な構成とする。

---

# 7. 制御仕様

## 7.1 PID制御

高度制御にはPID制御を使用する。

入力：

```text
Target Altitude
Current Altitude
```

出力：

```text
Thrust Command
```

制御周期：

```text
10 ms
```

---

## 7.2 制御ブロック

```text
Target Altitude
       │
       ↓
 ┌───────────┐
 │   Error   │
 └─────┬─────┘
       ↓
 ┌───────────┐
 │    PID    │
 └─────┬─────┘
       ↓
 ┌───────────┐
 │   Thrust  │
 │ Limiter   │
 └─────┬─────┘
       ↓
 ┌───────────┐
 │   eVTOL   │
 │  Dynamics │
 └─────┬─────┘
       ↓
 Current Altitude
       │
       └────────────→ Feedback
```

---

# 8. 機体モデル仕様

V1.0では鉛直方向の運動のみを扱う。

基本モデル：

```text
m × a = T - mg - Fd
```

ここで、

```text
m  = 機体質量
a  = 鉛直加速度
T  = 推力
g  = 重力加速度
Fd = 外乱・抵抗等
```

とする。

簡略化モデルとして、初期開発では空気抵抗などを無視してもよい。

---

# 9. センサモデル

V1.0では仮想センサを使用する。

## 高度センサ

入力：

```text
True Altitude
```

出力：

```text
Measured Altitude
```

将来的にはセンサノイズを追加する。

例：

```text
Measured Altitude
= True Altitude + Sensor Noise
```

---

# 10. 飛行シナリオ

## TEST-SCENARIO-001：離陸

初期状態：

```text
Altitude = 0 m
Velocity = 0 m/s
```

目標：

```text
Target Altitude = 100 m
```

期待動作：

```text
GROUND
 ↓
TAKEOFF
 ↓
CLIMB
 ↓
HOVER
```

---

## TEST-SCENARIO-002：ホバリング

目標高度：

```text
100 m
```

機体が100m付近で安定していることを確認する。

---

## TEST-SCENARIO-003：降下

目標高度を、

```text
100 m → 50 m
```

へ変更する。

期待動作：

```text
HOVER
 ↓
DESCEND
 ↓
HOVER
```

---

## TEST-SCENARIO-004：外乱

ホバリング中に外乱を入力する。

```text
External Force ≠ 0
```

期待動作：

```text
外乱発生
 ↓
高度変化
 ↓
制御器が誤差を検出
 ↓
推力調整
 ↓
目標高度へ復帰
```

---

# 11. 性能評価指標

制御性能を以下の指標で評価する。

### 定常偏差

目標高度と最終高度の差。

```text
Steady State Error
```

---

### オーバーシュート

目標高度を超えた最大量。

```text
Overshoot
```

---

### 整定時間

高度が目標値の許容範囲内に入り、その状態を維持するまでの時間。

---

### 最大偏差

外乱発生後の最大高度偏差。

---

# 12. テスト要求

## TEST-001：高度制御

条件：

```text
Initial Altitude = 0 m
Target Altitude = 100 m
```

期待結果：

```text
最終高度 ≈ 100 m
```

---

## TEST-002：目標高度変更

条件：

```text
Initial Target = 100 m
New Target = 50 m
```

期待結果：

```text
機体高度が50mへ収束する
```

---

## TEST-003：外乱耐性

条件：

```text
Target Altitude = 100 m
External Force = predetermined disturbance
```

期待結果：

```text
外乱後に目標高度へ復帰する
```

---

## TEST-004：推力上限

条件：

```text
Required Thrust > Maximum Thrust
```

期待結果：

```text
Thrust Command = Maximum Thrust
```

---

## TEST-005：推力下限

条件：

```text
Required Thrust < 0
```

期待結果：

```text
Thrust Command = 0
```

---

# 13. ソフトウェア構成

```text
src/
│
├── controller/
│   ├── pid_controller.cpp
│   └── pid_controller.hpp
│
├── dynamics/
│   ├── evtol_model.cpp
│   └── evtol_model.hpp
│
├── sensor/
│   ├── altitude_sensor.cpp
│   └── altitude_sensor.hpp
│
└── flight_state/
    ├── flight_state.cpp
    └── flight_state.hpp
```

Python側：

```text
simulation/
│
├── simulator.py
├── scenario.py
└── visualization.py
```

テスト：

```text
tests/
│
├── test_pid.cpp
├── test_dynamics.cpp
├── test_sensor.cpp
└── test_scenario.py
```

---

# 14. 使用技術

V1.0では以下を使用する。

### 必須

- C++
- Python
- Git
- GitHub
- CMake
- GoogleTest または同等のC++テストフレームワーク
- Python unittest / pytest

### 将来導入

- MATLAB / Simulink
- ROS 2
- Gazebo
- HILS
- CANoe
- GitHub Actions

---

# 15. 開発フェーズ

## Phase 1：高度制御

```text
PID
 ↓
高度制御
 ↓
シミュレーション
```

---

## Phase 2：飛行状態

```text
GROUND
TAKEOFF
CLIMB
HOVER
DESCEND
LANDING
```

を追加する。

---

## Phase 3：センサモデル

センサノイズや遅延を追加する。

---

## Phase 4：姿勢制御

以下を追加する。

```text
Roll
Pitch
Yaw
```

---

## Phase 5：6DOFモデル

位置：

```text
X
Y
Z
```

姿勢：

```text
Roll
Pitch
Yaw
```

を扱う。

---

## Phase 6：自律航法

```text
現在位置
   ↓
目標位置
   ↓
経路計画
   ↓
飛行制御
```

を追加する。

---

## Phase 7：HILS

実際のFlight Controllerまたは組込み環境を接続し、

```text
Simulation
    ↕
Flight Controller
```

というHILS環境へ発展させる。

---

# 16. 要求・設計・実装・テストのトレーサビリティ

本プロジェクトでは、要求からテストまでの関係を管理する。

例：

```text
REQ-F-003
高度誤差計算
     ↓
DESIGN-003
高度誤差計算モジュール
     ↓
IMPLEMENT-003
PIDController::calculateError()
     ↓
TEST-001
高度制御試験
```

この関係をGitHub上で管理する。

将来的にはPolarion等の要求管理ツールを使用することを想定する。

---

# 17. 成果物

最終的に以下をGitHubで公開する。

- 要求仕様書（`README.md`）
- システム構成図（`README.md` 第2章）
- 制御設計書（`docs/CONTROL_DESIGN.md`）
- 機体モデル仕様（`README.md` 第8章、制御設計書第2章）
- ソースコード
- テストコード
- シミュレーションコード
- テスト結果
- 制御性能グラフ
- GitHub Actionsによる自動テスト
- 開発履歴

---

# 18. V1.0の完成条件

以下をすべて満たした場合、V1.0完成とする。

- [x] C++でPID制御器が動作する
- [x] eVTOL簡易モデルが動作する
- [x] 目標高度100mへの制御が可能
- [x] 目標高度変更に追従できる
- [x] 外乱入力に対応できる
- [x] 飛行状態を判定できる
- [x] シミュレーション結果をログ出力できる
- [x] 高度・速度・推力をグラフ化できる
- [x] 自動テストが実行できる
- [x] GitHub上でソースコードを公開できる
- [x] 要求とテストの対応関係を確認できる

---

# 19. 将来目標

V1.0完成後、以下の順番でシステムを高度化する。

```text
V1.0
高度制御
  ↓
V2.0
姿勢制御
  ↓
V3.0
6DOF飛行モデル
  ↓
V4.0
自律航法
  ↓
V5.0
障害物回避
  ↓
V6.0
ROS 2統合
  ↓
V7.0
HILS
  ↓
V8.0
実機検証
```

最終目標は、**飛行制御・航法・シミュレーション・自動試験を統合した自律eVTOLシステム**とする。