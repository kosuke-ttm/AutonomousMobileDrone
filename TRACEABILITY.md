# 要求トレーサビリティ (V1.0)

| 要求 | 設計 / 実装 | テスト |
|------|-------------|--------|
| REQ-F-001 目標高度設定 | `Simulator.run(target_fn)` | TEST-001, TEST-002, `tests/test_scenario.py` |
| REQ-F-002 高度取得 | `EvtolModel::altitude`, `AltitudeSensor::measure` | `tests/test_dynamics.cpp`, `tests/test_sensor.cpp` |
| REQ-F-003 高度誤差計算 | `PidController::calculateError` | `tests/test_pid.cpp` CalculateError |
| REQ-F-004 制御入力計算 | `PidController::update`, `AltitudeController` | `tests/test_pid.cpp`, TEST-001 |
| REQ-F-005 推力制限 | PID limiter, `EvtolModel::step` clamp | TEST-004, TEST-005 |
| REQ-F-006 機体状態更新 | `EvtolModel::step` | `tests/test_dynamics.cpp` |
| REQ-F-007 フィードバック制御 | `Simulator.run` 閉ループ | `tests/test_scenario.py` |
| REQ-F-008 外乱入力 | `disturbance_fn` → `EvtolModel::step` | TEST-003, TEST-SCENARIO-004 |
| REQ-F-009 飛行状態判定 | `FlightStateEstimator` | `tests/test_flight_state.cpp` |
| REQ-F-010 ログ出力 | `SimulationResult.write_csv` | `test_logging_fields_present` |
| REQ-NF-001 再現性 | 決定論センサRNG、固定時間刻み | `AltitudeSensor.NoiseIsDeterministic` |
| REQ-NF-002 自動実行 | `simulation/run_sim.py`, CMake/ctest, pytest, CI | `.github/workflows/ci.yml` |
| REQ-NF-003 テスト可能性 | コンポーネント単体テスト | `tests/test_*.cpp` |
| REQ-NF-004 可視化 | `simulation/visualization.py` | `python simulation/run_sim.py` |
