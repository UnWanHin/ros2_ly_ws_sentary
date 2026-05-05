# 自检状态报告（2026-03-05）

## 执行命令

```bash
./scripts/selfcheck.sh pc
./scripts/selfcheck.sh sentry --static-only --skip-hz
./scripts/selfcheck.sh sentry --runtime-only --launch --wait 12 --skip-hz
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon test --packages-select behavior_tree buff_hitter outpost_hitter
colcon test-result --verbose
```

## 结果摘要

- `selfcheck.sh pc`：通过  
  - 9 个包构建完成。  
  - 静态契约检查 `PASS=19, FAIL=0`。
- `selfcheck.sh sentry --static-only`：通过  
  - 文件、BT XML、camera_sn 双键一致性检查全部通过。
- `selfcheck.sh sentry --runtime-only --launch`：失败（宿主权限环境复测）  
  - ROS2 图可建立，在线节点：`/gimbal_driver`、`/tracker_solver`、`/predictor_node`、`/outpost_hitter`。  
  - 缺失节点：`/detector`、`/buff_hitter`、`/behavior_tree`。  
  - `Launch Diagnosis` 关键签名：
    - `detector`: `Failed to initialize camera with SN: KE0200060396`
    - `gimbal_driver`: `IODevice::MakeDevice ...`（持续串口打开失败）
    - `behavior_tree`: `process has died ... exit code -11`
    - `buff_hitter`: `process has died ... exit code -11`
- `colcon test --packages-select behavior_tree buff_hitter outpost_hitter`：完成  
  - `Summary: 0 tests, 0 errors, 0 failures, 0 skipped`。

## 当前可确认状态

- 代码可编译，静态接口契约完整。
- 自检脚本可输出离车/上车模式提示，并在 `--launch` 失败时自动给出诊断摘要。
- 当前运行态主要问题不是 DDS 权限，而是硬件依赖与进程稳定性：
  - 相机未成功初始化；
  - 串口设备未打开（当前配置 `io_config/use_virtual_device=false`）；
  - `behavior_tree` 与 `buff_hitter` 仍存在早期崩溃问题（需继续单节点定位）。

## 建议复测命令（宿主机/车端）

```bash
./scripts/selfcheck.sh robot
# 或
./scripts/selfcheck.sh sentry --runtime-only --launch --wait 12 --skip-hz
```
