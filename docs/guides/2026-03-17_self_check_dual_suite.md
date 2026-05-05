# 双套自检指南（电脑端 + 车端）

本指南对应两套脚本：

- 电脑端（离车）：`scripts/selfcheck.sh pc`
- 车端（上车）：`scripts/selfcheck.sh robot`

目标是把“能编译/能启动”和“实机链路正常”分开检查，减少上车排障时间。

## 1. 电脑端离车自检

用途：开发机先做构建、静态契约和 launch 语法检查，不依赖硬件。

```bash
cd ~/ros2_ly_ws_sentary
./scripts/selfcheck.sh pc
```

常用参数：

```bash
# 不重建，只跑静态检查
./scripts/selfcheck.sh pc --no-build

# 指定包并执行 colcon test
./scripts/selfcheck.sh pc --packages "behavior_tree outpost_hitter predictor" --test
```

## 2. 车端上车自检

用途：在车载机验证串口/网络可达和整链路运行时 topic 契约。

```bash
cd ~/ros2_ly_ws_sentary
./scripts/selfcheck.sh robot
```

说明：该脚本默认调用 `selfcheck.sh sentry --runtime-only --launch`，只做运行态检查，避免重复静态检查。

常用参数：

```bash
# 含频率采样（更严格）
./scripts/selfcheck.sh robot --with-hz

# 传给 sentry_all.launch.py 的参数
./scripts/selfcheck.sh robot -- --config_file:=/abs/path/auto_aim_config.yaml
```

## 3. 基础套件说明

两套脚本都复用主套件：

```bash
./scripts/selfcheck.sh sentry
```

新增模式：

- `--static-only`：只检查文件/配置/BT XML（离车）
- `--runtime-only`：只检查 node/topic/hz（上车）

你常用的运行时命令（离车先验证 ROS 图）：

```bash
./scripts/selfcheck.sh sentry --runtime-only --launch --wait 12 --skip-hz
```

完整频率版：

```bash
./scripts/selfcheck.sh sentry --runtime-only --launch --wait 12
```

当使用 `--launch` 时，若失败会自动打印 `Launch Diagnosis`，给出崩溃签名和最近日志尾部，优先定位相机/串口/进程崩溃。

## 4. 失败快速判读

- 出现 `Failed to initialize camera`：
  - 离车场景通常是未接相机，改 `detector_config/use_video=true`。
  - 上车场景检查相机供电、线缆和 SN 配置。
- 出现 `IODevice::MakeDevice`、`ttyACM/ttyUSB` 打开失败：
  - 检查串口设备与权限，或离车时改 `io_config/use_virtual_device=true`。
- 出现 `getifaddrs: Operation not permitted` / `RTPS_TRANSPORT_SHM`：
  - 多见于受限容器/沙箱，属于 DDS 传输权限问题，不是业务逻辑链路错误。
  - 需在有正常网络能力的宿主机或车端环境复测。

## 5. 判定标准

- `FAIL=0` 才算通过。
- `WARN>0` 可继续，但要逐条确认是否为当前场景预期（如未接网线导致 192.168.12.1 不通）。
