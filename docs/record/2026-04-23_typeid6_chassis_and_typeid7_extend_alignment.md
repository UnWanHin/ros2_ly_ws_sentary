# TypeID=6/7 协议调整记录（2026-04-23）

## 1. 调整目标

- 将原 `TypeID=6`（旧 `ExtendData`）改名并对齐为底盘回传帧。
- 新增 `TypeID=7` 作为新的 `ExtendData`（12B 预留帧）。
- 在上位机新增一个聚合 topic：`/ly/gimbal/chassis`，承载四个底盘量。

## 2. 新结构定义

### 2.1 TypeID=6 -> `ChassisData`

```cpp
struct ChassisData {
  uint16_t UWBAngleYaw;
  uint16_t Posture;
  uint32_t ChassisPacked1; // low16=舵角当前角, high16=底盘角速度
  uint32_t ChassisPacked2; // low16=x速度, high16=y速度
};
```

其中每个 16-bit 子字段按“8位整数 + 8位小数（两位小数）”解码。

### 2.2 TypeID=7 -> `ExtendData`

```cpp
struct ExtendData {
  uint8_t Raw[12];
};
```

当前仅完成接入识别，不做字段拆解发布。

## 3. 上位机发布行为

`gimbal_driver` 对 `TypeID=6` 发布：

- `/ly/me/uwb_yaw`：`UWBAngleYaw`
- `/ly/gimbal/posture`：`Posture`（先低8位，低8位无效时回退高8位；仅 1/2/3 发布）
- `/ly/gimbal/chassis`：
  - `steer_angle`
  - `angular_velocity`
  - `velocity_x`
  - `velocity_y`

`TypeID=7` 当前仅识别，不发布新 topic。

## 4. 下位机对齐要求

- 仍保持固定 12B payload，不要改帧长。
- `TypeID=6` 按新字段语义填充。
- `TypeID=7` 若开始发送，当前上位机会接收但不解析。

## 5. 兼容性说明

- 新增 `/ly/gimbal/chassis` 作为唯一的完整底盘反馈话题。
