# `ExtendData` 上行协议速查

## 1. 结论先看

当前下位机回传给上位机的 `ExtendData`：

- 幀类型：`TypeID=6`
- 方向：下位机 -> 上位机
- 在本仓库当前上位机解析/发布逻辑里，明确读到的字段只有两个：
  - `UWBAngleYaw`
  - `Reserve_16` 低 2 bit（姿态回读）

按本仓库当前代码搜索，暂未发现其余字段的解析/发布引用：

- `Reserve_16` 的 bit2~15
- `Reserve_32_1`
- `Reserve_32_2`

代码依据：

- `src/gimbal_driver/module/BasicTypes.hpp`
- `src/gimbal_driver/main.cpp`

---

## 2. 结构体定义

当前代码中的定义如下：

```cpp
struct ExtendData {
    static constexpr auto TypeID = 6;
    std::uint16_t UWBAngleYaw;
    std::uint16_t Reserve_16;
    std::uint32_t Reserve_32_1;
    std::uint32_t Reserve_32_2;
};
```

来源：`src/gimbal_driver/module/BasicTypes.hpp`

在 `#pragma pack(push, 1)` 下，这个 payload 按当前定义是 12B。

---

## 3. 当前字段用途

| 字段 | 位范围 | 当前用途 | 上位机处理位置 |
|---|---|---|---|
| `UWBAngleYaw` | 16 bit | 自身 UWB 朝向角 | 发布到 `/ly/me/uwb_yaw` |
| `Reserve_16` | bit0~1 | 姿态状态回读 | 解析后发布到 `/ly/gimbal/posture` |
| `Reserve_16` | bit2~15 | 本仓库当前未发现解析/发布引用 | 无 |
| `Reserve_32_1` | 32 bit | 本仓库当前未发现解析/发布引用 | 无 |
| `Reserve_32_2` | 32 bit | 本仓库当前未发现解析/发布引用 | 无 |

当前上位机对 `Reserve_16` 的解释：

- `0`：未知 / 未实现 / 无效
- `1`：进攻
- `2`：防御
- `3`：移动

---

## 4. 当前上位机真实行为

`src/gimbal_driver/main.cpp` 里，`PubExtendData()` 只做两件事：

1. 读取 `UWBAngleYaw`，发布 `/ly/me/uwb_yaw`
2. 读取 `Reserve_16 & 0x03`，当作姿态值

对应逻辑要点：

- 只有 `1/2/3` 会被视为有效姿态
- 如果下位机回传 `0`，当前实现不会发布新的 `/ly/gimbal/posture`
- `Reserve_32_1` 和 `Reserve_32_2` 在本仓库当前代码里未发现解析/发布逻辑

这点要特别注意：

- 文档语义上 `0` 表示未知
- 但代码行为上，`0` 是“忽略，不发布”

也就是说，若下位机发 `0`，上位机现在看起来更像“没收到新的有效姿态”。

---

## 5. 给下位机同学的最简说明

如果只是对齐当前版本，下位机只需要保证：

1. 继续发送 `TypeID=6` 的 `ExtendData`
2. `UWBAngleYaw` 正常填写
3. 把姿态状态写入 `Reserve_16` 低 2 bit
4. `Reserve_32_1/2` 可以先不填，置 0 即可

姿态位约定：

```c
reserve_16 = (reserve_16 & ~0x3u) | (posture & 0x3u);
```

其中：

- `0` = 未知
- `1` = 进攻
- `2` = 防御
- `3` = 移动

---

## 6. 如果还想再塞别的数据

当前最安全的扩展顺序建议是：

1. 优先使用 `Reserve_32_1`
2. 再使用 `Reserve_32_2`
3. 最后才去切 `Reserve_16` 的高位

原因：

- `Reserve_16` 低 2 bit 已经被姿态占用
- `Reserve_32_1/2` 当前完全未被上位机解析，冲突最小
- 扩展时只需要同步修改：
  - `src/gimbal_driver/main.cpp`
  - 对应 topic / message 定义
  - 这份文档

---

## 7. 当前风险点

1. `Reserve_16` 的高位虽然暂时没用，但现在也没有正式位分配文档，临时塞值容易冲突。
2. 若下位机希望把“未知姿态”明确回传给上位机，当前代码还不够，因为 `0` 不会触发 `/ly/gimbal/posture` 发布。
3. 如果未来要把更多状态塞进 `ExtendData`，最好先定一版位分配表，再改代码。

---

## 8. 相关文件

- `src/gimbal_driver/module/BasicTypes.hpp`
- `src/gimbal_driver/main.cpp`
- `docs/modules/gimbal_driver.md`
- `docs/sentry/posture_lower_firmware_integration.md`
- `docs/sentry/sentry_posture_interface_change_2026-03-03.md`
