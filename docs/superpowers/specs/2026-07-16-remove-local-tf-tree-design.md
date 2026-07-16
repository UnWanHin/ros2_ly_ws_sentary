# 移除本仓 `tf_tree` fallback 设计

日期：2026-07-16

## 背景与决定

本仓已采用 decision-only 架构。外部 `sentry_tf` 是正式和调试环境固定的 gimbal TF 发布者；本仓 `tf_tree` 仅以 `/ly/gimbal/angles` 构建同一段局部 TF 的 fallback。两者同时运行会产生重复 TF owner 风险，且 fallback 已不再有使用需求。

决定完整移除 `tf_tree`，不保留 `use_tf_tree` 或 `tf_tree_params_file` 兼容入口。

## 变更范围

1. 删除 `src/tf_tree` ROS package。
2. 从 `sentry_all`、FaceMode launch 和它们的 wrapper launch 中删除本地 TF fallback 参数、默认路径解析、include 与日志。
3. 将操作脚本改为仅依赖外部 TF；删除其 `--with-tf-tree` / `--without-tf-tree` 选择和向 launch 传递的参数。
4. 自检不再构建或检查 `tf_tree`；保留“不能有两个 gimbal TF owner 同时在线”的运行检查，以检测外部部署异常。
5. 更新当前文档、Obsidian 生成索引与 Understand Anything fallback 图谱，明确外部 `sentry_tf` 是唯一 TF owner。

## 不变项

- `/tf`、`/tf_static` 的外部接口和 frame 命名不变。
- `navi_tf_bridge` 仍只查询 TF，不承担发布职责。
- 行为树、FaceMode、导航和 gimbal 的既有 ROS topic/message 契约不变。

## 验收

- 全仓无运行时 `tf_tree` / `use_tf_tree` / `tf_tree_params_file` 引用（历史记录可保留）。
- 受影响包可 `colcon build`。
- `./scripts/selfcheck.sh sentry --static-only` 通过。
- 图谱 JSON 可解析，图谱与当前 HEAD/包清单一致。
