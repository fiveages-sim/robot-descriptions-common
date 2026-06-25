# robot-descriptions-common Debian 发布说明

Workflow：[`.github/workflows/build-common-deb.yml`](.github/workflows/build-common-deb.yml)

仓库：[fiveages-sim/robot-descriptions-common](https://github.com/fiveages-sim/robot-descriptions-common)

## Pre-release 与版本号

### 触发

| 触发 | 产物 |
|------|------|
| PR 合并进 `main` | 更新唯一 rolling Release **`pre-release`** |
| 打 `v*` tag / `workflow_dispatch` | 正式 Release |
| PR 关闭未合并 | 不构建 |

### Pre-release 版本规则

| 项目 | 规则 | 示例 |
|------|------|------|
| GitHub Release tag | 固定 `pre-release` | `pre-release` |
| deb `Version`（control 字段） | 在 latest 正式版基础上 **patch 累加** + `~main.` + 短 SHA | 见下表 |
| deb 文件名（Release 资产） | `ros-jazzy-robot-descriptions-common_{Version}_{arch}.deb`；GitHub 上传时 `~` 会变为 `.` | `..._1.4.2.main.7b73a8b_amd64.deb` |

**计算逻辑**（latest 正式版为 `v1.4.0` 时）：

| 场景 | 上一次 pre-release deb 基础版本 | 本次 deb Version |
|------|--------------------------------|------------------|
| 第一次 PR 合并 | 无 | `1.4.1~main.{sha}` |
| 连续第二次合并 | `1.4.1` | `1.4.2~main.{sha}` |
| 连续第三次合并 | `1.4.2` | `1.4.3~main.{sha}` |
| **重置**：上次为 `1.5.0` 或 `1.3.0`，latest 仍为 `1.4.0` | major.minor 不一致 | `1.4.1~main.{sha}` |
| **重置**：新正式版 `v1.5.0`，上次 pre-release 为 `1.4.3` | major.minor 不一致 | `1.5.1~main.{sha}` |

步骤简述：

1. 读取 **latest 正式版** `X.Y.Z`（GitHub API `/releases/latest`）
2. 从现有 `pre-release` Release 的 deb 资产名解析上一次基础版本（如 `1.4.2`）
3. 若无上一次，或 `major.minor` 与 latest 不一致 → `X.Y.(Z+1)`
4. 否则在上一次基础上 patch +1
5. 追加 `~main.{merge_commit 前 7 位}`

若无正式 release：以 `robot_common_launch/package.xml` 版本作为 latest；仍无则自 `0.0.1` 起。

### 安装示例

```bash
gh release download pre-release --repo fiveages-sim/robot-descriptions-common --pattern '*_amd64.deb'
sudo dpkg -i ros-jazzy-robot-descriptions-common_1.4.2.main.7b73a8b_amd64.deb
```

### 正式 Release

| 项目 | 规则 |
|------|------|
| GitHub Release tag | git tag，如 `v1.4.0` |
| deb `Version` | tag 去 `v`，`-` 转 `~` |
