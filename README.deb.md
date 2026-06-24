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
| deb `Version` | latest **正式版** minor +1 + `~main.` + 短 SHA | latest `v1.5.0` → `1.6.0~main.7b73a8b` |
| deb 文件名 | `ros-jazzy-robot-descriptions-common_{Version}_{arch}.deb` | `ros-jazzy-robot-descriptions-common_1.6.0~main.7b73a8b_amd64.deb` |

计算步骤：

1. 通过 GitHub API 读取本仓库 **latest 正式版**（不含 `pre-release`）
2. 从 tag 解析 `X.Y.Z`，**次版本 Y +1**，patch 归零（`1.5.0` → `1.6.0`）
3. 追加 `~main.{merge_commit 前 7 位}`

若无正式 release：对 `robot_common_launch/package.xml` 中的版本做同样 minor +1；仍无则自 `0.1.0` 起。

### 安装示例

```bash
gh release download pre-release --repo fiveages-sim/robot-descriptions-common --pattern '*_amd64.deb'
sudo dpkg -i ros-jazzy-robot-descriptions-common_1.6.0~main.7b73a8b_amd64.deb
```

### 正式 Release

| 项目 | 规则 |
|------|------|
| GitHub Release tag | git tag，如 `v1.6.0` |
| deb `Version` | tag 去 `v`，`-` 转 `~` |
