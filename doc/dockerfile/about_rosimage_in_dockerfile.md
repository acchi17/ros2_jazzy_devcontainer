# DockerfileのFROMに指定するROSイメージについて

このドキュメントでは、`Dockerfile`の`FROM`に記述される`osrf/ros:jazzy-desktop`のような文字列の意味について説明します。

## 1. 基本的な書式

```
<リポジトリ名>/ros:<ディストリビューション名>-<バリアント名>
```

例: `osrf/ros:jazzy-desktop` → リポジトリ`osrf`、ディストリビューション`jazzy`、バリアント`desktop`

## 2. バリアント（タグ末尾）の階層構成

ROS 2の公式Dockerイメージは、含まれるパッケージ量に応じて以下のように積み上げ式の階層になっています。上位のバリアントは下位のバリアントを内包します。

| バリアント | 内容 |
|---|---|
| `ros-core` | 最小限のROS 2ランタイムのみ |
| `ros-base` | `ros-core` + 基本的なビルド・通信ツール群 |
| `robot` | `ros-base` + ロボット制御でよく使うパッケージ（tf2, diagnosticsなど） |
| `perception` | `robot` + 画像処理・センサー系パッケージ（image_transport, vision_opencv, PCL系など） |
| `desktop` | `perception` + RViz2やデモ用GUIツールなど、デスクトップ環境一式 |
| `desktop-full` | `desktop` + チュートリアルやシミュレーション関連の追加パッケージ |

この階層構成はDockerイメージ固有のものではなく、[REP 2001](https://www.ros.org/reps/rep-2001.html)で定義されているROS 2ディストリビューション共通のメタパッケージ構成に由来します。

GUIが不要な実機（組み込み用途など）では`perception`や`robot`のような軽量なバリアントを選び、RViz2などの可視化ツールが必要な開発環境では`desktop`を選ぶ、といった使い分けができます。

## 3. リポジトリ名（`osrf/` vs `amd64/`など）の違い

`FROM`の前半部分（リポジトリ名）によって、イメージの配布元が異なります。

**Docker公式イメージ（`amd64/ros`, `arm64v8/ros`など）**
- Docker Hubの[ros公式イメージ](https://hub.docker.com/_/ros)のアーキテクチャ別リポジトリです。
- [docker-library/official-images](https://github.com/docker-library/official-images/blob/master/library/ros)で管理されており、Dockerの公式イメージ審査プロセスを経て配布されています。
- Dockerfile定義自体は後述の`osrf/docker_images`を参照しています。

**OSRF配布イメージ（`osrf/ros`）**
- Open Robotics（OSRF）が[osrf/docker_images](https://github.com/osrf/docker_images)リポジトリのDockerfileから直接ビルド・公開しているイメージです。
- 公式イメージの審査プロセスを経ないため、タグの追加が早い場合があります。
- GUI関連（X11など）の重いパッケージを含むメタパッケージは、公式イメージ側を軽量に保つためOSRF側で配布される運用になっています。

どちらも元になっているDockerfile定義はほぼ同じですが、「Docker公式の配布パイプラインを通っているか」「OSRFが独自に配布しているか」という違いがあります（参考: [osrf/docker_images Issue #283](https://github.com/osrf/docker_images/issues/283)）。

## 参考リンク

- [ros - Official Image | Docker Hub](https://hub.docker.com/_/ros)
- [GitHub - osrf/docker_images](https://github.com/osrf/docker_images)
- [docker_images/README.md](https://github.com/osrf/docker_images/blob/master/README.md)
- [REP 2001 -- ROS 2 Package Naming and Versioning](https://www.ros.org/reps/rep-2001.html)
