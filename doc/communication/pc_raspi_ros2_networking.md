# PCとラズパイ間でROS2トピックをやり取りするための設定

PC(`.devcontainer/Dockerfile`環境)で動かすノードが配信する `/cmd_vel` などのトピックを、ラズパイ(`.devcontainer/Dockerfile.raspi`環境、`rc_driver`)側のノードが購読できるようにするための設定をまとめる。

## なぜ追加設定が必要か

ROS2のデフォルトDDS実装(Fast DDS)は、ノード同士の発見(ディスカバリ)にUDPマルチキャストを使う。マルチキャストはNAT(ネットワークアドレス変換)を越えられない。

PCはWindows上のDocker Desktop(WSL2バックエンド)で動作しており、WSL2はデフォルトで「NATモード」であるため、コンテナ側で `--network=host` を指定してWSL2 VMとネットワーク名前空間を共有しても、ラズパイから見ればPC側はまだNATの奥にいる。ラズパイ側から見てPC側への**インバウンド**到達性を作るには、WSL2のミラーモードやポートフォワードなど追加のWindows側設定が必要になる。

## 採用方式: Zenoh(rmw_zenoh)によるルーター間接続

`rmw_zenoh_cpp` に切り替え、**ルーター同士を接続する**構成にすることで、上記のインバウンド到達性の問題を回避する。

- PC側・ラズパイ側それぞれで `rmw_zenohd`(Zenohルーター)をローカルに常駐させる。**ROS2ノード側のZenohセッションはデフォルト設定のままでよい**(ノードは自動的に同一ホスト上のローカルルーターを見つけて使う)。
- **ラズパイ側のルーター**(`zenoh/raspi_router_config.json5`)はデフォルト同然の設定で、全インターフェースの `tcp/0.0.0.0:7447` で待ち受けるだけ。ラズパイはコンテナが `--network host` で動いており、かつNATされていない通常のLANメンバーなので、これだけで外部から到達可能になる。
- **PC側のルーター**(`zenoh/pc_router_config.json5`)は、`connect.endpoints` にラズパイの `tcp/<raspi-ip>:7447` を指定し、そこへ**アウトバウンドで接続**する。

接続の起点をPC側からラズパイ側への一方向(アウトバウンド)に固定しているのがポイント。

- **アウトバウンド接続**(内側から外へ接続を開始する)は、NAT装置が自動的にコネクション状態を記録して戻りのパケットを通す(NATマスカレード/SNAT)ため、事前設定なしに機能する。ブラウザがルーターの内側からWebサイトへアクセスできるのと同じ仕組み。
- 一方、**ポートフォワード**(外部からNATの内側へ接続を開始できるようにする静的な転送設定)は、逆方向(ラズパイ→PC)の到達性が必要な場合にのみ要る。今回はこの方向の通信を避ける設計にしているため、ポートフォワードもWSL2ミラーモードも不要になる。

必要なのはラズパイ側OSのファイアウォールでTCP 7447のインバウンドを許可しておくことだけ(PC・WSL2側の追加設定は不要)。

### 設定箇所

- Dockerfile: [`.devcontainer/Dockerfile`](../../.devcontainer/Dockerfile) と [`.devcontainer/Dockerfile.raspi`](../../.devcontainer/Dockerfile.raspi) に `ros-jazzy-rmw-zenoh-cpp` を追加。
- PC側: [`devcontainer.json`](../../.devcontainer/devcontainer.json) で `RMW_IMPLEMENTATION=rmw_zenoh_cpp` と `ZENOH_ROUTER_CONFIG_URI` を設定し、`postStartCommand` でコンテナ起動時に `rmw_zenohd` をバックグラウンド起動する。ルーター設定は [`zenoh/pc_router_config.json5`](../../zenoh/pc_router_config.json5)(`<raspi-ip>` は実際のラズパイのLAN IPに置き換える)。
- ラズパイ側: [`docker_script/docker-run-for-raspi.sh`](../../docker_script/docker-run-for-raspi.sh) で同様に環境変数を設定し、`rmw_zenohd` をバックグラウンドで起動する。ルーター設定は [`zenoh/raspi_router_config.json5`](../../zenoh/raspi_router_config.json5)。
- `ROS_DOMAIN_ID` は両側とも `30` のまま維持(Zenoh移行後も踏襲)。

## 動作確認手順

1. 前提確認: PC側devcontainer内から `curl <raspi-ip>` や `nc -vz <raspi-ip> <ポート>` でラズパイへのアウトバウンド疎通を確認する。
2. ラズパイ側: `docker-run-for-raspi.sh` でコンテナを起動し、`ss -tlnp | grep 7447` などでルーターが待受していることを確認する。
3. PC側: devcontainerを再ビルド・起動し、以下でテスト用のメッセージを配信する。
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
   ```
4. ラズパイ側で購読できているか確認する。
   ```bash
   ros2 topic echo /cmd_vel
   ```
5. 逆方向(ラズパイ→PC)のトピックも同様に確認する。
6. 届かない場合の切り分け:
   - `ros2 topic list` を両側で実行し、相手側のトピックが見えているか
   - PC側の `rmw_zenohd` ログ(`/tmp/rmw_zenohd.log`)でラズパイ側ルーターへの接続が確立しているか
   - ラズパイ側ファイアウォールでTCP 7447がブロックされていないか

## 参考: 以前検討していたフォールバック案(未採用)

Zenohに切り替える前に検討していた、デフォルトDDS(Fast DDSまたはCycloneDDS)のままNATを越える案。マルチキャストに頼らない構成が必要になり、以下のいずれかが必要だった。

- WSL2を「ミラーモード」にする(`%UserProfile%\.wslconfig` に `networkingMode=mirrored` を設定し、Windows 11 22H2以降が必要)+ Docker Desktopの「Enable host networking」を有効化する
- RMW実装をCycloneDDS(`rmw_cyclonedds_cpp`)に切り替え、`CYCLONEDDS_URI` で相手のIPアドレスを静的ピアとして指定 + `netsh interface portproxy` でWSL2 VMへポート転送する

いずれもラズパイ→PCへの**インバウンド**到達性を作り込む必要があり、Windows側の追加設定に依存するため、より単純に実現できるZenohのルーター間接続方式を採用した。
