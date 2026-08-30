# PCとラズパイ間でROS2トピックをやり取りするための設定

PC(`.devcontainer/Dockerfile`環境)で動かすノードが配信する `/cmd_vel` などのトピックを、ラズパイ(`.devcontainer/Dockerfile.raspi`環境、`rc_driver`)側のノードが購読できるようにするための設定をまとめる。

## なぜ追加設定が必要か

ROS2のデフォルトDDS実装(Fast DDS)は、ノード同士の発見(ディスカバリ)にUDPマルチキャストを使う。マルチキャストはNAT(ネットワークアドレス変換)を越えられないため、以下のいずれかの条件が満たされていないとPC側とラズパイ側のノードは互いを発見できない。

- 両方のコンテナが同一のLANセグメントに直接参加している(NATを挟まない)
- 両方の `ROS_DOMAIN_ID` が一致している

## PC側(Windows + Docker Desktop + WSL2)の前提設定

PCはWindows上のDocker Desktop(WSL2バックエンド)で動作している。WSL2はデフォルトで「NATモード」であり、WSL2 VM自体がPC内部の仮想スイッチ(`vEthernet (WSL)`)の背後にいる。この状態では、コンテナ側で `--network=host` を指定してWSL2 VMとネットワーク名前空間を共有しても、ラズパイから見ればまだNATの奥にいるままで到達できない。

そのため、次の2つを両方とも行う必要がある。

1. **WSL2をミラーモードにする**(Windows 11 22H2以降で利用可能)
   `%UserProfile%\.wslconfig` に以下を追記し、PowerShellで `wsl --shutdown` を実行して再起動する。
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
   ミラーモードにすると、WSL2 VMはPCの実ネットワークインターフェースをそのまま共有し、PCと同じLAN上のIPアドレスを持つようになる。

2. **Docker DesktopのHost networking機能を有効化する**
   Docker Desktop → Settings → Resources → Network → “Enable host networking” をオンにする。これにより `--network=host` を指定したコンテナが、WSL2 VMのネットワーク名前空間(ミラーモードによりPCのLANと同一)をそのまま利用できるようになる。

3. **Windowsファイアウォールの許可**
   プライベートネットワークプロファイル上で、ROS2 DDSが使うUDP通信(マルチキャスト含む)を許可する。動作確認時に届かない場合は、まずファイアウォールを一時的に無効化するか、該当ポートの許可ルールを作成して切り分けること。

`.devcontainer/devcontainer.json` にはこれに対応する `runArgs: ["--network=host"]` を設定済み(要Docker Desktop再起動・Dev Container再ビルド)。

## `ROS_DOMAIN_ID` を揃える

同じLAN上に複数のROS2システムが存在する場合の混信を避けるため、PC側・ラズパイ側で同じ `ROS_DOMAIN_ID` を明示的に設定している。

- PC側: `.devcontainer/devcontainer.json` の `containerEnv.ROS_DOMAIN_ID`
- ラズパイ側: `docker_script/docker-run-for-raspi.sh` の `-e ROS_DOMAIN_ID=30`

両方とも `30` に固定してある。値自体に意味はなく、両側で一致していることが重要。

## RMW実装について

ROS2 Jazzyのデフォルト実装(`rmw_fastrtps_cpp`)を両側とも変更せずに使う想定。明示的な設定は不要。

## ミラーモードが使えない場合(フォールバック)

Windows 10など、WSL2のミラーモードが利用できない環境では、マルチキャストディスカバリに頼らない構成が必要になる。

- RMW実装をCycloneDDS(`rmw_cyclonedds_cpp`)に切り替え、`CYCLONEDDS_URI` で相手のIPアドレスを静的ピアとして指定する
- WSL2 NAT配下のPCへラズパイから到達できるよう、Windows側で `netsh interface portproxy` を使い該当ポートをWSL2 VMのIPへ転送する

この構成は追加の設定項目が多く複雑になるため、可能な限りミラーモードの利用を推奨する。

## 動作確認手順

1. PC側: `.devcontainer/devcontainer.json` 変更後、Dev Containerを再ビルドして起動する。
2. ラズパイ側: 変更後の `docker_script/docker-run-for-raspi.sh` でコンテナを起動する(`rc_driver` は既存の起動コマンドのまま自動起動する)。
3. PC側で以下を実行してテスト用のメッセージを配信する。
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
   ```
4. ラズパイ側で購読できているか確認する。
   ```bash
   ros2 topic echo /cmd_vel
   ```
5. 届かない場合は以下で切り分ける。
   - `ros2 topic list` を両側で実行し、`/cmd_vel` が相手側からも見えているか
   - `ros2 multicast receive` / `ros2 multicast send` でマルチキャスト疎通そのものを確認
   - Windowsファイアウォールのログでブロックされていないか確認
