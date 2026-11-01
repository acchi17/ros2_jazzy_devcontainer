# ROS 2 Python（rclpy）でノードを作る方法（最小構成）

このドキュメントでは、ROS 2（想定: Jazzy）で **Python（rclpy）製ノード**を作成し、ビルドして実行するまでを、最小構成でまとめます。

題材として `geometry_msgs/msg/Twist` を `/cmd_vel` で購読し、受信内容をログ出力するサブスクライバを作ります。

---

## 前提

- ROS 2 がインストール済みで、端末で `ros2` コマンドが使えること
- ワークスペースを作業するディレクトリがあること（例: `~/ros2_ws`）

以降はワークスペースとして `ros2_ws` を使う例で説明します。

---

## 0. ワークスペースの準備

まだ無い場合は作成します。

```bash
mkdir -p ~/ros2_ws/src
```

---

## 1. ROS2パッケージ作成

Python用ROS2パッケージを作成します。

```bash
cd ~/ros2_ws/src
ros2 pkg create rc_driver --build-type ament_python --dependencies rclpy geometry_msgs
```

作成される主な構成（概略）:

```
rc_driver/
  package.xml
  setup.py
  setup.cfg
  resource/
  rc_driver/
    __init__.py
```

> ここで指定している **`rc_driver` は「パッケージ名」**です（`ros2 run` や `ros2 launch` で指定する `package` 名になります）。

このコマンドのオプションは次の意味です。

- `--build-type ament_python`: パッケージのビルド形式を **Python用（ament_python）** にします（C++なら `ament_cmake` が一般的です）。
- `--dependencies rclpy geometry_msgs`: このパッケージが依存するROSパッケージを指定します。指定した依存関係は `package.xml` などに反映されます。
  - `rclpy`: ROS 2 の **Pythonクライアントライブラリ**です（Pythonでノードを書くための基本機能）。
  - `geometry_msgs`: `Twist` や `Pose` など、よく使う **標準メッセージ型**が入ったパッケージです（今回の `Twist` は `geometry_msgs/msg/Twist`）。

---

## 2. ノード（Subscriber）実装

`rc_driver/rc_driver/` 配下にノード実装ファイルを作成します。

例: `rc_driver/rc_driver/twist_listener_node.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class TwistListener(Node):
    def __init__(self):
        super().__init__('twist_listener')

        topic_name = '/cmd_vel'
        qos_depth = 10

        self.subscription = self.create_subscription(
            Twist,
            topic_name,
            self.on_twist,
            qos_depth,
        )
        self.subscription  # keep reference

        self.get_logger().info(f'Subscribed to: {topic_name} (qos_depth={qos_depth})')

    def on_twist(self, msg: Twist) -> None:
        self.get_logger().info(
            'Twist received: '
            f'linear=({msg.linear.x:.3f}, {msg.linear.y:.3f}, {msg.linear.z:.3f}) '
            f'angular=({msg.angular.x:.3f}, {msg.angular.y:.3f}, {msg.angular.z:.3f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TwistListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 3. `setup.py` に entry point を追加（重要）

`ros2 run` で起動するために、`setup.py` の `entry_points` を設定します。

`twist_listener_py/setup.py` の末尾付近に以下のように追加/編集してください。

```python
entry_points={
    'console_scripts': [
        'twist_listener = twist_listener_py.twist_listener_node:main',
    ],
},
```

> 既に `entry_points` がある場合は、上の `console_scripts` を追記します。

---

## 4. ビルド

ワークスペース直下で `colcon build` します。

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

ビルド後、環境を反映します。

```bash
source install/setup.bash
```

---

## 5. 実行（ros2 run）

```bash
ros2 run twist_listener_py twist_listener
```

起動ログとして `Subscribed to: /cmd_vel ...` が表示されればOKです。

---

## 6. 動作確認（Twist を publish）

別ターミナルで以下を実行すると、`/cmd_vel` に Twist を送れます。

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -r 2
```

受信側ターミナルに以下のようなログが流れれば成功です。

```
Twist received: linear=(0.100, 0.000, 0.000) angular=(0.000, 0.000, 0.500)
```

---

## 補足: QoSについて（最低限）

上のサンプルでは `create_subscription(..., qos_depth=10)` としていて、最小限の既定QoSで購読しています。

- `cmd_vel` のような速度指令は、用途によって **BEST_EFFORT** が望ましい場合があります（多少欠落しても最新が重要など）。
- もしQoS不一致で受信できない場合は、QoS指定を明示すると切り分けしやすいです。

（必要なら、このドキュメントに BEST_EFFORT を指定する例も追記できます。）

---

## 参考: 単一ファイルで実行する最小例

パッケージ化せずに試すだけなら、以下のように単一ファイルで実行もできます。

```bash
python3 sample.py
```

ただし、チーム開発や再利用を考えると `ament_python` パッケージ化して `ros2 run` で実行できる形にするのが一般的です。
