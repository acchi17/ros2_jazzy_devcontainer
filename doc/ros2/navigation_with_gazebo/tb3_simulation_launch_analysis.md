# tb3_simulation_launch.py の処理内容まとめ（Nav2 + TurtleBot3 + Gazebo Sim）

## 概要

`tb3_simulation_launch.py` は、Nav2開発者向けの **all-in-one** な launch ファイルで、TurtleBot3（Waffle）を Gazebo Sim 上に立ち上げ、Nav2 スタック（必要に応じてSLAM）と RViz2 をまとめて起動します。

この launch 1つで、概ね以下を一括で実行します。

- ワールド（SDF.xacro）を展開して Gazebo Sim（server）を起動
- （headlessでなければ）Gazebo Sim の GUI（client）を起動
- TurtleBot3 を Gazebo にスポーンし、ros_gz_bridge を起動（別launchに委譲）
- `robot_state_publisher` を起動
- RViz2 を起動（別launchに委譲）
- Nav2 bringup を起動（別launchに委譲）

---

## 依存するパッケージ/launch（全体像）

このlaunchが内部で参照する主なパッケージは以下です。

| 目的 | パッケージ | 利用するもの |
|---|---|---|
| Nav2 bringup一式 | `nav2_bringup` | `bringup_launch.py`, `rviz_launch.py`, params/map/rviz設定 |
| TB3 Gazebo Sim最小構成 | `nav2_minimal_tb3_sim` | world, urdf, SDF(xacro), `spawn_tb3.launch.py` |
| Gazebo Sim起動（GUI側） | `ros_gz_sim` | `gz_sim.launch.py` |

アーキテクチャ（概念図）：

```
ROS 2 Launch (tb3_simulation_launch.py)
  |
  +-- Gazebo Sim server:  gz sim -r -s <world.sdf>
  +-- Gazebo Sim client:  (headless=false のとき) ros_gz_sim/gz_sim.launch.py
  |
  +-- spawn_tb3.launch.py
  |     +-- ros_gz_bridge (parameter_bridge)
  |     +-- ros_gz_sim/create (spawn)
  |
  +-- robot_state_publisher
  +-- nav2_bringup/rviz_launch.py
  +-- nav2_bringup/bringup_launch.py
```

---

## Launch引数（DeclareLaunchArgument）

本ファイルは Nav2 bringup 側の一般的な引数に加え、シミュレーション向け引数を多く持ちます。

### Nav2 bringup系

| 引数名 | デフォルト | 説明 |
|---|---|---|
| `namespace` | `''` | トップレベル名前空間 |
| `use_namespace` | `false` | Nav2 スタックに namespace を適用するか |
| `slam` | `False` | SLAM を動かすか（mapを使うか） |
| `map` | `<nav2_bringup>/maps/tb3_sandbox.yaml` | map yaml（slam=false時に使用） |
| `use_sim_time` | `true` | Gazebo の /clock を利用するか |
| `params_file` | `<nav2_bringup>/params/nav2_params.yaml` | Nav2 ノード群のパラメータ |
| `autostart` | `true` | Nav2 lifecycle の自動遷移 |
| `use_composition` | `True` | composable node で起動するか |
| `use_respawn` | `False` | クラッシュ時リスポーン（composition無効時に適用） |

### 表示/シミュレーション系

| 引数名 | デフォルト | 説明 |
|---|---|---|
| `rviz_config_file` | `<nav2_bringup>/rviz/nav2_default_view.rviz` | RViz表示設定 |
| `use_simulator` | `True` | Gazebo を起動するか |
| `use_robot_state_pub` | `True` | robot_state_publisher を起動するか |
| `use_rviz` | `True` | RViz を起動するか |
| `headless` | `True` | Gazebo GUI（client）を起動しない（※命名は「GUIを起動するか」ではなく headless の真偽で制御） |
| `world` | `<nav2_minimal_tb3_sim>/worlds/tb3_sandbox.sdf.xacro` | ワールド（SDF.xacro） |

#### world（未指定時のデフォルトは何？どこで決まる？）

`world` 引数を指定しない場合に使われるデフォルトは、このファイル内の `DeclareLaunchArgument('world', default_value=...)` で設定されています。

```python
sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

declare_world_cmd = DeclareLaunchArgument(
    'world',
    default_value=os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro'),
    description='Full path to world model file to load',
)
```

したがって、デフォルトで読み込まれるワールドは次のファイルです。

- `nav2_minimal_tb3_sim/worlds/tb3_sandbox.sdf.xacro`

実際に参照される **フルパス** は、あなたの環境で `nav2_minimal_tb3_sim` パッケージの share ディレクトリがどこにあるかで決まります。
確認するには次のコマンドを使います。

```bash
ros2 pkg prefix --share nav2_minimal_tb3_sim
```

上の出力に対して `worlds/tb3_sandbox.sdf.xacro` を連結したものが、未指定時に読み込まれるワールドです。

### ロボットスポーン系

| 引数名 | デフォルト | 説明 |
|---|---|---|
| `robot_name` | `turtlebot3_waffle` | Gazebo内モデル名 |
| `robot_sdf` | `<nav2_minimal_tb3_sim>/urdf/gz_waffle.sdf.xacro` | スポーンするSDF(xacro) |
| `x_pose` | `-2.00` | 初期位置X |
| `y_pose` | `-0.50` | 初期位置Y |
| `z_pose` | `0.01` | 初期位置Z |
| `roll` | `0.00` | 初期姿勢 roll |
| `pitch` | `0.00` | 初期姿勢 pitch |
| `yaw` | `0.00` | 初期姿勢 yaw |

> 注：`pose` は辞書にまとめられ、`LaunchConfiguration('x_pose', default='-2.00')` のように **この箇所でデフォルトが与えられています**。
> 一方で、このファイルでは `x_pose` / `y_pose` / `yaw` などの **DeclareLaunchArgument が存在しません**。
> そのため、`ros2 launch ... x_pose:=...` のようにコマンドラインから上書きできるかは、launchシステムの挙動（未宣言引数の扱い）に依存し、環境によってはエラーになります。
> 確実に上書きしたい場合は、このファイルに `DeclareLaunchArgument('x_pose', ...)` 等を追加するのが安全です。

---

## 処理のポイント（コード解説）

### 1) パス解決：bringup_dir / sim_dir

最初に `get_package_share_directory()` で以下を取得します。

- `bringup_dir = <nav2_bringup share>`
- `sim_dir = <nav2_minimal_tb3_sim share>`

これにより、map/params/rviz/world/urdf/sdf といった「リソースへのフルパス」を組み立てます。

---

### 2) LaunchConfiguration の準備

このファイルでは、引数の値はほぼ `LaunchConfiguration('name')` として受け取り、後段の Node / IncludeLaunchDescription に渡します。

例：

```python
slam = LaunchConfiguration('slam')
use_sim_time = LaunchConfiguration('use_sim_time')
headless = LaunchConfiguration('headless')
```

---

### 3) robot_state_publisher の起動（URDF読み込み）

`nav2_minimal_tb3_sim` に置かれた URDF をファイルとして読み込み、`robot_description` パラメータとして `robot_state_publisher` に渡します。

```python
urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
with open(urdf, 'r') as infp:
    robot_description = infp.read()
```

起動アクションは以下で、`use_robot_state_pub` が true のときのみ起動します。

```python
start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': use_sim_time,
                 'robot_description': robot_description}],
    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
)
```

#### remappings（/tf, /tf_static）

`('/tf', 'tf')` のような remap が入っているのは、名前空間利用時に **絶対パスの /tf が問題になりやすい**ためです。
（一般に tf はグローバルに扱われることが多い一方で、launchで namespace を切ると tf の解決が複雑になりがちです）

---

### 4) RViz と Nav2 bringup は「Include」で委譲

このファイル自身が RViz2 や Nav2 の各ノードを直接並べるのではなく、`nav2_bringup/launch` の標準launchに委譲しています。

#### RViz

```python
rviz_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
    condition=IfCondition(use_rviz),
    launch_arguments={
        'namespace': namespace,
        'use_namespace': use_namespace,
        'use_sim_time': use_sim_time,
        'rviz_config': rviz_config_file,
    }.items(),
)
```

#### Nav2 bringup

```python
bringup_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
    launch_arguments={
        'namespace': namespace,
        'use_namespace': use_namespace,
        'slam': slam,
        'map': map_yaml_file,
        'use_sim_time': use_sim_time,
        'params_file': params_file,
        'autostart': autostart,
        'use_composition': use_composition,
        'use_respawn': use_respawn,
    }.items(),
)
```

---

### 5) world が SDF.xacro であることによる「一時ファイル化」

ここがこのlaunchの重要ポイントです。

コメントにもある通り、ワールドを xacro で条件分岐（headlessでプラグイン有無を変える等）したい一方で、Gazebo の CLI は **ワールドをSDF文字列で受け取れず、ファイルパスが必要**です。

そのため、

1. `tempfile.mktemp()` で一時SDFファイル名を作る
2. `xacro -o <temp.sdf> headless:=... <world.sdf.xacro>` で実ファイルを生成
3. Gazebo は `<temp.sdf>` を読み込んで起動
4. launch終了時に一時ファイルを削除

という流れになっています。

```python
world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')

world_sdf_xacro = ExecuteProcess(
    cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])
```

#### 終了時の後片付け（OnShutdown + OpaqueFunction）

```python
remove_temp_sdf_file = RegisterEventHandler(
    event_handler=OnShutdown(
        on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))]
    )
)
```

`OnShutdown` を使うことで、launch停止時に一時ファイルが残るのを防ぎます。

> 補足：`tempfile.mktemp()` は競合リスクの観点で推奨されないことも多いですが、ここでは「Gazeboがファイルパスを要求する」「launch実行中に短命で使う」用途で割り切って使われています。

---

### 6) Gazebo Sim（server）起動：gz sim -r -s

Gazebo の物理シミュレーション側（server）を `ExecuteProcess` で直接起動します。

```python
gazebo_server = ExecuteProcess(
    cmd=['gz', 'sim', '-r', '-s', world_sdf],
    output='screen',
    condition=IfCondition(use_simulator)
)
```

- `-r`：起動後すぐに実行（run）状態にする
- `-s`：serverモード（物理計算側）

---

### 7) Gazebo Sim（client / GUI）起動：gz_sim.launch.py をInclude

GUI（client）側は `ros_gz_sim/launch/gz_sim.launch.py` に委譲しています。

```python
gazebo_client = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    ),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
    launch_arguments={'gz_args': ['-v4 -g ']}.items(),
)
```

ポイント：

- `use_simulator == true` かつ `headless == false` のときのみ起動
- `gz_args` として `-v4 -g` を渡している（詳細ログ + GUI起動）

`gz_sim.launch.py` 自体の役割や headless との関係は、別記事（`gz_sim_launch_analysis.md`）も参照してください。

---

### 8) ロボットのスポーン：spawn_tb3.launch.py をInclude

ロボットのスポーンと ROS-Gazebo ブリッジは、`nav2_minimal_tb3_sim/launch/spawn_tb3.launch.py` に委譲しています。

```python
gz_robot = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(sim_dir, 'launch', 'spawn_tb3.launch.py')),
    launch_arguments={
        'namespace': namespace,
        'use_sim_time': use_sim_time,
        'robot_name': robot_name,
        'robot_sdf': robot_sdf,
        'x_pose': pose['x'], 'y_pose': pose['y'], 'z_pose': pose['z'],
        'roll': pose['R'], 'pitch': pose['P'], 'yaw': pose['Y'],
    }.items(),
)
```

`spawn_tb3.launch.py` が内部で何をするか（`ros_gz_bridge` / `ros_gz_sim create` 等）は、別記事（`spawn_tb3_launch_analysis.md`）を参照してください。

---

## 実行フロー（LaunchDescriptionに積まれる順）

最後に `LaunchDescription()` に action を追加していきます。

重要なのは、**LaunchArgument宣言の後に**、ワールド生成→後片付けハンドラ→スポーン→Gazebo起動→Nav2/RViz起動、の順で登録されている点です。

登録順（抜粋）：

1. Launch引数（DeclareLaunchArgument）一式
2. `world_sdf_xacro`（xacroで一時SDF生成）
3. `remove_temp_sdf_file`（終了時削除）
4. `gz_robot`（TB3スポーン + bridge）
5. `gazebo_server`（gz sim server）
6. `gazebo_client`（GUI、条件付き）
7. `robot_state_publisher`（条件付き）
8. `rviz_cmd`（条件付き）
9. `bringup_cmd`（Nav2 bringup）

> 注：アクションの「登録順」は重要なヒントになりますが、厳密な実行順序はアクション種別（Process起動、Includeの中身、条件、依存関係）にも左右されます。特に `ExecuteProcess` や `IncludeLaunchDescription` は並列に立ち上がるため、必要なら `TimerAction` や event handler による順序制御を検討します。

---

## 使い方例

### 例1：最小（headless、Nav2 + Gazebo serverのみ）

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=False
```

### 例2：GUIあり（Gazebo client + RViz も起動）

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False use_rviz:=True
```

### 例3：初期位置・姿勢を変更

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py \
  headless:=False \
  x_pose:=0.0 y_pose:=0.0 yaw:=1.57
```

### 例4：SLAMを有効化

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True headless:=False
```

### 例5：world を差し替える（別ワールドで起動）

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py \
  headless:=False \
  world:=/absolute/path/to/your_world.sdf
```

ワールドが `.sdf.xacro` の場合でも、このlaunchは内部で `xacro -o <temp.sdf> ...` を実行してSDFを生成するため、そのまま指定できます。

> 注意：`world` は「フルパス」を期待しています（descriptionにも `Full path` とある通り）。相対パスだと期待通りに見つからないことがあります。

---

## 注意点・ハマりどころ

### headless の意味が「GUIを起動しない」

`headless:=True` だと `gazebo_client`（GUI）は起動しません。一方で `gazebo_server` は `use_simulator` に依存して起動します。

### world は xacro を経由するため、一時SDFが生成される

launch実行中に `nav2_XXXX.sdf` のような一時ファイルが生成されます。終了時に削除される設計ですが、異常終了時に残る可能性はあります。

### pose引数は DeclareLaunchArgument で宣言されていない

`x_pose` などは `LaunchConfiguration('x_pose', default=...)` でデフォルトが設定されています。
一方で、このファイルでは `x_pose` / `y_pose` / `yaw` などの `DeclareLaunchArgument` が無いため、コマンドラインから確実に上書きしたい場合は宣言追加が安全です。

---

## まとめ

`tb3_simulation_launch.py` は、Nav2 + TB3 + Gazebo Sim を一括で起動する統合launchで、特に以下がポイントです。

- Gazebo world を **SDF.xacro → 一時SDFファイル** に変換してから起動する
- Gazebo は server/client を分け、`headless` で GUI起動有無を切り替える
- TB3スポーンとブリッジは `spawn_tb3.launch.py`、GUI起動は `gz_sim.launch.py` に委譲して責務分割している
- Nav2 / RViz は `nav2_bringup` の標準launchに委譲しており、拡張・差し替えがしやすい

この構造を理解しておくと、「どこを差し替えればワールドやロボット、Nav2設定を変えられるか」「headless CI をどう作るか」といった設計判断がしやすくなります。
