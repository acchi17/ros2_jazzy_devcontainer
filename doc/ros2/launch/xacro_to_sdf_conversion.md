# ROS2 LaunchシステムにおけるXACROからSDFへの変換プロセス

このドキュメントでは、`tb3_simulation_launch.py`で行われているXACROファイル（`tb3_sandbox.sdf.xacro`）から一時的なSDFファイルへの変換プロセスと、それがGazeboシミュレーションに渡される仕組みを詳細に解説します。

## 1. 概要

### 目的
ROS2のTurtleBot3シミュレーション環境では、ワールドの設定を柔軟に変更できるようにXACROマクロを使用しています。しかし、Gazeboは直接XACROファイルを読み込むことができないため、実行時にXACROファイルを展開してSDFファイルに変換し、それをGazeboに渡す必要があります。

### 一時ファイルが必要な理由
Gazeboのコマンドライン（`gz sim`）は、SDFの文字列を直接受け取ることができず、ファイルパスを要求します。そのため、XACROで展開したSDFコンテンツを一時ファイルに保存し、そのパスをGazeboに渡す必要があります。

## 2. 処理フローの詳細

### Step 1: 一時ファイルパスの生成
```python
world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
```
- Pythonの`tempfile.mktemp()`を使用して一時ファイルのパスを生成
- プレフィックス「nav2_」とサフィックス「.sdf」を指定
- 例: `/tmp/nav2_abc123.sdf`（Linux）または`C:\Users\username\AppData\Local\Temp\nav2_abc123.sdf`（Windows）

### Step 2: XACROファイルの展開処理
```python
world_sdf_xacro = ExecuteProcess(
    cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])
```
- `xacro`コマンドを実行して、XACROファイルをSDFに変換
- `-o world_sdf`: 出力先を一時ファイルパスに指定
- `['headless:=', headless]`: headlessパラメータを渡す（true/false）
- `world`: 入力XACROファイルのパス（`tb3_sandbox.sdf.xacro`）

### Step 3: Gazeboサーバーでの使用
```python
gazebo_server = ExecuteProcess(
    cmd=['gz', 'sim', '-r', '-s', world_sdf],
    output='screen',
    condition=IfCondition(use_simulator)
)
```
- `gz sim`コマンドを実行してGazeboサーバーを起動
- `-r`: サーバーモードで実行（レンダリングなし）
- `-s`: サーバーのみ起動
- `world_sdf`: 展開された一時SDFファイルのパス

### Step 4: 一時ファイルのクリーンアップ
```python
remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
    on_shutdown=[
        OpaqueFunction(function=lambda _: os.remove(world_sdf))
    ]))
```
- シミュレーション終了時に一時ファイルを自動的に削除
- `OnShutdown`イベントハンドラーを使用
- `os.remove(world_sdf)`で一時ファイルを削除

## 3. コード解析

### 変数の定義と値の変遷

#### world変数（入力XACROファイル）
```python
declare_world_cmd = DeclareLaunchArgument(
    'world',
    default_value=os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro'),
    description='Full path to world model file to load',
)
```
- `sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')`
- 実際の値: `/opt/ros/jazzy/share/nav2_minimal_tb3_sim/worlds/tb3_sandbox.sdf.xacro`

#### world_sdf変数（出力一時ファイル）
```python
world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
```
- 実際の値（例）: `/tmp/nav2_abc123.sdf`（Linux）または`C:\Users\username\AppData\Local\Temp\nav2_abc123.sdf`（Windows）

#### headless変数（XACROパラメータ）
```python
declare_simulator_cmd = DeclareLaunchArgument(
    'headless', default_value='True', description='Whether to execute gzclient)'
)
```
- デフォルト値: `'True'`
- XACROファイル内での条件分岐に使用

### 実行されるコマンドの例

#### XACROコマンド（Linux環境の例）
```bash
xacro -o /tmp/nav2_abc123.sdf headless:=true /opt/ros/jazzy/share/nav2_minimal_tb3_sim/worlds/tb3_sandbox.sdf.xacro
```

#### Gazeboコマンド（Linux環境の例）
```bash
gz sim -r -s /tmp/nav2_abc123.sdf
```

## 4. 技術的背景

### XACROマクロの利点
- **条件付きコンテンツ**: `<xacro:if>`や`<xacro:unless>`を使用して条件分岐
- **パラメータ化**: 実行時にパラメータを変更可能
- **再利用性**: マクロ定義による共通部分の再利用
- **モジュール性**: 複雑なSDFを管理しやすい単位に分割

### Gazeboの制限事項
- SDFコンテンツを文字列として直接渡せない
- ファイルパスのみを受け付ける
- 実行時の動的な設定変更が難しい

### headlessパラメータの影響
`tb3_sandbox.sdf.xacro`内では、headlessパラメータによって以下のプラグインの有無が制御されています：

```xml
<xacro:unless value="$(arg headless)">
  <plugin
    filename="gz-sim-scene-broadcaster-system"
    name="gz::sim::systems::SceneBroadcaster">
  </plugin>
</xacro:unless>
```

- `headless=true`の場合: SceneBroadcasterプラグインは含まれない（GUIなし）
- `headless=false`の場合: SceneBroadcasterプラグインが含まれる（GUI表示）

## 5. 実践的な例

### 実際のファイルパス例（Windows環境）
- 入力XACROファイル: `C:\opt\ros\jazzy\share\nav2_minimal_tb3_sim\worlds\tb3_sandbox.sdf.xacro`
- 一時SDFファイル: `C:\Users\grf31\AppData\Local\Temp\nav2_a1b2c3d4.sdf`

### コマンド実行例（Windows環境）
```bash
# XACROコマンド
xacro -o C:\Users\grf31\AppData\Local\Temp\nav2_a1b2c3d4.sdf headless:=true C:\opt\ros\jazzy\share\nav2_minimal_tb3_sim\worlds\tb3_sandbox.sdf.xacro

# Gazeboコマンド
gz sim -r -s C:\Users\grf31\AppData\Local\Temp\nav2_a1b2c3d4.sdf
```

### デバッグ方法

#### 一時ファイルパスの確認
launchファイルに以下のデバッグ出力を追加：
```python
import tempfile
world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
print(f"DEBUG: Temporary SDF file path: {world_sdf}")
```

#### 一時ファイルの内容確認
シミュレーション実行中に一時ファイルの内容を確認：
```bash
# Linuxの場合
cat /tmp/nav2_*.sdf

# Windowsの場合
type C:\Users\username\AppData\Local\Temp\nav2_*.sdf
```

#### 一時ファイルの保存
自動削除を防ぐために、一時ファイルをコピー：
```bash
# Linuxの場合
cp /tmp/nav2_*.sdf ~/saved_world.sdf

# Windowsの場合
copy C:\Users\username\AppData\Local\Temp\nav2_*.sdf C:\Users\username\saved_world.sdf
```

## まとめ

ROS2 LaunchシステムでのXACROからSDFへの変換プロセスは、以下の4つのステップで構成されています：

1. 一時ファイルパスの生成
2. XACROファイルの展開と一時ファイルへの保存
3. 一時ファイルパスをGazeboに渡してシミュレーション実行
4. シミュレーション終了時の一時ファイル削除

この仕組みにより、XACROの柔軟性とGazeboの要件を両立させ、実行時パラメータに応じて動的にシミュレーション環境を構成することが可能になっています。
