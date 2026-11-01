# gz_sim.launch.py の役割分析

## 概要

`gz_sim.launch.py` は、ROS 2環境でGazebo Simulationの**クライアント側（GUI）**を起動するためのlaunchファイルです。このファイルは主にGazeboのビジュアライゼーション部分を担当し、Gazeboのサーバー・クライアント分離アーキテクチャの「クライアント」部分を起動する役割を持ちます。

## 主要な機能

1. **Gazebo Clientの起動**
   - `gz sim` または `ign gazebo` コマンドをGUIモードで実行
   - バージョンに応じて適切なコマンド（`gz` または `ign`）を選択

2. **環境変数の設定**
   - `GZ_SIM_SYSTEM_PLUGIN_PATH`: プラグインパスの設定
   - `GZ_SIM_RESOURCE_PATH`: モデル・リソースパスの設定

3. **ROSパッケージからのパス自動検出**
   - `GazeboRosPaths`クラスによるROSパッケージからのGazebo関連パスの収集
   - パッケージの`export`タグから`gazebo_model_path`、`plugin_path`、`gazebo_media_path`を検出

4. **バージョン互換性の管理**
   - Ignition GazeboからGazebo Simへの移行をサポート
   - バージョンに応じたコマンド選択ロジック

## tb3_simulation_launch.pyとの関係

`tb3_simulation_launch.py`内では、Gazeboの起動が2段階で行われています：

1. **Gazebo Server（物理シミュレーション）**:
   ```python
   gazebo_server = ExecuteProcess(
       cmd=['gz', 'sim', '-r', '-s', world_sdf],
       output='screen',
       condition=IfCondition(use_simulator)
   )
   ```
   - 直接`gz sim`コマンドを実行
   - `-s`フラグでサーバーモード（物理計算）

2. **Gazebo Client（GUI表示）**:
   ```python
   gazebo_client = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           os.path.join(get_package_share_directory('ros_gz_sim'),
                        'launch',
                        'gz_sim.launch.py')
       ),
       condition=IfCondition(PythonExpression(
           [use_simulator, ' and not ', headless])),
       launch_arguments={'gz_args': ['-v4 -g ']}.items(),
   )
   ```
   - `gz_sim.launch.py`を使用してクライアント起動
   - `-g`フラグでGUIモード
   - `headless=False`の場合のみ実行

## ヘッドレスモードとの関係

`tb3_simulation_launch.py`では、`headless`パラメータによって`gz_sim.launch.py`の実行が制御されています：

- **ヘッドレス実行時（`headless=True`）**:
  - `gz_sim.launch.py`は実行されない
  - Gazebo Serverのみが起動（GUIなし）
  - リソース効率が良く、CI/CDや自動テストに適している

- **GUI実行時（`headless=False`）**:
  - `gz_sim.launch.py`が実行される
  - Gazebo ServerとClientの両方が起動
  - 開発、デバッグ、デモンストレーションに適している

## 実装の詳細

`gz_sim.launch.py`の主要な実装ポイント：

1. **コマンド決定ロジック**:
   ```python
   if len(ign_version) or (ign_version == '' and int(gz_version) < 7):
       exec = 'ruby ' + get_executable_path('ign') + ' gazebo'
   else:
       exec = 'ruby ' + get_executable_path('gz') + ' sim'
   ```

2. **デバッグサポート**:
   ```python
   if debugger != 'false':
       debug_prefix = 'x-terminal-emulator -e gdb -ex run --args'
   ```

3. **環境変数設定**:
   ```python
   env = {
       "GZ_SIM_SYSTEM_PLUGIN_PATH": os.pathsep.join([...]),
       "GZ_SIM_RESOURCE_PATH": os.pathsep.join([...]),
   }
   ```

## まとめ

`gz_sim.launch.py`は、Gazeboのクライアント・サーバーアーキテクチャにおいて、**クライアント側（GUI）の起動を担当する専用launchファイル**です。このファイルは、適切な環境変数設定とともにGazeboのGUIクライアントを起動し、ユーザーがシミュレーション環境を視覚的に操作できるようにします。

ヘッドレスモードでの実行時には使用されず、GUIが必要な場合にのみ呼び出されるという特性を持ちます。これにより、同じlaunchファイル（`tb3_simulation_launch.py`）でGUI有無を柔軟に切り替えることが可能になっています。
