# Gazeboモデルのディレクトリ構成

このドキュメントでは、Gazeboシミュレーターで使用されるモデルの標準的なディレクトリ構成について説明します。

## 1. Gazeboにおけるモデルとワールドの基本概念

**モデル（Model）**:
- **個別のオブジェクト**を定義します
- 例えば、ロボット、家具、建物などの単一のエンティティ
- 他のモデルから独立して存在し、再利用可能

**ワールド（World）**:
- **シミュレーション環境全体**を定義します
- 複数のモデル、地面、照明、物理設定などを含みます
- 通常は`.world`ファイルや`.sdf`ファイル（ワールド用）で定義されます

## 2. 標準的なディレクトリ構成

Gazeboモデルの標準的なディレクトリ構成は以下の通りです：

```
model_name/
├── model.config      # モデルのメタデータ
├── model.sdf         # メインのSDF記述ファイル（SDF 1.5以降用）
├── model-1_4.sdf     # 古いバージョン用のSDF記述ファイル（オプション）
└── meshes/           # 3Dメッシュファイル（.dae, .stl等）
    ├── mesh1.dae
    └── mesh2.dae
```

場合によっては、以下のディレクトリも含まれることがあります：

```
model_name/
├── materials/        # テクスチャや材質情報
│   ├── textures/     # テクスチャ画像
│   └── scripts/      # マテリアルスクリプト
└── plugins/          # モデル固有のプラグイン
```

## 3. 各ファイルの役割と説明

### model.config

`model.config`ファイルはXML形式で、モデルに関するメタデータを提供します：

```xml
<?xml version="1.0"?>
<model>
  <name>モデル名</name>
  <version>バージョン番号</version>
  <sdf version="1.5">model.sdf</sdf>
  <sdf version="1.4">model-1_4.sdf</sdf>

  <author>
    <name>作成者名</name>
    <email>メールアドレス</email>
  </author>

  <description>
    モデルの説明
  </description>
</model>
```

主な要素：
- `<name>`: モデルの名前
- `<version>`: モデルのバージョン
- `<sdf>`: 対応するSDFバージョンとファイル名
- `<author>`: 作成者情報
- `<description>`: モデルの説明

### model.sdf

`model.sdf`ファイルはSDFormat（Simulation Description Format）で記述され、モデルの物理的・視覚的特性を定義します：

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="モデル名">
    <static>true/false</static>
    <link name="リンク名">
      <collision name="collision">
        <!-- 衝突検出用の形状定義 -->
      </collision>
      <visual name="visual">
        <!-- 視覚表現の定義 -->
      </visual>
      <inertial>
        <!-- 慣性特性の定義 -->
      </inertial>
    </link>
    <!-- 複数のリンクとジョイントを定義可能 -->
  </model>
</sdf>
```

### meshes/

`meshes/`ディレクトリには、モデルの3D形状を定義するメッシュファイルが格納されます：
- `.dae`（COLLADA）: 最も一般的に使用される形式
- `.stl`: 単純な形状に使用されることもある
- その他の形式（`.obj`など）

これらのメッシュは、SDFファイル内で以下のように参照されます：

```xml
<visual name="visual">
  <geometry>
    <mesh>
      <uri>model://model_name/meshes/mesh_name.dae</uri>
      <scale>1.0 1.0 1.0</scale>
    </mesh>
  </geometry>
</visual>
```

## 4. turtlebot3_worldの実例分析

`turtlebot3_world`ディレクトリは、標準的なGazeboモデル構成に従っています：

```
turtlebot3_world/
├── model.config       # モデルのメタデータ
├── model.sdf          # SDF 1.5用の定義ファイル
├── model-1_4.sdf      # SDF 1.4用の定義ファイル
└── meshes/            # 3Dメッシュファイル
    ├── hexagon.dae    # 六角形のメッシュ
    └── wall.dae       # 壁のメッシュ
```

### model.config の内容

```xml
<?xml version="1.0"?>
<model>
  <name>TurtleBot3 World</name>
  <version>1.0</version>
  <sdf version="1.4">model-1_4.sdf</sdf>
  <sdf version="1.5">model.sdf</sdf>  

  <author>
    <name>Taehun Lim(Darby)</name>
    <email>thlim@robotis.com</email>
  </author>

  <description>
    World of TurtleBot3 with ROS symbol
  </description>
</model>
```

### model.sdf の特徴

`turtlebot3_world`のSDFファイルでは：
- ROSロゴを模した形状（円柱と六角形）を定義
- 静的モデル（`<static>1</static>`）として設定
- メッシュファイル（hexagon.daeとwall.dae）を参照

## 5. モデルの参照方法

Gazeboでは、モデルを参照するために`model://`プロトコルを使用します：

### SDFファイル内での参照

```xml
<uri>model://model_name/meshes/mesh_name.dae</uri>
```

### ワールドファイルでのモデル読み込み

```xml
<include>
  <uri>model://model_name</uri>
  <pose>0 0 0 0 0 0</pose>
</include>
```

### 環境変数の設定

モデルを見つけるために、Gazeboは`GZ_SIM_RESOURCE_PATH`環境変数を使用します：

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/my_models:$GZ_SIM_RESOURCE_PATH
```

## 6. 実践的な使用例

### 既存モデルの使用

1. **Gazebo Fuelからのモデル使用**:
   ```xml
   <include>
     <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/モデル名</uri>
   </include>
   ```

2. **ローカルモデルの使用**:
   ```xml
   <include>
     <uri>model://モデル名</uri>
   </include>
   ```

### 独自モデルの作成

1. 適切なディレクトリ構造を作成
2. `model.config`ファイルを作成
3. `model.sdf`ファイルでモデルを定義
4. 必要に応じてメッシュファイルを`meshes/`ディレクトリに配置
5. 環境変数を設定して、Gazeboがモデルを見つけられるようにする

## 7. 参考資料

- [Building Your Own Robot](https://gazebosim.org/docs/harmonic/building_robot)
- [SDF Worlds](https://gazebosim.org/docs/harmonic/sdf_worlds)
- [Model Insertion from Fuel](https://gazebosim.org/docs/harmonic/fuel_insert)
- [SDFormat公式サイト](http://sdformat.org)
- [Gazebo Fuel](https://app.gazebosim.org/fuel)
