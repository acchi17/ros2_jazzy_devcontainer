# 「Adding models」節の要約 Version3

## 概要

Gazebo Fuelから既存のモデルを使用してワールドにモデルを追加する方法について説明しています。独自モデルを作成する代わりに、Gazebo Fuelにホストされている数百のモデルを簡単にワールドに追加できるとされています。

## 3つの追加方法の詳細比較

### 1. Resource Spawner Plugin使用
**特徴**: **GUI上でのインタラクティブな追加**

#### 1-a. Fuel Resources直接（オンライン方式）
**手順**:
1. GUI右上のプラグインメニューから「Resource Spawner」を選択
2. 右パネルにプラグインが表示（スクロールが必要な場合あり）
3. 'Fuel resources'パネルでリソース読み込み完了を待つ
4. リソース所有者を選択（例: openrobotics）
5. 右パネルでモデル名を検索
6. **雲アイコン**をクリックしてモデルをダウンロード
7. ダウンロード完了後、モデルをクリックしてシミュレーション画面に挿入

**メリット**:
- リアルタイムでシミュレーション中にモデル追加
- 位置を視覚的に確認しながら配置
- SDFファイル編集不要
- 豊富なFuelライブラリに直接アクセス

**デメリット**:
- 一時的な追加（シミュレーション終了で消失）
- インターネット接続必須
- 初回ダウンロード時間が必要

#### 1-b. Local Sources使用（オフライン方式）
**手順**:
1. [app.gazebosim.org/fuel/models](https://app.gazebosim.org/fuel/models)からモデルファイルを事前ダウンロード
2. ファイルを解凍し、ローカルモデルディレクトリに配置（例: `~/my-local-models/model-name`）
3. フォルダ構成確認: `materials`, `meshes`, `model.config`, `model.sdf`
4. 環境変数設定: `export GZ_SIM_RESOURCE_PATH=~/my-local-models/`
5. Gazeboシミュレーター起動、Resource Spawner Plugin追加
6. ローカルリソースからモデルを選択してシミュレーション画面に挿入

**メリット**:
- オフライン環境で動作
- 高速な読み込み（ローカルファイル）
- GUI上でのインタラクティブな配置
- ネットワーク環境に非依存

**デメリット**:
- 一時的な追加（シミュレーション終了で消失）
- 事前ダウンロード作業が必要

### 2. SDF Snippet コピー&ペースト
**特徴**: **SDFファイルにインターネットURL記述**

**手順**:
1. Gazebo Fuel websiteにアクセス
2. 好きなモデルを選択
3. モデル説明ページの**`<>`ボタン**をクリックしてSDFスニペットをクリップボードにコピー
4. カスタムワールドSDFファイルの閉じタグ</world>の直前にペースト

**コード例**:
```xml
<include>
  <uri>
    https://fuel.gazebosim.org/1.0/OpenRobotics/models/Mine Cart Engine
  </uri>
</include>
```

**メリット**:
- 永続的にワールドに保存
- インターネット接続時に自動ダウンロード
- 最新バージョンを自動取得
- 自動化・再現性に適している

**デメリット**:
- 実行時にインターネット接続必須
- 初回ダウンロード時間が必要
- ネットワーク環境に依存

### 3. モデルダウンロード（永続化・ローカル参照）
**特徴**: **ローカル保存後にファイルパス参照**

**手順**:
1. Gazebo Fuel websiteでモデルを選択
2. モデル画像右側の**ダウンロード矢印アイコン**をクリック
3. モデルファイルをダウンロードして永続保存
4. GZ_SIM_RESOURCE_PATH環境変数をモデルの親フォルダに設定
5. SDFファイルで相対パス参照

**コード例**:
```xml
<include>
  <uri>
    model://Mine_Cart_Engine
  </uri>
</include>
```

**環境変数設定例**:
```bash
export GZ_SIM_RESOURCE_PATH="$HOME/world_tutorial"
```

**メリット**:
- オフライン環境で動作
- 高速な読み込み（ローカルファイル）
- バージョン固定で再現性確保
- ネットワーク環境に非依存
- 永続的にワールドに保存

**デメリット**:
- 事前ダウンロード作業が必要
- 手動でのバージョン管理が必要
- ストレージ容量を消費

## 複数インスタンスの配置

同じモデルの複数インスタンスをスポーンする場合は`<name>`タグで異なる名前を付ける必要がある

**コード例**:
```xml
<include>
  <name>Coke0</name>
  <pose>0 0 0 0 0 0</pose>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke</uri>
</include>

<include>
  <name>Coke1</name>
  <pose>0 0.1 0 0 0 0</pose>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke</uri>
</include>
```

## 位置の設定

`<pose>`タグを使用してモデルの座標を設定できる（X Y Z Roll Pitch Yaw）

## 実行方法

```bash
gz sim world_demo.sdf
```

モデルはワールドの原点に表示される

## 用途別推奨方法

### リアルタイム実験・テスト
- **方法1-a**: Resource Spawner + Fuel Resources
- GUI上でのインタラクティブな操作
- シミュレーション実行中の動的追加

### オフライン開発・高速読み込み
- **方法1-b**: Resource Spawner + Local Sources  
- **方法3**: モデルダウンロード（永続化）
- ネットワーク非依存、高速読み込み

### ワールド設計・配布・自動化
- **方法2**: SDF Snippet
- 自動ダウンロード、最新バージョン取得
- CI/CD環境での自動化に適している

### 本格運用・バージョン固定
- **方法3**: モデルダウンロード（永続化）
- 完全にローカル化、バージョン管理可能
- 商用環境での安定運用に最適

## まとめ

**主なポイント**:
- **GUI操作 vs SDFファイル編集**: 方法1はリアルタイム操作、方法2・3は事前設定
- **オンライン vs オフライン**: インターネット依存度の違い
- **一時的 vs 永続的**: シミュレーション終了後の保持有無
- **位置指定**: `<pose>`タグで配置場所を制御
- **複数配置**: `<name>`タグで識別子を設定

ロボットの自律移動シミュレーションでは、開発段階では**方法1**、本格運用では**方法3**が最適と考えられます。

## 参考
- https://gazebosim.org/docs/latest/sdf_worlds/
- https://gazebosim.org/docs/latest/fuel_insert/