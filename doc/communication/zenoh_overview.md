# Zenohについてのまとめ

## 1. Zenohプロトコルの概要

Zenoh(ゼノ)は、Eclipse Foundation配下で開発されているオープンソースの通信プロトコル/ミドルウェア。「Zero Overhead Network Protocol」の略。

- Pub/Sub通信(データ配送)、クエリ(分散ストレージへのアクセス)、計算(Queryable/RPC的な仕組み)を、**「キー式(key expression)」**という階層的なキー(例: `home/room1/temperature`)を軸に統一的に扱えるのが最大の特徴。
- MQTTやDDSに比べてワイヤーオーバーヘッドが小さく、軽量。
- Serial、Bluetooth、LoRa、TCP/IP、UDP/IP、QUIC、WebSockets、CANbusなど幅広いトランスポートに対応し、サーバーからマイコンまで一気通貫で使えるよう設計されている。
- リファレンス実装はRust。C、C++、Python、Java、Kotlin、TypeScript、Goなど各言語のバインディングが存在する。
- ロボティクス、IoT、自動運転などで採用が進んでおり、特にROS 2の通信ミドルウェア(rmw)としての採用が広がっている。

## 2. プロトコルとしてのZenoh

Zenohという言葉は「仕様」と「実装」の両方を指す(TCP/IPが「RFC仕様」と「各OSのTCP/IPスタック実装」の両方を指すのと同じ構造)。

- 仕様は自然言語+メッセージフォーマットの図表で記述された**Zenoh Protocol Specification**(`spec.zenoh.io`で公開)。
- OSI参照モデル上の位置づけは基本的に**第7層(アプリケーション層)**。
  - 典型的にはTCP/UDP/QUIC(L4)の上で動作。
  - ただし柔軟性が高く、IPやIPv6(L3)の直上、あるいはEthernetやシリアルなどのデータリンク層(L2)の直上にも直接載せられる設計になっている。
  - これを実現しているのが、Zenoh独自の**Session Protocol**(セッション確立・フラグメンテーション・再送制御などをアプリ層内で自前実装した独自の内部レイヤー)。この「セッション層」「トランスポート層」という名称はZenoh内部の呼称であり、OSIのL4/L5そのものではない点に注意。
- DDS(RTPS)との比較: DDSも同じくL7に位置するが、UDP/IPマルチキャストを前提とした設計。Zenohはトランスポートの柔軟性・非マルチキャスト環境への対応を重視している点が異なる。

## 3. 実装としてのZenoh

仕様を実装した具体的なソフトウェアも複数存在する。

| 実装 | 特徴 |
|---|---|
| **zenoh(Rust)** | リファレンス実装。GitHub `eclipse-zenoh/zenoh` |
| **zenoh-pico** | マイコン等リソース制約の厳しい環境向けに、Cで書かれた軽量な独立実装 |
| **zenoh-c / zenoh-cpp** | Rust実装(またはzenoh-pico)をFFI経由で呼び出すC/C++バインディング |
| **zenoh-python / zenoh-java / zenoh-kotlin / zenoh-ts / zenoh-go** | 各言語向けバインディング |
| **zenohd** | Zenohのルーター機能を提供する実行可能デーモン |

既存の(別言語で書かれた)プロトコルスタックへの組み込みパターンは主に3つ。

1. **ライブラリとして直接リンク**: zenoh-c/zenoh-cppなどFFIバインディングを既存アプリにリンクし、API呼び出しを追加する。
2. **zenoh-picoをソース組み込み**: マイコンなど超省リソース環境向けに、Cソースをビルドに組み込む。
3. **zenohdを別プロセスとして起動**: 既存コードを一切変更せず、ネットワーク越しにzenohdへ接続する(プロトコルブリッジもこの一種)。

## 4. Zenohプロトコルの3つのモード

Zenohのセッション(=ノード)は、コンフィグの`mode`パラメータでネットワークトポロジ内での役割を指定する。これはプロトコルの種類を切り替えるものではなく、**セッションが持つ設定属性**であり、ワイヤープロトコル自体は共通。

| モード | 動作 |
|---|---|
| **peer** | 他のpeer/routerと対等に直接通信。マルチキャスト/gossipで自律的に発見し合う |
| **client** | 単独では発見できず、必ずrouterを経由して通信する(routerにぶら下がる形) |
| **router** | 複数のpeer/clientを中継するハブ。gossip情報の集約・転送も担う |

**gossip**とは、ネットワーク上のノードが互いに他ノードの存在情報(アドレス等)を伝聞形式で拡散し合うことで、中央集権的なレジストリなしに全体のトポロジを自律的に把握する、分散システムにおける設計パターンの一種(Zenoh固有の概念ではない)。Zenohでは主にpeer同士の自動発見や、router経由でのノードグラフ伝播に使われる。

## 5. rmw_zenohとは

ROS 2は具体的な通信プロトコルを直接使わず、**rmw(ROS MiddleWare interface)**という抽象化層の上にアプリケーションが書かれている。

```
ROS 2アプリケーション
──────
rcl
──────
rmw(抽象インターフェース)  ← 差し替え可能なポイント
──────
rmw_zenoh_cpp（アダプター）
──────
zenoh-cpp / zenoh-c（FFIバインディング）
──────
zenoh（Rust実装本体）
```

- rmw_zenohは、**Zenohプロトコル自体を実装しているのではなく**、既存のZenoh実装(Rust版、zenoh-cpp経由)をROS 2のrmw APIに繋ぐ**アダプター**。DDSでいう`rmw_fastrtps_cpp`や`rmw_cyclonedds_cpp`と同じ階層に位置する。
- 2025年初頭にリリースされた比較的新しい選択肢。ROS 2 Jazzy以降で利用可能(Humbleは非対応)。2026年時点でのデフォルトはまだDDS(Fast DDS)で、Zenohは代替rmw実装という位置づけ。
- **デフォルト設定ではマルチキャストが無効**になっており、ノード同士の発見にはルーター(`rmw_zenohd`)経由のgossip伝播が必須。そのためノード起動前にルーターを別プロセスとして起動しておく必要がある(ROS 1の`roscore`に近い運用感)。

### rmw_zenohでのモード設定方法

基本的にデフォルトのjson5設定ファイル自体がpeer用・router用に用意されているため、peer/routerへの切り替えに設定ファイルの書き換えは不要。clientだけは明示的な指定が要る。

| モード | 切り替え方法 |
|---|---|
| **peer**(デフォルト) | ROS2ノードのプロセスはデフォルトでこのモードになる(デフォルトのセッション設定ファイル:`share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5`が`peer`の設定になっているため)。ルーターが別ホストにある場合のみ、同ファイルの`connect/endpoints`を`["tcp/<ルーターのIP>:7447"]`に書き換える |
| **router** | `rmw_zenohd`というROS2ノード(`ros2 run rmw_zenoh_cpp rmw_zenohd`で起動)のプロセス専用モード。他のROS2ノードのプロセスをrouterモードにすることは基本的にない。このモード用の設定ファイルはルータ設定ファイルと呼ばれる(デフォルトのルータ設定ファイル:`share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5`) |
| **client** | ROS2ノードを起動するプロセスに`ZENOH_CONFIG_OVERRIDE`環境変数を設定してから起動する。デフォルトのセッション設定ファイルは書き換えず、環境変数でそのプロセスだけ設定を上書きするのが推奨される(下記参照) |

client例:

```bash
ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<ルーターのIP>:7447"]' ros2 run demo_nodes_cpp listener
```

補足: 恒久的に設定を変えたい場合は、デフォルトのセッション/ルータ設定ファイルをコピーして設定を書き換え、`ZENOH_SESSION_CONFIG_URI`(セッション設定ファイル用)や`ZENOH_ROUTER_CONFIG_URI`(ルーター設定ファイル用)という環境変数でそのファイルパスを指定する方法もある。

### rmw_zenohでのモードの使い分け

| コンポーネント | モード | 備考 |
|---|---|---|
| 通常のROS 2ノードを実行するプロセス(デフォルト) | peer | マルチキャストは無効、gossip経由でルーターに依存 |
| `rmw_zenohd` | router | `tcp/[::]:7447`で全インターフェース待受 |
| 別ホストのルーターに直接繋ぐノードを実行するプロセス | client | `ZENOH_CONFIG_OVERRIDE`等で明示的に接続先を指定 |

### 接続先の指定方法

- 同一ホスト内: ノードはループバック経由でルーターの`tcp/localhost:7447`に接続(デフォルト)。
- 別ホストのノードから接続: `ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<ルーターのIP>:7447"]'` のように実IPを明示。
- ホスト同士(ルーター同士)を接続: 片方のルーター設定の`connect.endpoints`にもう一方のルーターのIP:7447を指定すると、双方向にノードグラフが伝播する。
