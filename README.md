# 日本語指示からA*経路探索で迷路内を自律移動

Apple Silicon / Docker Desktop / Ubuntu 24.04 ARM64 / ROS 2 Jazzy / Gazebo Harmonic。
日本語をローカルの文字n-gram TF-IDF＋LinearSVCで分類し、登録された赤い箱の手前まで青い差動二輪車両が移動します。
AIは意図分類、A*は経路計算、P制御は走行を担当します。色と物体の位置は登録データであり、画像認識は行いません。

## 起動

Docker Desktopを起動し、このリポジトリのディレクトリで実行します。初回はROS/Gazeboのダウンロードに時間がかかります。

```sh
docker compose up -d --build
docker compose exec sim python3 /workspace/app/train.py
docker compose exec sim python3 -m pytest -q /workspace/tests
```

学習例に`GO_TO_OBJECT`を追加しているため、以前のモデルは必ず再学習してください。
ホスト側のPythonには依存せず、上のコンテナ内コマンドを使います。

ターミナルA（実行を維持）：

```sh
docker compose exec sim bash /workspace/scripts/run_maze.sh
```

ブラウザで http://localhost:6080/vnc.html を開いてConnectを押します。
物理シミュレーションのみの検証では、起動コマンドを次に変更できます。

```sh
docker compose exec -e HEADLESS=1 sim bash /workspace/scripts/run_maze.sh
```

ターミナルB：

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/app/robot_control.py "赤いオブジェクトまで行ってください"'
```

分類、現在位置、接近ゴール、ウェイポイント、各点の到達誤差、`ARRIVED`と走行中の最小車体離隔を出力します。
最終位置の許容誤差は0.10m。現在位置から毎回計画するため、2回目の指示でも初期位置を仮定しません。
同時に別の速度指令を送るプロセスを動かさないでください。

座標指定も可能です（壁と箱を避けた自由空間内の座標を指定）：

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/app/robot_control.py --goal -5.5 4.5'
```

停止は実行中ターミナルBでCtrl+C。例外・位置喪失・経路なし・停滞・タイムアウト時もゼロTwistを送ります。
別ターミナルから明示的に停止する場合は、まず走行プロセスをCtrl+Cで終了してから送信します。

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/app/robot_control.py "ストップして"'
```

迷路のリセットはターミナルAをCtrl+Cで終了し、同じ起動コマンドを再実行します。

## 従来の基本操作

公式デモを使う場合は迷路を終了してから、ターミナルAで起動します。

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 launch ros_gz_sim_demos diff_drive.launch.py rviz:=false'
```

```sh
docker compose exec sim python3 /workspace/app/robot_control.py --dry-run --demo
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/app/robot_control.py "前に進んで" "左に向いて" "ストップして"'
```

基本動作の時間制御は従来どおりです。迷路内での手動前進は障害物回避をしません。
目的地への自律移動には専用の迷路起動が必要です。

## 構成と座標

| ファイル | 役割 |
|---|---|
| `config/maze.json` | 壁・箱・初期位置・車体半径・地図の唯一の配置定義 |
| `app/generate_world.py` | インストール済み公式車両を残し、設定から迷路SDFを生成 |
| `app/intent_model.py` | ローカルAI分類、停止語の優先処理 |
| `app/target_resolver.py` | 色・名前の照合、到達可能な接近地点を選択 |
| `app/grid_map.py` | 膨張済み占有格子と安全な線分の確認 |
| `app/path_planner.py` | 4近傍A*、マンハッタン距離、経路復元・直線区間の圧縮 |
| `app/pose_provider.py` | Gazeboワールド位置の購読、Quaternion→yaw、鮮度確認 |
| `app/navigator.py` | 低速の位置フィードバック、到達・停滞・停止処理 |
| `app/robot_control.py` | 自然言語と座標指定のCLI、従来操作との統合 |
| `scripts/run_maze.sh` | SDF生成、GazeboとROS bridgeの起動・終了 |

地図は16m四方、範囲`[-8,8] × [-8,8]`、セル幅0.1m。境界は計画上の境界です。
ワールド座標`(x,y)`から`(floor((x+8)/0.1), floor((y+8)/0.1))`へ変換し、
セル中心は`(-8+(i+0.5)*0.1, -8+(j+0.5)*0.1)`です。
車両初期位置は`(-5,-4, yaw=0)`。赤い箱の中心は`(5,4)`です。

公式車両は車体長が約2mあり、駆動輪中央と元のモデル原点が約0.55mずれています。
生成時に全リンクを平行移動し、迷路用車両の原点を駆動輪中央に合わせます。
保守的な包絡円半径1.85mに安全余裕0.25mを足して、壁・箱・境界を膨張します。
セル中心だけでなくセル全体を保守的に占有判定します。箱中心はゴールにせず、膨張領域からさらに0.35m離れた4候補のうち、到達可能で経路長が短い地点を選びます。
最大前進速度0.35m/s、角速度0.5rad/s。大きな角度誤差がある間はその場で旋回します。

## 位置トピックの確認

専用ワールドに`PosePublisher`を追加し、`vehicle_blue`のワールドposeを30Hzで配信します。
`/model/vehicle_blue/pose`のGazebo `gz.msgs.Pose_V`をROS `tf2_msgs/msg/TFMessage`へブリッジします。
`child_frame_id=vehicle_blue`のみを購読します。車輪オドメトリの原点や積分誤差には依存しません。

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic list -t'
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic echo /model/vehicle_blue/pose --once'
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && gz topic -i -t /model/vehicle_blue/pose'
```

購読callbackは制御周期で`rclpy.spin_once`を呼んで処理します。
位置更新が1.5秒途絶えた場合は停止。初期位置待機10秒、全体240秒、進捗なし25秒で中断します。
未知色・複数対象・曖昧な対象は指示バッチを実行せず停止します。
`--dry-run`は分類と対象参照のみで、位置取得や走行成功を検証するものではありません。

## 制限と検証

既知で静的な障害物のみを扱います。Nav2、SLAM、外部API、大規模LLMは使用しません。
SIGKILL・OS停止・通信切断のようにfinallyを実行できない停止に対する独立watchdogはありません。
地図設定を変更した場合はワールドと走行プロセスを両方再起動してください。

単体テストには迂回、行き止まり、到達不能、車体より狭い通路、無効座標、対象不明、
位置の鮮度、理想運動モデルでの追従、SDFと配置定義の一致、既存日本語分類の回帰を含めます。
Gazeboの物理シミュレーション結果は、単体テストとは区別して[検証記録](VALIDATION.md)に記載しています。
11件の単体テストと、HEADLESSでの赤い箱への到着（誤差約5.4cm、最小包絡円離隔約43cm）を確認済みです。

## トラブルシューティング

- 車両が表示されない：`docker compose logs sim`とターミナルAの出力を確認。ソフトウェア描画のためDockerメモリは8GB以上を推奨します。
- `No subscribers`：ターミナルAのbridgeが起動しているか確認します。
- `No world pose`：公式デモではなく専用迷路を起動し、上のトピックを確認します。
- `Old model`：`train.py`で再学習します。
- タイムアウト：Gazeboの再生状態・実時間速度を確認します。重い場合は`HEADLESS=1`で切り分けます。

## 撮影・提出

日本語指示・AI分類ログ・障害物を迂回する車両・到着停止を1〜3分の動画に収録します。
提出用PDFにはAI分類、A*、ROS制御の担当範囲、実測結果、失敗条件を区別して記載してください。
公開GitHub・動画・PDFの提出およびアップロードはこの実装作業では行いません。

終了：`docker compose down`

参考：
[公式差動二輪車両SDF](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/diff_drive.sdf)、
[ROS bridgeの型対応](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge)。
車両形状・関節・物理設定は公式サンプルを利用し、迷路生成・AI分類・A*・追従を本プロジェクトで実装しています。
