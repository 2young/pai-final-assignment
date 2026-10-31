# 日本語指示からA*経路探索で迷路内を自律移動

Apple Silicon / Docker Desktop / Ubuntu 24.04 ARM64 / ROS 2 Jazzy / Gazebo Harmonic。
日本語をローカルの文字n-gram TF-IDF＋LinearSVCで分類し、青い差動二輪車両が登録された赤い箱まで移動し、最後は低速で接触して停止します。
AIは意図分類、A*は経路計算、P制御は走行を担当します。色・名前・サイズは登録データを使い、物体の現在位置はGazeboから取得します。画像認識は行いません。

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

## 箱を移動してから指示する

今回の更新を反映する際は、起動中の迷路をターミナルAでCtrl+C終了し、上の起動コマンドで再起動してください。Dockerの再ビルド・分類器の再学習は不要です。接触センサーを追加したため、この更新の初回は迷路の再起動が必要です。

1. Gazeboの赤い箱（モデル名`red_box`）を選択します。
2. 移動ツールまたはPose編集で、X/Y座標を変更します。例として`x=-5, y=3`へ移動できます。箱の中心のZ座標は`0.4m`を維持してください。
3. 水平面内の回転（yaw）は可能です。箱は立てたまま配置し、壁・車両への重なりや地図外への配置を避けます。壁は動かさないでください。
4. Gazeboを再生状態にし、移動操作を終えてからターミナルBで次のコマンドを実行します。

ターミナルB：

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/app/robot_control.py "赤いオブジェクトまで行ってください"'
```

`Live object: red_box, center=[...]`に移動後の座標が出ることを確認してください。
箱は再配置できますが、安全な接近地点・経路・最後の接触区間があることが到達条件です。

分類、ロボットと箱の現在位置、接近ゴール、ウェイポイント、各点の到達誤差を出力します。
`ARRIVED`はA*の接近地点への到着で、許容誤差は0.10mです。その後に低速接近し、**`CONTACT_REACHED`が最終的な接触完了**を示します。
現在位置から毎回計画するため、2回目の指示でも初期位置を仮定しません。
同時に別の速度指令を送るプロセスを動かさないでください。

指示のたびに全登録オブジェクトの位置を読み直し、地図を作り直します。箱の元の位置は占有地図から取り除きます。
走行中に箱が計画時から5cm超移動、または姿勢が3度超変化した場合は停止します。
追いかけ続ける機能はなく、箱を置き直して同じコマンドを再実行します。
箱の位置情報が取得できない場合は停止し、初期座標では走行しません。

## 接触して停止するゴール

既定の`config/maze.json`は`"goal_mode": "contact"`です。通常の日本語コマンドで次の順に実行します。

1. A*で箱の手前の安全な接近地点へ移動し、停止します。
2. 箱の中心へ向きを合わせ、0.10m/sで前進します。
3. 箱の接触センサーが`vehicle_blue`との接触を検知したら、ゼロ速度を送って終了します。

箱は静的モデルのままです。最後の接近区間では対象の箱への接触を許可し、壁・他の箱・地図境界は安全余裕0.25mを含む車両の矩形外形で確認します。
車両外形はワールド生成時に実際のコリジョン形状を含むか検算します。
箱の位置喪失・移動、接近方向のずれ、進捗なし3秒、全体60秒、接近距離上限に達した場合は中断して停止します。
接触センサーのGazebo publisherまたはROS bridgeがない場合も走行しません。
距離だけで接触成功とは判断しません。

従来の手前停止に戻す場合は、コマンドに`--goal-mode stand_off`を追加します。

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/app/robot_control.py --goal-mode stand_off "赤いオブジェクトまで行ってください"'
```

既定値を戻す場合は`config/maze.json`の`goal_mode`を`stand_off`へ変更します。`--goal-mode contact`でも接触モードを明示できます。
接触直後の再指示では、向きと障害物の余裕が適切なら低速接近から再開し、新しい接触を確認します。近距離で向きが合わない場合は停止し、箱やロボットを配置し直してください。

## 座標指定

座標指定も可能です（壁と箱を避けた自由空間内の座標を指定）。座標指定では接触区間を実行しません。


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
| `config/maze.json` | 固定壁・箱の登録情報と初期配置・車体寸法 |
| `app/generate_world.py` | インストール済み公式車両を残し、設定から迷路SDFを生成 |
| `app/intent_model.py` | ローカルAI分類、停止語の優先処理 |
| `app/target_resolver.py` | 色・名前の照合、到達可能な接近地点を選択 |
| `app/grid_map.py` | 膨張済み占有格子と安全な線分の確認 |
| `app/path_planner.py` | 4近傍A*、マンハッタン距離、経路復元・直線区間の圧縮 |
| `app/pose_provider.py` | 車両・登録オブジェクトのワールド位置、姿勢、鮮度確認 |
| `app/object_tracker.py` | 実行時位置のスナップショット、配置検査、走行中の移動検知 |
| `app/motion_config.py` | 車両と追従制御で共通の速度設定・値の検査 |
| `app/contact_goal.py` | 接触区間の計画、車両の矩形外形、接触検知と低速停止 |
| `app/navigator.py` | 低速の位置フィードバック、到達・停滞・停止処理 |
| `app/robot_control.py` | 自然言語と座標指定のCLI、従来操作との統合 |
| `scripts/run_maze.sh` | SDF生成、GazeboとROS bridgeの起動・終了 |

地図は16m四方、範囲`[-8,8] × [-8,8]`、セル幅0.1m。境界は計画上の境界です。
ワールド座標`(x,y)`から`(floor((x+8)/0.1), floor((y+8)/0.1))`へ変換し、
セル中心は`(-8+(i+0.5)*0.1, -8+(j+0.5)*0.1)`です。
車両初期位置は`(-5,-4, yaw=0)`。赤い箱の初期中心は`(5,4)`です。自律走行ではGazebo上の現在位置を使います。

公式車両は車体長が約2mあり、駆動輪中央と元のモデル原点が約0.55mずれています。
生成時に全リンクを平行移動し、迷路用車両の原点を駆動輪中央に合わせます。
保守的な包絡円半径1.85mに安全余裕0.25mを足して、壁・箱・境界を膨張します。
セル中心だけでなくセル全体を保守的に占有判定します。A*は箱中心ではなく、膨張領域からさらに0.35m離れた上下左右の接近候補を使います。接触モードでは格子境界で候補が塞がる場合に0.25m/0.45mの近傍候補も調べ、接触まで車体が安全に通れる地点を選びます。
最大前進速度0.7m/s、角速度0.5rad/sです。ゴールへ近づくと減速し、大きな角度誤差がある間はその場で旋回します。

## 走行速度の設定

`config/maze.json`の`navigation`で速度を変更できます。追従制御とGazebo車両の上限の両方に反映します。

```json
"navigation": {
  "max_linear_speed": 0.7,
  "max_angular_speed": 0.5,
  "distance_gain": 0.8,
  "heading_gain": 1.8
}
```

単位は直進がm/s、旋回がrad/sです。直進上限は以前の2倍です。旋回速度と接近時の制御は従来値を維持しており、
経由点での減速・旋回があるため、到着までの時間がそのまま半分になるわけではありません。
`distance_gain`は接近時の速度、`heading_gain`は旋回の応答を調整します。
以前の低速設定に戻す場合は、順に`0.35`、`0.5`、`0.8`、`1.8`を指定します。

設定変更後は、迷路を起動したターミナルでCtrl+Cを押して終了し、`run_maze.sh`を一度だけ実行し直してください。
Dockerの再起動・再ビルドは不要です。起動中の迷路を終了する前に別の迷路を起動しないでください。
車両側の速度上限は迷路内の手動操作にも適用されます。
到着ログには走行制御開始からの`elapsed`（実時間の秒）を表示します。
HEADLESSの同じ15.82mの経路では、手前停止モードのコマンド全体が約101秒から約84秒へ短縮しました（各1回の比較）。接触モードではさらに低速の接触区間が加わります。詳細は[検証記録](VALIDATION.md)を参照してください。

## 位置トピックの確認

専用ワールドに`PosePublisher`を追加し、`vehicle_blue`と登録オブジェクトのワールドposeを30Hzで配信します。
`/model/vehicle_blue/pose`のGazebo `gz.msgs.Pose_V`をROS `tf2_msgs/msg/TFMessage`へブリッジします。
赤い箱の`/model/red_box/pose`も同じ型でブリッジし、`frame_id=maze`と各モデルの`child_frame_id`を照合します。
静的な箱にも現在時刻付きのposeが配信されるため、移動後の位置と受信の鮮度を確認できます。車輪オドメトリの原点や積分誤差には依存しません。
箱が回転している場合は、回転した箱を包むワールド軸に平行な矩形で地図と接近候補を計算します。

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic list -t'
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic echo /model/vehicle_blue/pose --once'
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic echo /model/red_box/pose --once'
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && gz topic -i -t /model/vehicle_blue/pose'
```

箱の`/model/red_box/contact`はGazebo `gz.msgs.Contacts`からROS `ros_gz_interfaces/msg/Contacts`へブリッジします。接触がある時だけ配信され、箱とロボットのコリジョンの組み合わせを照合します。

購読callbackは制御周期で`rclpy.spin_once`を呼んで処理します。
位置更新が1.5秒途絶えた場合は停止。初期位置待機10秒、全体240秒、進捗なし25秒で中断します。
未知色・複数対象・曖昧な対象は指示バッチを実行せず停止します。
`--dry-run`は分類と対象参照のみで、位置取得や走行成功を検証するものではありません。

## 制限と検証

壁は既知で固定とし、登録された箱は走行開始前に移動できます。Nav2、SLAM、外部API、大規模LLMは使用しません。
SIGKILL・OS停止・通信切断のようにfinallyを実行できない停止に対する独立watchdogはありません。
地図設定を変更した場合はワールドと走行プロセスを両方再起動してください。

単体テストには迂回、行き止まり、到達不能、車体より狭い通路、無効座標、対象不明、
位置の鮮度、理想運動モデルでの追従、SDFと配置定義の一致、既存日本語分類の回帰を含めます。
Gazeboの物理シミュレーション結果は、単体テストとは区別して[検証記録](VALIDATION.md)に記載しています。
単体テストとHEADLESSでの走行結果は検証記録を参照してください。移動後の占有・回転・無効配置・対象移動時の停止もテスト対象です。

## トラブルシューティング

- 車両が表示されない：`docker compose logs sim`とターミナルAの出力を確認。ソフトウェア描画のためDockerメモリは8GB以上を推奨します。
- `No subscribers`：ターミナルAのbridgeが起動しているか確認します。
- `No world pose`：公式デモではなく専用迷路を起動し、上のトピックを確認します。
- `No fresh object world pose`：迷路を再起動し、`/model/red_box/pose`を確認してください。初期座標への自動切替はしません。
- `object moved during navigation`：箱の配置を確定してコマンドを再実行してください。
- `overlaps` / `outside field` / `place object upright`：箱を地図内の地面上へ配置し直してください。
- `No Gazebo contact sensor publisher` / `No ROS contact bridge`：更新後の`run_maze.sh`で迷路を再起動してください。
- `No reachable approach point with safe contact corridor`：手前への経路または最後の接触区間が塞がっています。箱を配置し直すか`--goal-mode stand_off`を試してください。
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
