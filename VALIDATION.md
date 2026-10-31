# 検証記録（2026-10-31 JST）

## 斜め走行と旋回削減（2026-10-31 JST）

上下左右のA*を8近傍へ変更し、任意角度の直線で接続できる候補グラフ上で、
方向転換回数・経路長・総回転角をこの順に優先して経路を選ぶようにしました。
現在の車体の向き、接触直前に必要な向きも評価します。
接近地点も8方向と、現在位置・向きに沿った候補へ拡張しています。

### 同じ初期配置での実測比較

HEADLESSで各回の迷路を初期化し、ロボット`(-5,-4,yaw=0)`から箱`(5,4,yaw=0)`への接触完了までを計測しました。
変更前のコードでbaselineを測定した後、変更後のコードで同じ試験を実施しています。

| 指標 | 変更前 | 変更後 |
|---|---|---|
| 計画経路長（接近地点まで） | 約15.82m | 約15.11m |
| 接近地点までの経由点数（開始点を除く） | 8 | 3 |
| その場旋回の指令区間数 | 9回 | 4回 |
| ワールド姿勢からの総回転角 | 911.4度 | 203.7度 |
| コマンド全体の実時間（接触完了まで） | 106.127秒 | 69.513秒 |
| 壁・境界への最小包絡円離隔下限 | 0.438m | 0.526m |
| 実際の箱・シャーシ接触 | 確認 | 確認 |
| 最後の速度指令 | `(0,0)` | `(0,0)` |

時間は約34.5%短縮しました。各1回の比較で、環境の負荷・配置によって変わります。
離隔の比較では、意図的に接触する箱を除外し、壁と地図境界に対する半径1.85mの包絡円の下限を計算しています。
角度と旋回回数は別ノードによる独立購読で計測しました。
その場旋回は線速度ゼロかつ角速度の絶対値が0.02rad/sを超える指令の連続区間です。

変更後の計画・結果ログ：

```text
planned_turns=4, planned_yaw=197.1deg
waypoints≈[(-5,-4),(-4.45,4.35),(0.65,4.35),(2.25,4)]
ARRIVED: error=0.094m, minimum_footprint_clearance=0.526m,
         elapsed=40.2s, rotation_episodes=3, total_yaw=188.6deg
CONTACT_REACHED: collisions=('red_box::body::collision','vehicle_blue::chassis::collision'),
                 rotation_episodes=1, total_yaw=14.7deg
NAVIGATION_SUMMARY: planned_turns=4, rotation_episodes=4, total_yaw=203.3deg
```

本体ログと独立購読の回転角の差は、計測開始・終了時点とサンプリングの差です。

再計測用スクリプト（初期配置の迷路を起動してから実行）：

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/tests/gazebo_route_benchmark.py --label current'
```

`--label`は記録名であり、旧・新アルゴリズムを切り替えるものではありません。

### 斜めの接触区間と回転した箱

箱を`(-4.5,3,yaw=45度)`へ移動し、ロボットを初期位置から実行しました。
接近経路は約4.14mの1本の斜め直線となりました。

```text
waypoints≈[(-5,-4),(-4.7053,0.1257)]
planned_turns=1, planned_yaw=85.9deg
ARRIVED: error=0.090m, elapsed=11.7s, rotation_episodes=1
CONTACT_REACHED: pose≈(-4.5514,2.2787,1.4996), rotation_episodes=0
NAVIGATION_SUMMARY: rotation_episodes=1, total_yaw=88.3deg
PASS: contacts=('red_box::body::collision','vehicle_blue::chassis::collision'),
      final Twist=(0.0,0.0), stationary_drift=0.000000m
```

箱へ向くための追加のその場旋回をせず、斜めに接触して停止しました。
接触の独立購読、最後のゼロ速度指令、停止後1秒の静止を確認しました。

### 単体テスト・安全停止

```sh
docker compose exec sim python3 -m pytest -q /workspace/tests
# 67 passed in 1.46s
```

8近傍の距離と角抜け禁止、格子の端・角への接触禁止、独立した線分・矩形交差判定との100ケース比較、
任意角度の直線、開始・終端姿勢による経路選択、候補経路の全列挙との最適性比較、回転計測、無効入力を検証しました。
既存の分類・対象移動・接触停止の回帰テストも含みます。
`gazebo_object_safety.py`の全6条件も再実行し、壁内・地図外・接近不能の拒否、
走行中の対象移動・削除・削除後の再指示の停止を確認しました。最後のTwistはいずれもゼロです。

### 安全条件と最適化の範囲

- A*の斜め移動は隣接する直交2セルも空いている場合だけ許可します。
- 直線判定は触れる全セルを調べます。角や格子の境界への接触も占有として扱います。
- 計画用の地図には、従来の車体半径・安全余裕に加えて追従用に0.10mを追加します。最終接触の壁・境界余裕は従来どおりです。
- 最少化はA*の曲がり角・サンプル・姿勢に沿った接続点から生成した有限グラフの範囲です。連続空間全体の厳密な最小回数を保証しません。
- `planned_turns`は1度を超える方向変化で、指令上のその場旋回回数とは異なります。物理的な追従誤差によって実測回数は変わります。
- 計画後に位置情報を更新し、対象の移動とロボット位置を再確認してから走行します。

検証前の箱は`x≈8.47m`で地図外にあり、ロボットも計画用境界付近にいました。
配置を`/tmp/pai-diagonal-baseline/poses.json`へ保存し、検証後のGUIは到達可能な初期配置で起動し直しています。
GUIの目視による動画確認は未実施です。

---

## 赤い箱への接触をゴールにする更新（2026-10-31 JST）

既定の`goal_mode`を`contact`にしました。A*で安全な接近地点へ移動し、その後0.10m/sで前進して、
箱のContactセンサーが`vehicle_blue`との接触を通知したら停止します。
`ARRIVED`は接近地点、`CONTACT_REACHED`は接触完了のログです。

ワールドへContact system、登録オブジェクトのコリジョンへ接触センサーを追加しました。
Gazebo `gz.msgs.Contacts`をROS `ros_gz_interfaces/msg/Contacts`へブリッジし、
`red_box::body::collision`と`vehicle_blue::chassis::collision`の組み合わせを実測しました。
接触がない時の空メッセージは配信されないため、走行前にGazebo publisherとROS bridgeの存在を確認します。

### 検証結果

単体テスト：**52件成功**。対象以外の障害物・境界、車両の向きと後端の張り出し、
回転箱の中心線との交点、接近候補の格子境界、接触相手の識別、古い接触メッセージの除外、センサー不在、
位置・向き・距離条件付きの接近再開、移動・タイムアウト・距離超過時に成功と報告せず停止することを確認しました。

```sh
docker compose exec sim python3 -m pytest -q /workspace/tests
# 52 passed
```

実シミュレーションはHEADLESSで行いました。
ユーザーの配置を保存し、ロボット`(-1.1746361,4.3315657,yaw=1.3381)`、
箱`(-1.1242635,7.0689702,z=0.4)`を再現して、実CLIによる接触を検証しました。

| 試験 | 接触検知時のロボット位置 | 独立購読による接触確認 | 最終Twist | 停止後1秒の位置変化 |
|---|---|---|---|---|
| 箱 yaw=0 | `(-1.1345969,6.4630442)` | 箱とシャーシの接触あり | `(0,0)` | 0.000000m |
| 接触後に同じ指示を再実行 | `(-1.1330510,6.4662188)` | 箱とシャーシの接触あり | `(0,0)` | 0.000000m |
| 箱 yaw=45度 | `(-1.1444539,6.3451756)` | 箱とシャーシの接触あり | `(0,0)` | 0.000000m |

2回目の指示は、近距離で向きが合い、壁・境界との余裕があることを確認して低速接近から再開しました。
45度の配置では、通常の接近候補が占有格子の境界に入ったため、近傍の安全な接近候補へ切り替えました。
手前停止の`--goal-mode stand_off`も、同じ配置で`ARRIVED`まで正常終了することを確認しました。

再実行用の統合テスト：

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/tests/gazebo_contact_goal.py'
```

このテストは接触モードを明示し、実CLI・接触トピック・速度トピック・ワールドposeを使って確認します。
ロボットは箱まで到達可能な位置から開始してください。成功すると`PASS: contacts=..., final Twist=(0.0, 0.0), stationary_drift=...`を表示します。

### 接触区間の条件と制限

- 対象の箱への接触を許可し、壁・他の箱・境界には安全余裕0.25mを適用します。
- 接触区間の車両外形は向き付き矩形です。ローカルの範囲`x=[-1.72,0.31]`、`y=[-0.94,0.94]`が、生成時の全コリジョンを含むことを検算します。
- 接触の成功判定にはセンサーメッセージが必要です。進捗なし3秒、全体60秒、接近距離上限、位置喪失・対象移動・方向逸脱は中断して停止します。
- 箱は静的モデルです。長い迂回経路から接触までの一連の動画・GUIでの目視試験は未実施です。今回の接触試験は既存の長距離A*検証と区別しています。

接触センサー設定は[Gazebo公式のContact sensor例](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/contact_sensor.sdf)とインストール済みのROS型を確認しています。
以下の速度比較・移動対応の記録は、接近地点までの手前停止モードの結果です。

---

## 走行速度の調整（2026-10-31 JST）

直進上限を0.35m/sから0.7m/sへ変更しました。旋回上限0.5rad/s、接近時の距離ゲイン0.8、旋回ゲイン1.8は従来値を維持しています。
`config/maze.json`の`navigation`を追従制御とSDF生成で共有し、Gazebo側の速度制限も一致させました。

HEADLESSで迷路を毎回初期化し、ロボット`(-5,-4)`から箱`(5,4)`の接近ゴール`(2.25,4)`まで、同じ約15.82mの経路で比較しました。

| 設定 | 直進上限 | 旋回上限 | コマンド実行時間 | 停止後の到着誤差 | 最小車体包絡円離隔 |
|---|---|---|---|---|---|
| 従来 | 0.35m/s | 0.5rad/s | 101.426秒 | 0.055m | 0.432m |
| 更新後 | 0.7m/s | 0.5rad/s | 83.651秒 | 0.053m | 0.437m |

コマンド全体の実時間をBashの`time`で計測しました。短縮は約17.5%です。
更新後の追従制御開始から到着までのログは`elapsed=81.6s`でした。
各設定1回の比較であり、GUIの描画負荷・経路・実行環境によって時間は変わります。

旋回0.8rad/s・接近ゲイン1.0の試行では、曲がり角で`Tracking deviation: next segment unsafe`により安全停止しました。
このため最終設定は直進上限のみを引き上げています。地図の膨張、安全判定、到達許容誤差は変更していません。

```sh
docker compose exec sim python3 -m pytest -q /workspace/tests
# 35 passed in 0.61s
```

速度設定の共通化、独自上限の追従、接近時の減速、不正値・設定名の誤りの拒否、ROS向けの数値型を追加で検証しています。
今回の更新にDockerの再ビルド・再起動は不要ですが、SDF側の上限を反映するためGazebo迷路の再起動が必要です。

---

## 箱の移動対応：追加検証（2026-10-31 JST）

初回実装の固定位置試験に加え、以下を実施しました。この追加検証時点の単体テストは **28件成功** です。

```sh
docker compose exec sim python3 -m pytest -q /workspace/tests
# 28 passed in 0.63s
```

移動前後で占有位置が置き換わること、設定JSONが変更されないこと、回転した矩形の包含、
地図外・重なり・非接地・傾き・非有限値の拒否、最新観測なしの拒否、
移動・回転・位置喪失の監視、停止中に対象が動いた場合に到着と報告しないことを追加で検証しました。

### Gazeboの静的な箱の位置取得

`red_box`にもPosePublisherとROS bridgeを追加しました。
`/model/red_box/pose`はROS `tf2_msgs/msg/TFMessage`、Gazebo `gz.msgs.Pose_V`で、
`frame_id=maze`、`child_frame_id=red_box`を実測しました。

静的モデルをワールドの`set_pose`サービスで`(5,4,0.4)`から`(-5,3,0.4)`へ動かし、
配信座標も`(-5,3,0.4)`に変わることを確認しました。
移動操作には以下を使用しました。

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && gz service -s /world/maze/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 --req "name: \"red_box\", position: {x: -5, y: 3, z: 0.4}, orientation: {w: 1}"'
```

GUIのマウス操作自体は未検証です。物理シミュレーション試験はHEADLESSで行いました。

### 同じ日本語コマンドによる移動後の到着

使用コマンドは変更していません。

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/app/robot_control.py "赤いオブジェクトまで行ってください"'
```

| 配置 | 接近ゴール | 停止後のロボット位置 | 最終誤差 | 最小車体包絡円離隔 |
|---|---|---|---|---|
| `(-5,3)`, yaw=0 | `(-5,0.25)` | `(-5.0506205,0.2256138)` | 約0.056m | 0.625m |
| `(-4.8,-3)`, yaw=45度 | `(-4.8,-0.1257359)` | `(-4.7682933,-0.1694214)` | 約0.054m | 0.540m |
| 初期配置`(5,4)`の回帰試験 | `(2.25,4)` | `(2.2264828,4.0489751)` | 約0.054m | 0.432m |

1回目は初期ロボット位置から約4.32m、2回目はその到着位置から約0.78mの計画経路でした。
2回目の箱の占有矩形は、元の`0.6 × 0.6m`から回転を包む`約0.8485 × 0.8485m`になりました。
いずれも`Live object`ログで実行時位置を確認し、`ARRIVED`で終了しました。
初期配置の回帰試験ではロボットも初期位置にリセットし、約15.82mの経路を走行しました。

### 実シミュレーションでの失敗時停止

専用の統合テストを追加しました。速度トピックを別ノードで購読し、実CLIの出力と停止指令を検証します。
**このテストは箱を移動・削除するため、終了後に迷路を再起動してください。**

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/tests/gazebo_object_safety.py'
```

| 条件 | 結果 |
|---|---|
| 箱を壁内`(0,-3)`へ移動 | `overlaps`で拒否、速度指令はすべてゼロ |
| 箱を地図外`(9,0)`へ移動 | `object outside field`で拒否、速度指令はすべてゼロ |
| 箱を接近不能な`(1,-3)`へ移動 | `No reachable approach point`で拒否、速度指令はすべてゼロ |
| 走行指令を確認してから箱を`(5,4)`→`(5,5)`へ移動 | `object moved during navigation`で中断、最後のTwistはゼロ |
| 走行指令を確認してから箱を削除 | `red_box: World pose missing or stale`で中断、最後のTwistはゼロ |
| 箱の削除後に再指示 | `No fresh object world pose`で拒否、初期座標へ走行せず、速度指令はすべてゼロ |

全6条件がPASSで終了しました。
launcherの終了後にGazeboとbridgeの子プロセスも残らないことを`pgrep`で確認しました。

対象の動的追従や自動再計画は行いません。箱の移動を検知したら停止し、再指示で新しい地図・経路を作ります。
壁の移動、未知の物体、GUIでの動画収録は今回の検証範囲に含みません。

---

以下は、初回実装時の固定位置を使った検証記録です。

## 環境

Docker DesktopのARM64コンテナ、Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic。
`docker compose up -d --build`で構築し、`HEADLESS=1`でGazebo物理シミュレーションを起動しました。
GUIの目視確認・動画収録・実機試験は実施していません。

## 単体テスト

```sh
docker compose exec sim python3 -m pytest -q /workspace/tests
```

結果：`11 passed in 0.49s`。

既存分類と目的地分類、停止語優先、未知・曖昧な対象、4近傍A*の迂回・行き止まり・到達不能、
膨張後の狭路拒否、無効座標、設定とSDFの整合、位置の鮮度、理想運動モデルの追従、
例外・割り込み時の停止処理を確認しました。
理想運動モデルのテストはGazebo走行検証と区別しています。

## ROS / Gazeboの位置情報

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic list -t'
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && gz topic -i -t /model/vehicle_blue/pose'
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic echo /model/vehicle_blue/pose --once'
```

実測したトピック：

```text
/model/vehicle_blue/cmd_vel  geometry_msgs/msg/Twist
/model/vehicle_blue/odometry nav_msgs/msg/Odometry
/model/vehicle_blue/pose     tf2_msgs/msg/TFMessage
Gazebo pose message type: gz.msgs.Pose_V
frame_id: maze
child_frame_id: vehicle_blue
初期位置: x=-5.000000355, y=-4.000000000, yaw≈0
```

地図との座標合わせに使うのはワールドposeです。
公式車両をそのまま使った最初の試行では、旋回時に原点が動いて近距離の経由点で停滞し、タイムアウト停止しました。
生成車両の全リンクを平行移動して、原点を駆動輪中央に合わせました。
その基準点から公式車両の全コリジョンを包む半径は約1.783mで、設定半径は1.85mです。
生成処理でも包絡半径を検算します。
曲がり角の経由点を早く切り替えた試行は安全な線分の確認で停止したため、中間点の許容誤差を0.025mにし、通路も広げています。
以下は修正後の設定・制御での結果です。

## 日本語指示からの走行

```sh
docker compose exec -e HEADLESS=1 sim bash /workspace/scripts/run_maze.sh
# 別ターミナル
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/app/robot_control.py "赤いオブジェクトまで行ってください"'
```

```text
AI inference: '赤いオブジェクトまで行ってください' -> GO_TO_OBJECT
start ≈ (-5,-4)
接近ゴール = (2.25,4)
waypoints ≈ [(-5,-4), (-5.05,-3.95), (-5.05,4.05),
             (-4.35,4.05), (-4.35,4.25), (0.35,4.25),
             (0.35,4.05), (2.25,4.05), (2.25,4)]
ARRIVED: pose=(2.226814339,4.049011661,0.040594195)
minimum_footprint_clearance=0.431m
```

計画経路長：約15.82m（上の実測開始位置と経路から算出）。接近ゴールへの最終誤差：約0.054m。
制御周期ごとにワールドposeから計算した車体包絡円と壁・箱・境界の最小離隔下限は0.431mでした。
これは接触センサーの計測ではありませんが、包絡円が障害物から離れていることを確認しています。
終了後のワールドposeを1秒間隔で比較した平面位置変化は0mでした。

## 座標指定と基本操作の回帰

現在位置が初期位置と異なる状態から実行しました。

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/app/robot_control.py --goal 1.5 4 "前に進んで" "左に向いて" "ストップして"'
```

```text
FORWARD / LEFT / STOP と分類
start=(2.268321144,4.078839776), goal=(1.5,4), length=0.80m
ARRIVED: pose=(1.571720998,4.058964685,-2.883947040)
minimum_footprint_clearance=0.568m
Executing FORWARD
Executing LEFT
Executing STOP
```

座標到着誤差：約0.093m。続く基本動作も正常終了しました。
基本動作後のワールドposeは`x=0.704979600, y=3.812394700, yaw≈-2.00rad`で、位置・姿勢の変化を確認しました。
この座標走行は迷路内の自由空間で実行しており、壁なし専用ワールドの試験ではありません。

## 失敗時の停止

ROSの速度トピックを別ノードで購読し、以下を確認しました。

| 条件 | 実測結果 |
|---|---|
| 座標走行中のプロセスにSIGINT | 終了時のTwistはlinear.x=0、angular.z=0。ROS context終了によるpublishエラーなし |
| `紫の箱まで` | GO_TO_OBJECTと分類、対象不明で失敗。受信した5つのTwistはすべてゼロ |
| 座標走行中にGazeboをpause | 位置更新が途絶え、`World pose missing or stale`で失敗。最後のTwistはゼロ |

pause試験には`gz service -s /world/maze/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 2000 --req 'pause: true'`を使用し、試験後は`pause: false`に戻しました。
Ctrl+C時もfinallyで停止指令を送れるよう、rclpyの自動シグナル処理を無効にしてPythonの割り込み処理を使っています。

## 未検証・制限

- GUI表示での走行映像、動画・PDF作成、アップロードは未実施です。
- 単純なL字だけの別ワールドでの走行、壁なし専用ワールドの座標走行は個別には行っていません。
- 未知・動的な障害物には対応しません。
- SIGKILL、OS停止、通信切断に対する独立watchdogはありません。
- 設定や物理条件を変えた場合は、再度走行と離隔を確認してください。
