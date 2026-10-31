# 検証記録（2026-10-31 JST）

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
