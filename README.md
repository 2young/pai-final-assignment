# 日本語指示からA*経路探索で迷路内を自律移動

Apple Silicon / Docker Desktop / Ubuntu 24.04 ARM64 / ROS 2 Jazzy / Gazebo Harmonic。
日本語をローカルの文字n-gram TF-IDF＋LinearSVCで分類し、青い差動二輪車両が登録された赤い箱まで移動し、最後は低速で接触して停止します。
AIは意図分類、A*は経路計算、Pure Pursuitと速度制御は走行を担当します。色・名前・サイズは登録データを使い、物体の現在位置はGazeboから取得します。画像認識は行いません。

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

今回の障害物追跡の更新後、初回はターミナルAでCtrl+C終了し、上の起動コマンドで迷路を再起動してください。障害物にも位置配信とbridgeを追加します。Dockerの再ビルド・分類器の再学習は不要です。その後は箱や障害物を移動するたびに再起動する必要はありません。

1. Gazeboの赤い箱（モデル名`red_box`）を選択します。
2. 移動ツールまたはPose編集で、X/Y座標を変更します。例として`x=-5, y=3`へ移動できます。箱の中心のZ座標は`0.4m`を維持してください。
3. 水平面内の回転（yaw）は可能です。箱は立てたまま配置し、壁・車両への重なりを避けます。運用範囲はX/Yとも`-12〜12m`です。箱が境界をまたぐ場合も、範囲内から安全に接触できる面があれば到達できます。登録済みの9個の障害物も移動・水平回転できます。
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

指示のたびに赤い箱と全9個の障害物の位置を読み直し、地図を作り直します。移動前の箱・障害物の占有は残しません。
走行中に箱または障害物が計画時から5cm超移動、または姿勢が3度超変化した場合は停止します。
追いかけ続ける機能はなく、箱を置き直して同じコマンドを再実行します。
箱の位置情報が取得できない場合は停止し、初期座標では走行しません。

障害物を移動する場合もGazeboの移動ツール・Pose編集を使い、移動を終えて再生状態にしてから同じ指示コマンドを実行します。
高さは各障害物の中心Z=0.5mを維持し、傾けずに配置してください。壁同士の接続・重なりは許容しますが、車両の通路・旋回余裕と箱への接触面を残してください。
`Live obstacle: detour, center=[...], yaw=...`などのログで取得した配置を確認できます。
JSONの位置は初期配置に使い、実行時は取得した座標・向きを使います。寸法は引き続き登録値です。
1つでも位置を取得できなければモデル名を出して停止し、JSON座標へ自動で戻しません。削除・未登録モデルの追加・寸法変更への自動対応は行いません。
走行中の移動後は配置を確定して再指示してください。動く障害物を追跡しての自動再計画は行いません。

## 運用範囲と段階的な探索

運用範囲は固定の24m四方です。以前の16m四方から広げたため、`(8.4,4)`などの箱にも移動できます。
壁の位置は同じですが、壁の端を回り込める経路が増えます。境界は物理的な壁ではなく、車両の計画・外形検査で守る範囲です。

毎回24m四方を探索する代わりに、現在位置と目的地を囲む範囲から探索します。経路が見つからなければ範囲を広げ、既定では最後に運用範囲全体を調べます。
`Planning window`ログに探索範囲と格子数、`operating_bounds`に固定の運用範囲を出します。探索用の一時的な端を物理的な壁として膨張しません。
経路が見つかった時点で拡張を終えるため、運用範囲全体での最短経路・最少旋回を保証するものではありません。

`config/maze.json`の`planning`で上限を変更できます。

| 設定 | 既定値 | 内容 |
|---|---:|---|
| `initial_padding` | 3.0 | 最初の探索範囲に足す余白（m） |
| `expansion_step` | 3.0 | 次の探索で各方向に足す余白（m） |
| `max_attempts` | 4 | 探索範囲の試行上限。2以上なら最終試行は運用範囲全体 |
| `max_cells` | 60000 | 1つの格子地図のセル数上限（全範囲は240×240） |
| `max_expansions` | 200000 | 全試行・接近候補を合計した探索状態の展開上限 |
| `timeout` | 8.0 | 地図生成を含む計画全体の実時間上限（秒） |

箱の判定は`boundary_policy: reachable_surface`です。箱の一部が運用範囲と交差すれば接近計画を試み、車両全体と安全余裕が範囲内に収まる接触経路を探します。
箱の実座標・寸法を境界へ切り詰めることはありません。一部が範囲内でも、接触可能とは限りません。
従来の箱全体を範囲内に要求する判定は`whole_object`で選択できます。

境界近くで接触した後、別の目的地へ進むには、その場旋回の余裕が不足することがあります。
その場合は姿勢を保ち、最大0.20m/sで前進または後退して内側へ戻ります。車両外形・全障害物を確認し、その先の経路まで見つかった場合だけ実行します。
境界に沿う向きや後退先の障害物などで安全に戻れなければ停止し、ロボットの位置・向きの調整を求めます。

| 中断ログ | 状況・対処 |
|---|---|
| `object outside operating area` | 箱が範囲と交差していません。表示された中心座標・範囲を確認して配置し直します |
| `No reachable approach/route within operating area` | 全範囲を調べても安全な経路・接触面がありません。箱やロボットの配置を調整します |
| `Planning ... limit reached` | 時間・セル数・探索状態数・試行数の上限です。到達不能とは区別し、該当設定と配置を確認します |
| `No safe straight entry from boundary` | 境界付近から安全に内側へ戻れません。車両の位置・向きを調整します |

いずれもゼロ速度を送って終了します。

## 接触して停止するゴール

既定の`config/maze.json`は`"goal_mode": "contact"`です。通常の日本語コマンドで次の順に実行します。

1. A*で箱の手前の安全な接近地点へ移動し、停止します。
2. 箱の中心へ向きを合わせ、離れている間は最大0.40m/sで前進し、接触直前に0.10m/sへ減速します。
3. 箱の接触センサーが`vehicle_blue`との接触を検知したら、ゼロ速度を送って終了します。

箱は静的モデルのままです。最後の接近区間では対象の箱への接触を許可し、壁・他の箱・地図境界は安全余裕0.25mを含む車両の矩形外形で確認します。
車両外形はワールド生成時に実際のコリジョン形状を含むか検算します。
箱の位置喪失・移動、接近方向のずれ、進捗なし3秒、全体60秒、接近距離上限に達した場合は中断して停止します。
接触区間ではロボット・登録オブジェクトの位置更新が実時間で0.30秒途絶える場合も停止します。
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
| `config/maze.json` | 障害物・箱の登録情報と初期配置・車体寸法 |
| `app/generate_world.py` | インストール済み公式車両を残し、設定から迷路SDFを生成 |
| `app/intent_model.py` | ローカルAI分類、停止語の優先処理 |
| `app/target_resolver.py` | 色・名前の照合、到達可能な接近地点を選択 |
| `app/grid_map.py` | 膨張済み占有格子と安全な線分の確認 |
| `app/adaptive_planner.py` | 固定の運用範囲内で探索範囲を段階的に拡張 |
| `app/planning_limits.py` | 計画時間・格子数・探索状態数の上限 |
| `app/field_geometry.py` | 回転した箱と運用範囲の実際の交差判定 |
| `app/boundary_entry.py` | 境界付近から姿勢を保って内側へ戻る計画・走行 |
| `app/path_planner.py` | 8近傍A*、安全な任意角度の直線接続、時間・旋回を評価した経路選択 |
| `app/path_geometry.py` | 曲線全体の安全確認、角の曲線化、経路の距離と進捗 |
| `app/pure_pursuit.py` | 先読み追従、状況に応じた速度制御、制動軌跡の検査 |
| `app/follower_config.py` | 追従・先読み・加減速・鮮度の設定検査 |
| `app/pose_provider.py` | 車両・登録オブジェクトのワールド位置、姿勢、鮮度確認 |
| `app/object_tracker.py` | 全障害物・箱の位置取得、配置検査、走行中の移動検知 |
| `app/ros_feedback.py` | 制御周期内で複数の受信処理を行う、時間・回数を制限した処理 |
| `app/motion_config.py` | 車両と追従制御で共通の速度設定・値の検査 |
| `app/contact_speed.py` | 外形間の距離下限、制動距離、シミュレーション時刻に合わせた加減速 |
| `app/contact_goal.py` | 接触区間の計画、車両の矩形外形、接触検知と低速停止 |
| `app/route_metrics.py` | 旋回・回転角・シミュレーション時刻による実速度の計測 |
| `app/navigator.py` | Pure Pursuit／従来追従、到達・停滞・停止処理 |
| `app/robot_control.py` | 自然言語と座標指定のCLI、従来操作との統合 |
| `scripts/run_maze.sh` | SDF生成、GazeboとROS bridgeの起動・終了 |

運用範囲は24m四方、`[-12,12] × [-12,12]`、セル幅0.1m。境界は計画上の境界です。
探索格子の左下を`(x_min,y_min)`とし、ワールド座標`(x,y)`から`(floor((x-x_min)/0.1), floor((y-y_min)/0.1))`へ変換します。
セル中心は`(x_min+(i+0.5)*0.1, y_min+(j+0.5)*0.1)`です。探索範囲の拡張時もワールド座標は変わりません。
車両初期位置は`(-5,-4, yaw=0)`。赤い箱の初期中心は`(5,4)`です。自律走行ではGazebo上の現在位置を使います。
固定障害物は合計9個で、中央の壁に加え、西側・北側の壁と四隅のブロックを配置しています。[追加した障害物の座標と反映方法](worlds/README.md)を参照してください。

公式車両は車体長が約2mあり、駆動輪中央と元のモデル原点が約0.55mずれています。
生成時に全リンクを平行移動し、迷路用車両の原点を駆動輪中央に合わせます。
保守的な包絡円半径1.85mに安全余裕0.25mを足して、壁・箱・境界を膨張します。
セル中心だけでなくセル全体を保守的に占有判定します。経路計画時は追従誤差に備えて、さらに0.10mの余裕を加えます。
A*は箱中心ではなく、膨張領域から離れた接近候補を使います。上下左右・斜め・現在位置や向きに沿った候補を調べます。接触モードでは0.35mを基本に0.25m/0.45m/0.15mの近傍候補も調べ、最後の接触区間が安全な地点を選びます。
経路追従の最大前進速度は1.0m/s、角速度は0.8rad/sです。曲がり角・障害物の近く・ゴール手前では減速します。

## 斜め走行と旋回回数の削減

差動二輪車両が斜め方向を向いて前進します。指示コマンドは従来どおりです。

1. 8近傍A*で経路を探索します。直交移動は1、斜め移動は√2のコストとし、対応する距離の推定を使います。斜めの両側のセルが空いている場合だけ移動できます。
2. 線分が触れる格子を端・角を含めて確認し、見通しのある経由点同士を任意の角度で結びます。
3. A*の曲がり角・途中のサンプル・開始点と終点・姿勢に合わせた接続点から候補グラフを作り、到着方向も保持して探索します。
4. 既定では`経路長 / 最大直進速度 + 0.4 × 総回転角 / 最大角速度`の評価値で経路と接近地点を選びます。旋回が少なくても大きく迂回する経路を避けます。現在の車体の向きと、接触モードの最後に箱を向く旋回も評価します。

最少化の対象は生成した有限の候補グラフです。上記の評価値は所要時間の近似で、連続空間全体の最短距離・最短時間を保証しません。
`path_following.path_objective`を`turns`にすると、従来の**方向転換回数 → 経路長 → 総回転角**の順に戻せます。

経路追従はPure Pursuitを使います。経路上の前方点を選び、車体座標系の横方向変位から曲率を計算して前進と旋回を同時に行います。
中間点ごとの停止をなくし、安全に曲がれる角は二次Bezier曲線で接続します。曲線全体を含む三角形が膨張済み占有格子に触れない場合だけ採用し、各線分も再検査します。
曲線を置けない角や大きな向きのずれでは、減速して停止旋回します。最終接近地点では停止し、従来の接触処理へ切り替えます。
曲率による減速と衝突予測の考え方は[Nav2のRegulated Pure Pursuit解説](https://api.nav2.org/nav2-jazzy/html/md_nav2_regulated_pure_pursuit_controller_README.html)も参照しています。本実装は既存Python構成に追加した独自の追従処理で、Nav2の導入は不要です。

ログには次を出力します。

- `planned_turns`：1度を超える方向変化の計画回数。開始姿勢と接触方向を含みます。
- `rotation_episodes`：線速度ゼロ・角速度の絶対値が0.02rad/sを超える指令の連続区間数です。
- `total_yaw`：ワールド姿勢から積算した実際の総回転角です。
- `NAVIGATION_SUMMARY`：接近経路と最終接触を合わせた計測値です。
- `PLAN_STATS`：選択した接近地点への生A*の格子数・格子経路長と、簡略化後の経路長・点数です。
- `FOLLOW_PATH`：角の曲線化数と、停止旋回を残した角の数です。
- Pure Pursuitの`ARRIVED`：`sim_elapsed`、最大実速度、経路からの最大横ずれも出力します。

計画上の方向転換回数とその場旋回の指令回数は異なる指標で、追従誤差によっても変わります。
斜め経路を追加した時点（最終接近0.10m/s固定）のHEADLESS比較では、その場旋回9回→4回、接触完了まで約106秒→約70秒となりました。
詳細と失敗時停止の再検証結果は[検証記録](VALIDATION.md)を参照してください。

## 走行速度の設定

`config/maze.json`の`navigation`で速度を変更できます。追従制御とGazebo車両の上限の両方に反映します。

```json
"navigation": {
  "max_linear_speed": 1.0,
  "max_angular_speed": 0.8,
  "distance_gain": 0.8,
  "heading_gain": 1.8
}
```

単位は直進がm/s、旋回がrad/sです。直前の版の上限は0.7m/s・0.5rad/sでした。
速度上限だけでなく、経路追従・ゴール手前の減速と組み合わせて到達時間を短縮します。
`distance_gain`・`heading_gain`は従来の経由点追従用です。Pure Pursuitは残距離・制動距離で減速します。接触直前の0.10m/sは維持します。

`path_following`で追従方式と先読み距離を設定します。未指定の値には検査済みの既定値を使います。

```json
"path_following": {
  "controller": "pure_pursuit",
  "path_objective": "travel_time",
  "tracking_margin": 0.1,
  "lookahead_min": 0.35,
  "lookahead_max": 1.0
}
```

先読み距離は速度に応じて0.35〜1.0mで変化し、見通しが悪い場所ではさらに短縮します。曲率・角速度上限・障害物までの外形余裕・残距離による各速度上限の最小値を採用します。先の曲がり角の曲率も評価し、制動距離と反応時間を確保して進入前から減速します。
加速0.5m/s²、減速0.7m/s²、角速度変化1.5rad/s²をシミュレーション時刻で制限します。
各指令について実速度・角速度から反応時間0.35秒と制動後までの軌跡を予測し、膨張格子に対して全区間を確認します。衝突予測時は速度を下げ、安全に進めなければ停止・旋回または中断します。
位置更新の実時間0.30秒の欠落、sim時刻の逆行・0.20秒超の更新間隔、経路から0.35mを超える横ずれも停止条件です。
非常停止・例外時は加減速ランプを待たずゼロTwistを送ります。

従来の経由点追従へ切り替える場合は、コマンドに`--controller waypoint`を追加します。
速度と経路選択も従来へ戻すには、`navigation`を0.7m/s・0.5rad/sにし、`path_objective`を`turns`へ変更して迷路を再起動します。

設定変更後は、迷路を起動したターミナルでCtrl+Cを押して終了し、`run_maze.sh`を一度だけ実行し直してください。
Dockerの再起動・再ビルドは不要です。起動中の迷路を終了する前に別の迷路を起動しないでください。
車両側の速度上限は迷路内の手動操作にも適用されます。
到着ログには走行制御開始からの`elapsed`（実時間の秒）を表示します。
変更前後のHEADLESSでの比較データと再実行方法は[検証記録](VALIDATION.md)を参照してください。最終接触区間は経路追従と別に計測しています。

## 最後の接近区間の速度

`config/maze.json`の`contact_approach`で設定します。指示コマンドの変更は不要です。

```json
"contact_approach": {
  "max_speed": 0.4,
  "touch_speed": 0.1,
  "acceleration": 0.3,
  "deceleration": 0.35,
  "safety_buffer": 0.2,
  "reaction_time": 0.2
}
```

- `max_speed`：離れている時の上限（m/s）。`navigation.max_linear_speed`以下にも制限します。
- `touch_speed`：接触直前の速度（m/s）。`max_speed`以下に設定します。
- `acceleration` / `deceleration`：速度指令の加減速上限（m/s²）。車両側の上限1.0以下です。
- `safety_buffer`：減速時に残す距離の余裕（m）。
- `reaction_time`：位置更新・制御の遅れに備える時間（シミュレーション時間の秒）。

箱と車両の外形を進行方向へ投影し、接触までの距離を安全側に見積もります。
箱の回転と車両の前面・側面を含めるため、斜めの接触でも減速位置を判断できます。
実速度と制動距離から速度上限を計算し、シミュレーション時刻に合わせて加減速します。
接触・エラー時はランプを待たずゼロ速度を送ります。
`max_speed`と`touch_speed`を両方0.1にすると、上限を従来と同じ低速にできます（発進時の加速ランプは残ります）。

この項目は次回の指示から反映され、Docker・Gazeboの再起動は不要です。
`CONTACT_REACHED`には接触区間の`elapsed`、`max_actual_speed`、`pre_contact_actual_speed`を出力します。
実速度はワールドposeのシミュレーション時刻から計測し、接触直前速度は接触時刻より前のサンプルを使います。
同じ配置で前進開始から接触まで約22.2秒→約8.7秒、接触直前の実速度はどちらも0.10m/sでした。
詳細は[検証記録](VALIDATION.md)を参照してください。

## 位置トピックの確認

専用ワールドに`PosePublisher`を追加し、`vehicle_blue`・登録障害物9個・赤い箱のワールドposeを30Hzで配信します。
`/model/vehicle_blue/pose`のGazebo `gz.msgs.Pose_V`をROS `tf2_msgs/msg/TFMessage`へブリッジします。
`/model/<モデル名>/pose`を同じ型でブリッジし、`frame_id=maze`と各モデルの`child_frame_id`を照合します。
静的モデルにも現在時刻付きのposeが配信されるため、移動後の位置と受信の鮮度を確認できます。車輪オドメトリの原点や積分誤差には依存しません。
箱や壁が回転している場合は、その外形を包むワールド軸に平行な矩形で地図を計算します。この保守的な判定により、回転した壁の近くで通路を狭く判定する場合があります。
全モデルで指示後の観測を要求し、受信時刻の差が0.15秒以内の配置を使います。完全な同時刻の観測を保証するものではなく、計画後の再確認と走行中の変化検知を併用します。

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic list -t'
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic echo /model/vehicle_blue/pose --once'
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic echo /model/red_box/pose --once'
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic echo /model/detour/pose --once'
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && gz topic -i -t /model/vehicle_blue/pose'
```

箱の`/model/red_box/contact`はGazebo `gz.msgs.Contacts`からROS `ros_gz_interfaces/msg/Contacts`へブリッジします。接触がある時だけ配信され、箱とロボットのコリジョンの組み合わせを照合します。

購読callbackは制御周期ごとに複数件処理し、追加分は最大63件・実時間10msまでに制限します。
A*追従では位置更新が1.5秒途絶えた場合に停止し、最終接近では0.30秒で停止します。初期位置待機10秒、全体240秒、進捗なし25秒で中断します。
未知色・複数対象・曖昧な対象は指示バッチを実行せず停止します。
`--dry-run`は分類と対象参照のみで、位置取得や走行成功を検証するものではありません。

## 制限と検証

障害物・箱は登録済みのモデル名と寸法を使い、走行開始前に移動・水平回転できます。走行中は配置が変わらないことを要求します。Nav2、SLAM、外部API、大規模LLMは使用しません。
SIGKILL・OS停止・通信切断のようにfinallyを実行できない停止に対する独立watchdogはありません。
壁・箱の寸法や初期配置・車体設定を変更した場合はワールドと走行プロセスを両方再起動してください。運用範囲・探索上限だけの変更は次の指示から反映されます。

単体テストには迂回、行き止まり、到達不能、車体より狭い通路、無効座標、対象不明、
位置の鮮度、理想運動モデルでの追従、SDFと配置定義の一致、既存日本語分類の回帰を含めます。
Gazeboの物理シミュレーション結果は、単体テストとは区別して[検証記録](VALIDATION.md)に記載しています。
単体テストとHEADLESSでの走行結果は検証記録を参照してください。移動後の占有・回転・無効配置・対象移動時の停止もテスト対象です。

## トラブルシューティング

- 車両が表示されない：`docker compose logs sim`とターミナルAの出力を確認。ソフトウェア描画のためDockerメモリは8GB以上を推奨します。
- `No subscribers`：ターミナルAのbridgeが起動しているか確認します。
- `No world pose`：公式デモではなく専用迷路を起動し、上のトピックを確認します。
- `No fresh object world pose`：表示されたモデルの`/model/<モデル名>/pose`を確認してください。更新後の初回は迷路を再起動します。初期座標への自動切替はしません。
- `Scene pose observations are not time-coherent`：全モデルを近い受信時刻で取得できません。Gazeboの再生状態・bridge・負荷を確認します。
- `object moved during navigation`：表示された箱・障害物の配置を確定してコマンドを再実行してください。
- `Robot footprint violates current obstacle`：車両と移動後の障害物の安全余裕が不足しています。車両や障害物を配置し直してください。
- `overlaps` / `object outside operating area` / `place object upright`：重なり・表示された運用範囲・箱の高さと傾きを確認して配置し直してください。
- `No Gazebo contact sensor publisher` / `No ROS contact bridge`：更新後の`run_maze.sh`で迷路を再起動してください。
- `No reachable approach point with safe contact corridor`：手前への経路または最後の接触区間が塞がっています。箱を配置し直すか`--goal-mode stand_off`を試してください。
- `No fresh pose after planning`：Gazeboが再生中か確認します。計画中に箱・ロボットを動かした場合は配置を確定して再指示します。
- `Contact robot pose missing or stale` / `contact object pose missing or stale`：位置更新が途絶えています。Gazeboの再生状態と負荷を確認してください。
- `Contact feedback interval too large`：シミュレーション時刻で0.20秒を超える位置更新の間隔があり、加減速の継続を中断しています。
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
