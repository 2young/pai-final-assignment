# PAI 最終課題：日本語指示をAIが分類し、ROS 2＋Gazeboのロボットを操作

対象：Apple Silicon Mac / Docker Desktop / Ubuntu 24.04 ARM64 / ROS 2 Jazzy / Gazebo Harmonic。
**注意：このスターターはファイル生成・Python学習処理の静的検証用です。Mac上のDocker/Gazeboの実機動作は未検証です。**

## 0. Docker Desktopを起動

- Docker Desktop の設定でメモリをできれば 8 GB 以上割り当てる。
- ローカルのポート6080を使用する。外部公開しないこと。

## 1. ビルドとコンテナ起動（Macターミナル）

```sh
cd pai_jazzy_gazebo_starter
docker compose build
docker compose up -d
docker compose exec sim uname -m
```

最後が `aarch64` ならARM64で動作している。

## 2. ROS/Gazeboの存在確認

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 --help >/dev/null; gz sim --versions; ros2 pkg prefix ros_gz_sim_demos'
```

`gz sim --versions` はHarmonicの8.xを期待。

ブラウザで http://localhost:6080/vnc.html を開き、Connectをクリック。
画面が出ない場合は `docker compose logs sim` で確認。

## 3. ロボットを手動で動かす：Gazeboを起動（ターミナルA）

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 launch ros_gz_sim_demos diff_drive.launch.py rviz:=false'
```

別のMacターミナルBで以下を実行（約3秒後 Ctrl+C）。

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 topic pub --rate 10 /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.2}, angular: {z: 0.0}}"'
```

Gazeboの「青い車」が動くことを確認。停止するにはターミナルBで Ctrl+C のあと、以下の停止コマンドを送信：

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 topic pub --once /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"'
```

## 4. AIモデルの訓練（ターミナルB）

```sh
docker compose exec sim python3 /workspace/app/train.py
```

日本語の短文と`FORWARD/BACKWARD/LEFT/RIGHT/STOP`のラベルから、文字n-gram TF-IDF＋線形SVMの分類器を学習します。

## 5. Gazeboを使わずAIの推論を動作確認

```sh
docker compose exec sim python3 /workspace/app/robot_control.py --dry-run --demo
```

## 6. AIの推論でGazeboのロボットを操作

ターミナルAでGazeboの起動を続けたまま、ターミナルBで以下を実行：

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/app/robot_control.py --demo'
```

自然言語入力の変更例：

```sh
docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/app/robot_control.py "前に進んで" "左に向いて" "ストップして"'
```

`/model/vehicle_blue/cmd_vel` に指令を出します。Gazebo内で車が動くことを確認します。

## 7. 撮影・提出

- Mac画面録画でGazeboの車両とAI分類ログを含む1〜3分の動画を作成。
- レポートに課題の目的、モデルの学習データ、構成図、推論例、試行結果、独自の工夫、限界を記載。
- GitHub public にソースコード一式をアップロード。実行環境のREADMEも添付。
- PDFとGitHub URLとYouTube動画URLをOmniCampusへ提出。
- 公式サンプルの丸写しではなく、分類データ・AI判定・ROS制御コードが独自部分。
- 個人の氏名・所属・メール等をコード、README、動画に記載しない。

## トラブルシューティング

1. `docker compose build`でROS/Gazeboパッケージに失敗：arm64のaptパッケージ供給状況を確認。ソースからの大規模ビルドはせず代替環境を検討。
2. noVNCが真っ黒：`docker compose logs sim` でXvfb/x11vncを確認。`docker compose exec sim bash -lc 'glxinfo -B'`でGLXを診断。
3. Gazebo GUIがクラッシュ：`LIBGL_ALWAYS_SOFTWARE=1` は設定済。必要ならGazeboのレンダリングエンジンを`ogre`に切替。サーバとGUIを分離するのは次の対処。
4. `ros2 topic list`に`/model/vehicle_blue/cmd_vel`がない：サンプルlaunchが動いているか、Bridgeのログを確認。
5. AIが間違える：`app/intent_model.py`の`EXAMPLES`に自分の言い回しを追加して再学習。
6. Macのデフォルト設定ではGPUアクセラレーションなし。Gazeboが重い場合はDocker Desktopメモリ8 GB以上、rvizなし、シンプルなワールドを使う。

参考：
- https://gazebosim.org/docs/harmonic/ros_installation/
- https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim_demos/README.md
- https://gazebosim.org/docs/harmonic/troubleshooting/

## 終了

```sh
docker compose down
```
