`maze.sdf` は `scripts/run_maze.sh` 起動時に、インストール済みの公式 `diff_drive.sdf` と `config/maze.json` から生成します。生成物を手で編集せず、配置はJSONで変更してください。

赤い箱はGazebo起動後に移動できます。自律走行では設定上の初期座標ではなく、PosePublisherで取得した現在位置を使います。壁配置は固定です。

登録オブジェクトには接触センサーも生成します。既定の接触モードでは低速で触れて停止し、`CONTACT_REACHED`を表示します。
