# 図面A・BのデモをMP4で録画

ローカルモデルとDocker Desktopを起動し、GUIイメージをビルドする。FFmpegはGUIイメージに含める。

```bash
docker compose --profile gui build gui
python3 scripts/record_demo.py
```

録画中はダッシュボードや別のターミナルからデモを開始しない。スクリプトは専用GUIコンテナを再起動し、各図面をローカルモデルで読み直して実際にロボットを動かす。

左側に入力図面と実際のAI出力、右側にGazebo画面を配置する。進捗と成功表示は実際の実行ログ・結果から更新する。AI読解の待ち時間は収録せず、検証済みの指示と動作を収録する。図面AとBは新しい基板・部品でそれぞれ実行する。

生成物：

- `recordings/drawing_a.mp4`：図面A、60秒。
- `recordings/drawing_b.mp4`：図面B、60秒。
- `recordings/pcb_assembly_demo.mp4`：A→Bを連結した約2分、1920×900、等速、音声なし。
- `outputs/record_task_*.json`、`.audit.json`：実際の読解結果と監査記録。
- `outputs/record_result_*.json`：解放・退避後の挿入結果。

挿入に失敗した場合や録画が挿入完了より先に終わった場合はエラーで停止する。再録画時は同名ファイルを上書きするため、残したい動画は別名で保存する。動画はGitの管理対象から除外している。

YouTubeへ提出するときは、タイトルを課題指定の`PAI最終課題_omnicampusアカウント名`にする。必要なら日本語の説明・字幕を追加し、限定公開または公開でアップロードする。倍速に編集した場合は明記する。録画後は動画を再生し、動作と図面が見えること、尺と表示内容、個人情報がないことを確認する。
