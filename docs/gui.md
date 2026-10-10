# ブラウザでGazeboを見る

Mac上のDockerで仮想ディスプレイを起動し、noVNC経由でGazebo GUIを表示する。ブラウザで見えるのは実際のGazebo画面で、物理シミュレーションとROS 2制御はコンテナ内で実行する。

## 起動

Docker Desktopを起動し、リポジトリのルートで実行する。

```bash
docker compose --profile gui up -d --build gui
```

ブラウザで次を開く。

[Gazeboデスクトップ](http://localhost:6080/vnc.html?autoconnect=true&resize=scale)

最初はターミナルだけが表示される。次のコマンドでデモを開始する。

```bash
docker compose --profile gui exec gui /ros_entrypoint.sh python3 scripts/run_insertion.py --gui --start-delay 10 --keep-open
```

GUI起動後、10秒待ってロボットが動く。完了後も画面を開いたままにし、結果を観察できる。実行したターミナルでCtrl+Cを押すとGazeboが終了する。同じコマンドを再実行すれば初期状態から再生できる。

既にバックグラウンドのデモが動いている場合は、先にGUIコンテナを再起動する。ブラウザは再読み込みしてから、上記の実行コマンドを使う。同じGUIコンテナでデモを重複起動しない。

```bash
docker compose --profile gui restart gui
```

## 視点操作

- マウスホイール：拡大・縮小。
- 左ドラッグ：画面を移動。
- Shift＋左ドラッグ：視点を回転。

初期視点と背景色は`sim/gui.config`に設定する。ソフトウェア描画を利用するため、表示速度は環境によって変わる。

## 停止

```bash
docker compose --profile gui stop gui
```

GUIのHTTPポートはホストの127.0.0.1だけに公開し、VNCポートは外部公開しない。

## ログ

- デスクトップ：`outputs/desktop.log`。
- Gazebo・ROSブリッジ：`outputs/simulation.log`。
- 挿入結果：`outputs/insertion_result.json`。

画像からの読解も含めて実行する場合は、Mac上で[ローカルAI](local-ai.md)を起動し、空いているGUIコンテナに対して次を実行する。

```bash
python3 scripts/run_from_drawing.py drawings/assembly_b.png --gui --keep-open
```

図面を差し替えると挿入箇所が変わる。図面とAI出力をGazebo内へ重ねる表示や動画録画は未整備。

## 検証記録（2026-10-10）

Apple Silicon Mac上でGUIイメージをビルドし、noVNC接続とGazeboの3Dモデル表示を確認した。GUI表示中もピック・挿入・解放・退避を完了し、結果JSONのsuccess=trueを確認。ピン先端の挿入深さは約40 mmだった。
