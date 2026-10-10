# ローカルモデルで組立図を読む

外部AIサービスへの送信や有料APIは使わず、Mac上のOllamaで画像対応モデルを動かす。モデルは`qwen3-vl:2b-instruct`を使用する。モデルの初回ダウンロードにはネット接続が必要だが、図面読解はローカルで実行する。

## 準備

Mac側で実行する。Docker内ではなくMac上で推論するため、Apple Siliconを利用できる。

```bash
brew install ollama
bash scripts/start_local_ai.sh
```

別のターミナルでモデルを取得する。

```bash
ollama pull qwen3-vl:2b-instruct
```

起動スクリプトはクラウド機能を無効化し、127.0.0.1:11434だけで待ち受ける。モデルの保存先はリポジトリ内の`.cache/ollama`で、GitとDockerのビルド対象から除外する。重みはGitHubへアップロードしない。

## 図面から作業指示を生成

```bash
python3 scripts/read_drawing.py drawings/assembly_a.png --output outputs/ai_task_a.json
python3 scripts/read_drawing.py drawings/assembly_b.png --output outputs/ai_task_b.json
```

モデルに渡すものは画像と共通の読解プロンプト・出力形式だけ。ファイル名、正解JSON、基板のソケット座標一覧は渡さない。寸法注記から部品の中心座標、向き、挿入箇所を読む。

モデル出力に、既知の基板原点（治具の校正値）をプログラムで追加する。その後、`configs/board.json`と照合する。部品・座標・向き・ソケット番号が一致しない場合は拒否し、座標を正解へ補正する処理は行わない。

対応範囲は、部品1種類・1個・向き0度の簡略組立図。任意の工場図面を読めることは示していない。

## 読解を評価

```bash
python3 scripts/evaluate_drawings.py
```

図面A・Bを読み、推論が終わってから正解JSONと比較する。結果は`outputs/drawing_evaluation.json`。モデル名、画像のSHA-256、推論時間、生の返答は各`outputs/ai_task_*.audit.json`に記録する。

## 画像入力から挿入まで一括実行

Dockerのビルドとローカルモデルの準備後、次を実行する。

```bash
python3 scripts/run_from_drawing.py drawings/assembly_a.png
python3 scripts/run_from_drawing.py drawings/assembly_b.png
```

画像読解・検証に失敗した場合はロボットを起動しない。生成指示は`outputs/current_ai_task.json`に保存する。GUI表示は`--gui --keep-open`を指定し、先にGUIサービスを起動する。同じGUIコンテナに複数のデモを重複起動しない。

## 生成された指示でロボットを実行

```bash
docker compose run --rm dev python3 scripts/run_insertion.py --task outputs/ai_task_a.json
docker compose run --rm dev python3 scripts/run_insertion.py --task outputs/ai_task_b.json
```

GUIを使う場合は、[GUI起動手順](gui.md)に従って起動し、次を実行する。

```bash
docker compose --profile gui exec gui /ros_entrypoint.sh python3 scripts/run_insertion.py --task outputs/ai_task_b.json --gui --start-delay 10 --keep-open
```

## 参考

- [モデルと配布サイズ](https://ollama.com/library/qwen3-vl/tags)
- [Ollamaの画像入力](https://docs.ollama.com/capabilities/vision)
- [画像入力と構造化出力](https://docs.ollama.com/capabilities/structured-outputs)
- [クラウド無効化・モデル保存先](https://docs.ollama.com/faq)
