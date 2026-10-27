# 2026-10-27時点の提出用成果物

OmniCampusアカウント名：`283t_123`

**接続未達を含む開発成果の提出版。元の全コネクタ接続Goalは未完了。**
初期版の簡略接続と現行双腕の物理把持・保持を区別し、成功していない挿入を成功表示しない。

## 提出ファイル

リポジトリ直下の `outputs/submission_20261027/deliverables/` に生成する。

- `説明資料_PAI最終課題_283t_123.pdf`：4ページ。目的、AI、結果、失敗、制約、再実行手順。
- `PAI最終課題_283t_123.mp4`：120秒。音声なし。初期版の実録抜粋は等速。
- `コード_PAI最終課題_283t_123.zip`：プログラム一式、関連初期入力、最新試験の生ログと再監査結果。
- `SHA256SUMS.json`：提出物のSHA-256。

OmniCampusへ登録するコードURL：<https://github.com/283T/PhysicalAI2-1-finalReport>
ZIPは手元保存・補足用。過去の大容量回帰データは全て含めておらず、過去を含むテスト一式にはGitHubの完全なcheckoutを使う。
モデル重み・外部モデルメッシュは含めない。

## 動画構成

| 時間 | 内容 | 証拠の種類 |
| --- | --- | --- |
| 00:00-00:08 | 目的と到達点 | 説明カード |
| 00:08-00:36 | 初期版A：P1→J1 | 実際のGazebo録画を等速で抜粋 |
| 00:36-01:04 | 初期版B：P1→J2 | 実際のGazebo録画を等速で抜粋 |
| 01:04-01:22 | 現行双腕のハーネス保持 | 保存RGB画像。動作動画ではない |
| 01:22-01:40 | 実測と独立再計算 | 数値の説明カード |
| 01:40-02:00 | 未達点と次の工程 | 説明カード |

初期版は直交型ロボット・固定ジョイント把持・変形しない視覚ケーブルを使う。
現行のMELFA双腕・接触把持・柔軟ハーネスの接続成功には数えない。
カットは区間間だけで、実録抜粋内の倍速・時間圧縮は行わない。
旧動画のSHAは `docs/evaluation/video_validation.json` と照合してから使用する。

## 今回の再確認

- 最新物理試験は `67633a0` で開始した比較試験。終了判定は失敗。
- 右単独と双腕の持上げ・保持は報告上通過。治具搬送 `nest_transfer_1` で時間予算を使い切った。
- 全把持ログの独立再計算は一致。右126,037／左90,375標本、記録上の把持は基準内。
- 接触再監査は、指定ソケットの必要な記録区間がないため不合格・exit 1。
  これは失敗した開発試験に対する想定結果であり、成功扱いしない。
- 10/27の再確認時はDockerデーモンが停止していた。今回、新規推論・物理試験・クリーン環境からの全工程再現は行っていない。

## 保存ログを再計算する

コードZIPを展開すると `submission_evidence/` に生ログがある。
Python 3.9以上とNumPyを用意し、リポジトリ（または展開先）のルートで実行する。
再計算は失敗した試験の記録を検査するもので、物理を再実行するものではない。

```bash
python3 scripts/replay_grip_evidence.py \
  --pickup submission_evidence/pickup --output /tmp/submission-grips.json
python3 scripts/replay_contact_evidence.py \
  --pickup submission_evidence/pickup --case submission_evidence/case \
  --output /tmp/submission-contacts.json
```

2番目のコマンドは、指定ソケット接触の記録がないためexit 1が期待結果。
`arm_contact_audit_passed=true` だけを全体成功へ読み替えない。
`submission_evidence/manifest.json` で原本とコピーのSHAを対応付ける。

## 成果物を再生成する

生成用の固定依存バージョンは `requirements.txt` に記載（Python 3.11以上）。
Pillow、ReportLab、FFmpeg、FFprobeと日本語TrueTypeフォントが必要。
日本語フォントはMacではArial Unicode、LinuxではIPAexゴシック等を `--font` に指定する。
ZIP展開先では次を実行する。

```bash
python3 scripts/build_submission.py --account 283t_123 \
  --source-evidence submission_evidence --output outputs/submission_rebuild \
  --font /path/to/JapaneseFont.ttf
```

生成スクリプトは実験判定を変更しない。動画・PDFは再生成できるが、展開ZIPから再度作るコードZIPの内容は、Git管理情報の有無で変わる。
PDFの生成日時等のメタデータによって、再生成ファイルのSHAが元と一致しない場合がある。

## 物理実験の再実行

初期版は `docs/setup.md`、`docs/local-ai.md` に従い、次を実行する。

```bash
python3 scripts/run_from_drawing.py drawings/assembly_a.png
python3 scripts/run_from_drawing.py drawings/assembly_b.png
```

現行版は `docs/reproduce-fixed-trial.md` の初期入力準備と起動手順を使う。
この手順の搬送オプションは以前の剛体協調比較のため、今回の独立支持比較には
**`--rigid-nest-transfer` を削除し、`--nest-orientation flipped` を追加**する。
`--place-in-nest --feedback-paced-transfer --centred-grips` と後段オプションは維持する。
保存した終了姿勢や関節値は復元せず、初期worldから新規観測・計画で実行する。
この比較試験は失敗しており、接続成功を保証する再現手順ではない。

## 公開・提出の準備

動画タイトルは `PAI最終課題_283t_123`。概要欄の案は `youtube_description.txt`。
ユーザーの最終確認まではYouTubeへのアップロード・公開、OmniCampus提出を行わない。
限定公開または公開の選択、子ども向け・年齢制限の設定、URL登録は最後に確認する。
