# 構成と保守

## 現在の入口

| 目的 | 実行スクリプト | 補足 |
| --- | --- | --- |
| MELFA双腕シーンとRGB-D認識 | `scripts/run_perception_demo.py` | 現在の開発対象。`--evaluate`で精度評価も実行 |
| 図面・認識結果のIK計画 | `scripts/plan_reach.py` | `run_perception_demo.py --plan-reach`で一括実行 |
| IK目標の隔離シーン検証 | `scripts/check_reach.py` | 把持・挿入や衝突回避とは分けて確認 |
| 図面・基板・ランダムハーネス生成 | `scripts/build_case.py` | 設計seedと配置seedを独立に指定 |
| 全端末のローカルAI図面読解 | `scripts/read_connection_plan.py` | 読解後に基板との整合性を検証 |
| 旧1端末デモ | `scripts/run_from_drawing.py` | 直交型ロボットと固定形状ワイヤーの基礎デモ |
| 双腕／柔軟ケーブルの単独確認 | `scripts/check_arms.py` / `scripts/check_cable.py` | 統合動作とは分けて確認 |

CLIの名前と既存の実行コマンドは維持する。旧デモは動作比較と録画用に残している。双腕で全端末を把持・挿入する統合処理はまだ実装していない。

## 共通処理

- `scripts/project_paths.py`：プロジェクトの絶対パス。別のCLIからパス定数を借用しない。
- `scripts/local_vision.py`：Ollamaのloopback通信、画像ハッシュ、監査ログ、不完全な出力の拒否。基板カタログは受け取らず、推論と正解による検証を分離する。新しい通信が失敗したときは古い結果と監査ログを残さない。
- `scripts/perception_stream.py`：RGB/depthの時刻同期、画像の行パディング・エンディアン処理、全3端末の安定判定。ROSをimportしないため単体テストできる。
- `scripts/arm_kinematics.py`：URDF由来の順運動学、数値IK、共通の回転変換。
- `scripts/connector_vision.py`：画像からの色・マーカー検出と座標変換。
- `scripts/observe_connectors.py`：ROS購読・publishと観測結果の保存。
- `scripts/evaluate_perception.py`：評価専用のGazebo正解購読。検出器への正解入力は行わない。
- `scripts/processes.py`：終了待ち・タイムアウト時の強制終了。ROS bridgeなどは新規プロセスグループとして起動し、そのグループを終了する。
- `scripts/start_desktop.py --wait-only`：既存のXディスプレイ起動待ち。一括デモに埋め込んでいたPythonコードを置き換えた。

## ファイルの置き場所

| 場所 | 内容／扱い |
| --- | --- |
| `scripts/`, `tests/` | 実装と回帰テスト |
| `configs/`, `sim/` | 入力設定、モデルのラッパー、起動用シーン |
| `examples/`, `drawings/` | 再現可能な入力サンプル。複数端末worldは当時の生成結果のスナップショット |
| `docs/evaluation/` | 保存する評価結果・画像・監査記録 |
| `docs/archive/pcb/` | 電子部品版の過去の記録 |
| `outputs/` | 実行時の生成物。各新規デモは`case/`と`observations/`を同じ実行フォルダへ保存 |
| `recordings/` | ローカル動画。Gitには含めない |
| `.cache/` | ローカルAI重み・MELFA取得元。Git/Dockerビルドには含めない |
| `outputs/melfa_install/` | シーンが参照するモデルとメッシュ。利用中は残す |

`docs/evaluation/`の記録やハッシュ付きサンプルを消して容量を減らすことはしない。モデル重み、動画、稼働中の`outputs/perception_demo_seed2/`も今回の整理対象から外す。

## 不要な生成物の整理

```bash
python3 scripts/clean_generated.py          # 対象の一覧だけ表示
python3 scripts/clean_generated.py --apply  # 一覧の既知の古い試行を削除
```

削除対象はスクリプト内の明示的なリストと`scripts/`・`tests/`のPythonキャッシュのみ。上面マーカーだけの失敗試行、保存済み評価と重複する古い認識試行、ルート直下の古いcolconログを整理する。ディレクトリ全体の一括削除や、動画・モデルの自動削除はしない。別の古い試行を追加するときは、その結果が必要な記録へ保存済みであることと、実行中のシーンから参照されていないことを確認する。

## 検証と書式

```bash
# 依存ライブラリがそろうGUI環境で、ROSなしの単体テストを実行
docker compose exec -T gui python3 -m unittest discover -s tests -v

# Python書式・import・未使用コードの確認（設定はpyproject.toml）
uvx ruff check scripts tests
uvx ruff format --check scripts tests

# 実際のROS/Gazeboと画像認識を確認
python3 scripts/run_perception_demo.py --design-seed 101 --placement-seed 2 --evaluate

docker compose run --rm dev python3 scripts/check_sim.py
docker compose run --rm dev python3 scripts/check_cable.py
```

Ruffは開発時に使用する。ロボット実行環境の依存には追加しない。

## 整理後の確認（2026-10-10）

[検証記録](evaluation/refactor_validation.json)に結果をまとめた。

- 26件の単体テスト、Ruffの静的検査・書式確認、全Pythonファイルの構文確認を通過。
- 書式だけ変更した設計・図面・world・検出・指示検証の5モジュールは、import以外のASTが変更前と一致。
- MELFA一括認識：全3端末検出、最大XY誤差4.17 mm。変更前の同じseedと一致。
- ローカルQwen3-VLで図面D100の全3接続を再読解し、基板との整合性検証を通過。
- MELFA左右独立制御：最大関節誤差約0.00255 rad。
- ROS/Gazeboの時計連携、13リンクのケーブル重力変形、旧1端末の接続・解放を確認。

これらは個別処理の回帰確認であり、双腕で柔軟ハーネスの全端末を接続した結果ではない。
