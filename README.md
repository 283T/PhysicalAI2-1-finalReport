# 図面読解によるワイヤーハーネス接続デモ

Physical AI応用1講座の最終課題。多品種少量生産を想定し、一品一様の図面・基板とランダム配置された複数端末ハーネスを扱うデモを開発しています。

## 現在の目標：双腕・柔軟ワイヤー・画像認識

搬送されてくるハーネスをカメラで認識し、2本の関節型ロボットアームでコネクタとワイヤーを支え、図面で指定された基板の受け側へ接続します。柔らかいワイヤーの変形・垂れ・揺れを物理的に再現します。

[双腕工程の設計](docs/factory-design.md)に基づき、柔軟ケーブル近似・搬送台・RGB-D認識を実装しています。[MELFA RV-4FRL×2](docs/melfa.md)を現在の一括デモに使い、公式UR3e×2も選択できます。両モデルの左右独立制御は確認済みですが、把持・ワイヤー支持・全端末挿入の統合は未完了です。

## 追加要件：複数端末・ランダム配置・一品一様の基板

1本の分岐ハーネスに付いた全コネクタを接続します。最初は3端末を扱い、基板と図面を同じ設計データから生成し、初期配置を別のseedでランダム化します。[生成・ローカルAI読解・検証の手順](docs/multi-connector.md)。全端末を双腕で接続する統合制御は未完了です。

## MELFA双腕とRGB-D端末認識の実行

```bash
python3 scripts/run_perception_demo.py --design-seed 101 --placement-seed 2
```

MELFAシーン生成からカメラ画像の受信、全3端末の検出まで実行します。`--require-6d`で面識別マーカー付き試作端末の中心・立体姿勢を必須にできます。[6D姿勢・把持候補の検証](docs/grasp-candidates.md)。[精度評価と制約](docs/perception.md)。把持・挿入動作はまだ統合していません。

## 図面と認識結果からのIK計画

```bash
python3 scripts/run_perception_demo.py --design-seed 101 --placement-seed 2 \
  --output outputs/reach_pipeline --evaluate --require-6d --plan-reach
```

ローカルAIで今回の図面を読み、全端末の認識結果と対応させ、把持候補と端末・接続先の上方へ届く腕と関節角を計算します。`--check-reach`では腕ごとの隔離シーンで到達を検証します。[到達性確認の手順と制約](docs/reachability.md)。双腕での把持・挿入と衝突回避は未実装です。

## 接触による端末把持の単独試験

双腕と同じ指を直動治具に載せ、柔軟ケーブル付き端末の持ち上げ・解放を確認しました。摩擦ゼロでは保持できないことも検証しています。[実行手順と比較記録](docs/contact-grasp.md)。MELFAアームと画像認識を使った把持は次の統合対象です。

## これまでの基礎デモ

- 図面A：ハーネスWH1のプラグP1をJ1へ接続。
- 図面B：同じWH1/P1をJ2へ接続。
- Mac上のOllama / Qwen3-VL 2Bで画像からハーネス番号・プラグ番号・接続先・座標・向きを抽出。
- 指示を検証し、ROS 2 Jazzy＋Gazebo Harmonicの直交型ロボットでピック・接続・解放。
- 短い2本のケーブル付きプラグと、パネル上の受け側コネクタを使用。

ケーブルはプラグに付いた**固定形状の視覚モデル**です。変形、張力、絡み、他端が固定されたハーネスの引き回しは再現していません。接続は解放後の位置・姿勢・差し込み深さで確認し、電気的導通やラッチ係合は判定しません。部品1種類・1プラグ・向き0度の簡略版です。

## 実行

[Dockerの準備](docs/setup.md)と[ローカルAIの準備](docs/local-ai.md)を済ませて実行します。

```bash
docker compose build
python3 scripts/run_from_drawing.py drawings/assembly_a.png
python3 scripts/run_from_drawing.py drawings/assembly_b.png
```

[図面・AI出力・動作を並べる画面](docs/dashboard.md)：

```bash
docker compose --profile gui up -d --build gui
python3 scripts/serve_demo.py
```

[デモ画面](http://127.0.0.1:8080/)で図面を選んで実行します。[MP4録画](docs/recording.md)も利用できます。

ハーネス版の図面読解は2/2件一致し、J1/J2への接続動作が成功しました。120秒・等速の録画試作も確認済みです。

## 検証と開発計画

[最新の検証記録](docs/validation.md)、[接続動作と簡略化](docs/insertion-demo.md)、[図面A・B](docs/drawing-variants.md)、[開発計画](docs/development-plan.md)を参照してください。

以前の電子部品挿入デモの結果は[過去の検証記録](docs/archive/pcb/validation.md)に保存しています。ハーネス接続の検証とは分けて扱います。

## 構成と保守

実行入口、共通モジュール、保存記録と生成物の区別は[構成と保守](docs/maintenance.md)にまとめています。

```bash
python3 scripts/clean_generated.py  # 既知の不要な生成物をプレビュー
uvx ruff check scripts tests
```

## 提出予定

1〜3分の動作動画、公開GitHubのコード、目的・AIの役割・工夫・結果・制約をまとめたPDFレポート。実在の工場の機密図面や個人情報を含めず、自作の図面を使用します。ローカルモデルの重みとログ・動画はGitの管理対象から除外します。
