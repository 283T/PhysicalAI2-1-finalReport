# 図面読解による電子部品のロボット挿入デモ

Physical AI応用1講座の最終課題。多品種少量生産を想定し、基板組立図を差し替えることで、電子部品の挿入位置・向きを変更できるシミュレーションデモを開発します。

## 目標

組立図の画像をAIで読み取り、部品番号・種類・基板上の座標・向きを抽出します。抽出結果を検証してロボット座標へ変換し、アームが供給トレイから部品をピックして基板に挿入します。

同じ制御コードで図面A・Bを順に組み立て、品種切り替えに対応する様子を示します。

## 最初の実装範囲

- Ubuntu 24.04ベースの専用Docker環境でROS 2 Jazzy＋Gazebo Harmonicを利用します。
- 部品は拡大した2ピンコネクタ1種類。
- 基板組立図は2種類、各図面に部品1個。図面AはJ1、BはJ2に挿入。
- 図面には基板原点、単位、挿入座標、部品の向きを明記。
- 基板と供給トレイは固定し、部品は既知の位置・向きで整列。
- 穴とピンの寸法を簡略化し、まず垂直挿入を実現。
- Mac上のOllamaでQwen3-VL 2Bの推論を利用。外部AIサービスには送信しません。

## 構成

```text
基板組立図の画像
  → AIによる部品・座標・向きの抽出
  → 部品登録・座標範囲・穴配置の検証
  → 基板座標からロボット座標への変換
  → ピック・移動・挿入・解放
  → 挿入結果の確認
```

AIの読み取りとロボット制御を分離し、読解結果を画面に表示して動作との対応を説明します。接触や把持を簡略化する場合は、実際に再現した範囲をレポートに記載します。

## 使用技術と実行環境

- **ROS 2**：図面読解、作業指示の検証、動作実行、結果報告を連携します。
- **Gazebo**：アーム、基板、部品、把持・挿入時の物理動作をシミュレーションします。
- **Genesis**：必要に応じて挿入動作の試作・比較に利用する候補です。最初のデモはROS 2＋Gazeboで構成します。
- **画像対応AIモデル**：組立図から部品・座標・向きを抽出します。ローカルの`qwen3-vl:2b-instruct`を使用します。

指定された講義教材はDebianベースのPushT/RoboOps用Docker環境だったため、この課題用のDocker環境を別に用意します。Mac上でDockerを実行し、まずGazeboサーバーをGUIなしで確認します。[起動手順](docs/setup.md)と[構成](docs/architecture.md)を参照してください。

## 開発状況

ローカルAIが図面画像から部品・挿入箇所・座標・向きを抽出し、ROS 2＋Gazeboの直交型ロボットがピック・挿入・解放します。同じ制御コードで図面AはJ1、BはJ2へ挿入できることを確認しました。今回の簡略図面2枚では読解が2/2件一致し、両方の挿入が成功しました。[検証結果](docs/validation.md)を参照してください。

Dockerをビルドし、[ローカルAIの準備](docs/local-ai.md)を済ませてから実行します。

```bash
docker compose build
python3 scripts/run_from_drawing.py drawings/assembly_a.png
python3 scripts/run_from_drawing.py drawings/assembly_b.png
```

AIを使わず動作だけ確認する場合は、`docker compose run --rm dev python3 scripts/run_insertion.py`で手書きの基準指示を実行できます。[図面A・B](docs/drawing-variants.md)と[物理動作の説明](docs/insertion-demo.md)も参照してください。対応範囲は部品1種類・各1個・向き0度です。

図面・AI出力・動作を並べた[デモ画面の起動手順](docs/dashboard.md)を用意しています。Gazebo単体の表示は[GUI起動手順](docs/gui.md)を参照してください。
[録画手順](docs/recording.md)で、図面・AI出力・実際のGazebo動作を並べたMP4を生成できます。約2分の録画試作を確認済みです。
開発の順序と完了条件は [開発計画](docs/development-plan.md) に記載しています。

## 提出予定

- 1〜3分の動作動画。倍速を使う場合は明記。
- 使用コード一式をこの公開リポジトリに配置。
- 目的、AIの役割、独自の工夫、結果、制約をまとめたPDFレポート。

成果物に氏名・所属・メールアドレスなどの個人情報や、実在の工場の機密図面を含めません。デモ用の図面と部品モデルを用意します。ローカルモデルの重みと実行ログはGitの管理対象から除外します。

## 参考

- [Genesis: Inverse kinematics and motion planning](https://genesis-world.readthedocs.io/en/latest/user_guide/robot_control/inverse_kinematics_motion_planning.html)
- [課題サンプル: UR3 + Robotiq Pick and Place](https://www.youtube.com/watch?v=t4L4VAfgqZw)

参考コードを導入する際はライセンスを確認し、利用元と変更点を記録します。
