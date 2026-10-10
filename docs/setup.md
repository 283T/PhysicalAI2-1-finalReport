# 開発環境の起動

## 採用環境

Ubuntu 24.04ベースのROS 2 Jazzy公式Dockerイメージに、Gazebo Harmonicとros_gzを追加します。Apple SiliconではDockerが対応するネイティブアーキテクチャを選択します。まずGUIなしでサーバーとROS連携を確認し、動画用の表示環境は後から追加します。

指定された講義教材はDebian Bookworm＋Python 3.12のPushT/RoboOps環境でした。ROS 2・Gazeboのインストール設定はないため、この課題用に別のDockerイメージを作ります。

## ビルド

Docker Desktopを起動した状態で、このリポジトリのルートから実行します。

```bash
docker compose build
```

## 基本確認

```bash
docker compose run --rm dev bash -c 'printenv ROS_DISTRO; ros2 pkg prefix ros_gz_bridge; gz sim --version'
```

## Gazeboサーバー

サーバー起動とGazebo→ROS 2の`/clock`受信をまとめて確認するには、次を実行します。30秒以内にシミュレーション時刻を受信できれば成功です。確認用プロセスは終了時に停止します。

```bash
docker compose run --rm dev python3 scripts/check_sim.py
```

サーバーだけ起動する場合：

```bash
docker compose run --rm dev gz sim -s -r /workspace/sim/worlds/smoke.sdf
```

この段階のworldは床だけで、アームや基板はまだ含みません。停止はCtrl+Cです。

## 開発用シェル

```bash
docker compose run --rm dev
```

コンテナの`/workspace`に、このリポジトリがマウントされます。起動時に公式イメージのentrypointがROS環境を設定します。

ベースイメージは取得したダイジェストで固定しています。aptパッケージはバージョン未固定のため、提出時に実際の依存バージョンを記録します。

## 動作確認記録（2026-10-10）

- 実行環境：Apple Silicon Mac上のDocker Desktop、Linux aarch64コンテナ。
- Ubuntu：24.04.5 LTS。
- ROS 2：Jazzy。
- Gazebo Sim：8.15.0（Harmonic）。
- Dockerイメージのビルド：成功。
- `scripts/check_sim.py`：成功。GazeboからROS 2へ届いた時刻が進むことを確認。

直交型ロボットによる部品1個のピック・挿入も実装・確認済みです。詳細は[基礎デモ](insertion-demo.md)を参照してください。[ブラウザ経由のGUI表示](gui.md)も確認済みです。ロボットのカメラセンサー、動画録画、AIによる図面読解は未実装・未確認です。
