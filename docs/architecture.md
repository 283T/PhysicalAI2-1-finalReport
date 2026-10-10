# 構成と環境確認

## 実行環境

現在のMacはApple Siliconで、PATH上にros2・gzはなく、既定のPythonにはrclpy・genesisがない。指定された講義教材はDebian BookwormベースのPushT/RoboOps用Docker環境で、ROS 2・Gazeboの設定は含まれていなかった。

この課題用にROS公式Dockerイメージを使った専用環境を用意し、Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonicを採用する。起動方法は[setup.md](setup.md)を参照。

GazeboサーバーとROS連携を確認済み。追加のGUI用DockerサービスでXvfb・noVNCを使い、ブラウザから実際のGazebo画面を確認できる。動画録画は別途整備する。

参考：[Gazebo公式のROS互換表](https://gazebosim.org/docs/ionic/ros_installation/)

## 処理の分担

1. 図面読解：Mac上のOllama/Qwen3-VL 2Bで図面画像を作業指示JSONに変換する。
2. 検証：登録部品、単位、座標範囲、向き、穴配置との整合性をチェックする。
3. 作業管理：部品ごとにピック・移動・挿入・解放を順に実行する。
4. ロボット制御：基礎デモは直交型ロボットのX・Y・Z関節へ位置指令を送る。把持は固定ジョイントで簡略化する。
5. Gazebo：物理状態、関節状態、カメラ画像などを提供する。
6. 結果確認：部品位置・向き・挿入深さを確認し、結果を表示する。

画像読解と指示の検証はMac上で実行し、検証済みJSONをDocker内の実行スクリプトへ渡す。ROS 2のTopicで関節指令、関節状態、部品姿勢、把持状態をGazeboと連携する。`run_from_drawing.py`で画像入力から挿入まで一括実行できる。

図面A・Bの画像をローカルモデルで読み、同じ制御コードでJ1・J2へ挿入する処理は検証済み。回転軸・複数部品の連続挿入・録画用の情報表示は未実装。[ローカルAI](local-ai.md)と[検証記録](validation.md)を参照。

## Genesisの位置づけ

挿入動作を独立して試す場合の追加候補とする。Gazeboと同時に同じ物理状態を制御する構成は採用せず、共通の作業指示を別のシミュレータで実行する比較を想定する。追加は最小デモの完成後に判断する。

参考：[Genesisのインストールと対応環境](https://genesis-world.readthedocs.io/en/latest/user_guide/overview/installation.html)

## Ubuntu側で確認する項目

ROS環境を有効化したターミナルで確認する。以下は情報取得用で、インストールは行わない。

```bash
cat /etc/os-release
uname -m
printenv ROS_DISTRO
command -v ros2
command -v gz
command -v ign
command -v gazebo
python3 --version
ros2 doctor --report
```

Gazeboのコマンド名は世代で異なるため、見つかったコマンドでバージョンを確認する。SSH接続先、作業ディレクトリ、GUIを表示する方法、利用可能なアームモデルも確認する。
