# 構成と環境確認

## 実行環境

講義で利用したUbuntu環境を使用する。現在のMacはApple Siliconで、PATH上にros2・gzはなく、既定のPythonにはrclpy・genesisがない。Docker CLIはあるが、確認時にはDockerデーモンへ接続できなかった。

Ubuntu側のバージョンと接続方法は確認待ち。まず既存の講義環境を使い、新規インストールやバージョン変更は互換性確認後に判断する。

新規環境が必要な場合の候補はUbuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic。既存環境がHumble＋Fortressなら、その組み合わせを優先する。

参考：[Gazebo公式のROS互換表](https://gazebosim.org/docs/ionic/ros_installation/)

## 処理の分担（設計案・未実装）

1. 図面読解：画像対応AIモデルで図面画像を作業指示に変換する。
2. 検証：登録部品、単位、座標範囲、向き、穴配置との整合性をチェックする。
3. 作業管理：部品ごとにピック・移動・挿入・解放を順に実行する。
4. ロボット制御：逆運動学や経路計画でアームとグリッパを制御する。
5. Gazebo：物理状態、関節状態、カメラ画像などを提供する。
6. 結果確認：部品位置・向き・挿入深さを確認し、結果を表示する。

ROS 2で各処理を連携する。作業全体は結果・進捗・キャンセルを扱えるAction、状態表示はTopicの利用を想定する。具体的なインターフェースはロボットモデルと制御方式の選定後に確定する。

最初は固定した作業指示で部品1個を挿入する。次に画像から生成した指示へ置き換え、同じ制御コードで図面A・Bを処理する。

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
