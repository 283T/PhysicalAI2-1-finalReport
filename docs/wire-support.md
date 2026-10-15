# RGB-Dによるワイヤー支持候補

図面の接続指定はローカルAIが読み、動作用の支持点は新しい登録済みRGB-D画像から求める。シミュレータの物体位置を目標生成へ渡さない。

現行プロトタイプでは枝ごとの色、8 mmの正方形断面、観測したコネクタ中心を使用する。中心から75–145 mmの範囲で局所平面を当て、幅5–11 mm・長さ20 mm以上・平面残差0.6 mm以下の部分を候補とする。表面から4 mm内側をワイヤー中心、さらに2 mm内側を指先TCPとする。工具Yを接線、Zを接近方向へ向ける。隠れた部分、絡み、一般的な丸断面・無着色ワイヤーの認識を保証する方式ではない。

新規受信は次で実行できる。既存Gazeboシーンのカメラが必要。初期状態からの全3枝の観測は2026-10-16に約52.44秒、同じ落ち着いたシーンでの再取得は約2.65秒だった。後者の連続安定数は保存した候補から再計算して一致を確認した。3フレームの窓全体で位置2 mm・姿勢0.05 rad以内を要求する。これは認識用の開発ゲートであり、最終課題の挿入成功基準とは別。

```bash
docker compose exec -T gui bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 scripts/observe_connectors.py --bridge --wire-supports --output outputs/wire_support_observation'

# 保存した入力から一枚の候補を再生成。ロボットは動かさない。
docker compose run --rm dev python3 scripts/wire_support_vision.py \
  --source docs/evaluation/wire_support_rgbd_20261016/live_settled_repeat \
  --output outputs/wire_support_replay
```

[入力・計算結果](evaluation/wire_support_rgbd_20261016/live_settled_repeat/wire_supports.json)と[連続判定の再評価](evaluation/wire_support_rgbd_20261016/live_settled_repeat/replay_validation.json)を保存。支持点の真値との誤差、可到達性、接触・安定把持はまだ評価していない。最小開口16 mmの既存指では8 mmワイヤーを挟めないため、支持用の指と開口計測の変更が次の工程となる。


## 支持指と接触試験（2026-10-16）

`--left-wire-gripper` を工場モデル生成とデモ入口へ追加。左腕は指の内側へ各5 mm延長した裸のパッド、右腕は従来のコネクタ保持形状とする。左の最小開口は6 mm、最大指令開口は42 mm。各指の概算質量16.25 gと矩形近似の慣性・重心も更新する。実機同定値ではない。

開口はURDFの接触面と実測した左右の指関節から計算する。8 mmワイヤーが左右約1 mmの移動で指を止めた場合は8 mm、空閉じなら6 mmとなる。20 mmハウジングの開口基準も維持し、異なる指形状で固定の16 mmを加算して誤判定しない。

直動試験台で12節の柔軟ケーブルの中間リンクを接触把持した。既知位置の機械試験であり、カメラ・MELFA・双腕制御・基板挿入の試験ではない。全失敗と全軌跡を [実験記録](evaluation/wire_grip_contact_20261016) に保存。軌跡はデータを減らさず `trajectory.json.gz` に圧縮し、`gzip.open` または `gunzip -c` で読める。

時間指定だけの接近では工具が台の高さ0.140 mへ下がり、閉指開口15.41 mmで停止。実測到達待ちでもP=500／I=100／D=20、P=5000／I=100／D=60、P=5000／I=1000／D=60の3条件は30秒内の0.5 mm到達基準を満たさず閉指前に停止した。保持は未実行。PやIを上げるだけでは今回の接近・待機が成立しなかった。

昇降試験台をP=5000／I=0／D=60とした試験では到達誤差0.358 mm、閉指開口8.000 mm。対象ワイヤーの最小上昇99.60 mm、持ち上げからの位置関係の変化0.295 mm、最大回転0.02513 radで、既存の70 mm超／5 mm未満／0.05 rad未満の保持基準を通過。開放後99.53 mm下がった。支持指のPIDや接触摩擦はこの昇降軸の比較と区別する。

摩擦ゼロの負の対照は、工具が約100 mm上がっても対象ワイヤーは上昇せず、開口6 mm・位置関係の変化約100 mmで保持を拒否した。`expectation_met=true` は失敗を検出できたという意味で、作業成功ではない。

```bash
docker compose exec -T gui bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 scripts/grasp_benchmark.py --wire-grip --output outputs/wire_grip'
docker compose exec -T gui bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 scripts/grasp_benchmark.py --wire-grip --friction 0 --expect-slip --output outputs/wire_grip_zero'
python3 scripts/export_wire_grip_trial.py --source outputs/wire_grip --output outputs/wire_grip_archive
```

同じROSドメイン・トピックを使う機械試験同士は同時実行しない。各試験のGazeboは別パーティションで起動・終了する。

## MELFAへ移すための経路検査

新しい左右非対称のURDFを生成し、保存したRGB-D候補に対し左腕は3枝ともホバー／把持の6D IKを通過した。P3では右腕が直立待機中なら左腕のホバー・接近・単独上昇の両腕包絡箱／設備検査を通過。一方、過去にP3を保持した右腕の関節姿勢を置いた検査は、支持指の通常向き・180°反転とも経路検査で拒否した。

[検査入力と結果](evaluation/wire_pickup_plan_20261016) は実行済みの双腕把持を意味しない。検査は支持側の単独上昇も含んでおり、最終把持姿勢そのものが不可能だとは断定しない。次は拒否された工程を特定し、ワイヤー支持とコネクタ把持の順序、両腕同時上昇、代替の支持点を計画する。経路の合格を把持や挿入の成功と扱わない。


同じPD設定での独立した再起動・再試験も通過した。最小上昇99.60 mm、持ち上げからの位置関係の変化0.297 mm、回転0.02556 rad、開放後落下99.52 mm。正の試験2回と摩擦ゼロ1回、途中4失敗の計7試験を保存した。結果一覧は [summary.json](evaluation/wire_grip_contact_20261016/summary.json)。136件の単体テストと保存ファイルのハッシュ・圧縮軌跡の読出しも検証した。
