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


## MELFAの直接支持試験（2026-10-16）

現在の入口は `scripts/pick_wire.py`。左腕の支持点を新しい上面RGB-Dから選び、側面の新しいRGB-Dでワイヤー中心・接線と工具の位置・姿勢を再判定してから閉じる。両腕と固定設備の検査に停止中の搬送パレットも加え、実行中のパレットの移動を0.5 mm／0.002 rad以内へ制限する。自己衝突やケーブル全体の絡みの予測は未実装。

接触メッセージは接触のない状態では届かないため、起動時には実衝突を待たず、SDFに記載した30個の接触センサーの配信元接続を確認する。接続消失・腕同士や設備との接触は停止する。失敗時の指開放は、通常の幾何・鮮度検査が例外になっていても送信し、腕への新しい移動指令は送らない。開放時の物体の落下は、この試験の成功条件には含めていない。

保存した以前の画像を使う幾何探索で、110 mm候補は右腕のP3保持姿勢と前腕が干渉、170／200 mm候補は接近経路を通過した。これらは一枚の画像の探索であり、観測安定数を0として扱い、物理試験とは区別した。実行用プランナーは既定で3回の安定観測を要求する。

新規の170 mm観測では、色の境界に台の奥行きが混ざり、単一平面への全点当てはめが失敗した。0.5 mmの距離内にある支配的平面をRANSACで選び、24点かつ60%以上の実測点を要求する。平面残差0.6 mm以下、幅5–11 mm、長さ20 mm以上の条件は維持。外れ値の人工的な補完や判定値の拡大は行わない。

ワイヤー面が傾いている場合は、指を世界Zに対して垂直に保つ。8 mm正方形断面の二つの横軸を実際の顎方向へ投影し、予測開口を求める。閉指差1 mm未満の条件はそのまま使用する。これは幅の測定モデルを姿勢へ対応させる変更であり、ワイヤーを円筒へ置き換えたり、保持の回転基準を緩めたりするものではない。

[7件の試験一覧](evaluation/melfa_wire_pick_20261016/summary.json)を保存した。全件失敗。初期パレットとの重なり、接触のない起動時の待機、深度境界、傾けた指と台の接触、包絡箱の重なり、閉指後の隙間不足を順に検出・修正した。最後の試験は画像再確認と閉指を通過し、約95.65 mm上昇したが、保持中の回転0.414 radで拒否した。位置関係の最大変化4.52 mm、実接触による腕同士・設備の衝突は0件。

回転を分解するとワイヤー軸回り0.093 rad、曲げ方向0.403 radが主で、隣り合う2節の端へ指が接触していた。現行の接触長20 mmでは荷重を受けて曲がる余地が残る。次は接触長の拡大を試し、質量・慣性と経路検査を更新する。MELFAの保持、双腕協調搬送、基板挿入はまだ通過していない。

```bash
# 事前に新しい非対称モデルと対応シーンを生成・起動しておく。
docker compose exec -T gui bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 scripts/pick_wire.py --world outputs/melfa_wire_pick_notch16_20261016/case/world.sdf --models outputs/melfa_regrasp_worlds_g3000_capture_front_right_fence8_end_stops_tip2_left_wire --output outputs/wire_pick_new'

# 終了した試験だけを、コード・画像・深度・全軌跡と一緒に新規ディレクトリへ保存。
docker compose run --rm dev python3 scripts/export_wire_pick_trial.py \
  --case outputs/melfa_wire_pick_notch16_20261016/case \
  --pickup outputs/wire_pick_new \
  --models outputs/melfa_regrasp_worlds_g3000_capture_front_right_fence8_end_stops_tip2_left_wire \
  --output outputs/wire_pick_new_archive
```

初期状態への再起動が必要。途中姿勢から開始する経路は現在の入口では未実装。アーカイブの `validation.json` が終了結果の正本で、途中の `progress.json` より優先する。
