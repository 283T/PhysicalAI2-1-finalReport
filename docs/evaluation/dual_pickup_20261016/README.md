# コネクタ先行・空中ワイヤー支持の比較

固定D101／配置2／P3の開発試験。基板挿入も最終12試行も未実行。各試験の失敗を保存し、途中工程の通過を全体成功へ置き換えない。結果の正本は各 `pickup/validation.json`、集計は `summary.json`。

| 保存先 | 条件・到達点 | 停止 |
|---|---|---|
| full_stroke_preflight_rejected | 初期の右接近 | 実開口にかかわらず全指ストロークを評価した箱がパレットと重なる |
| primary_rear_fence_closure_rejected | 実測開口の±0.5 mmで箱を構築し、実行中も範囲を監視 | コネクタ位置合わせ後、閉指開口24 mmで拒否。後方ガイド接触 |
| compensated_servo_tracking_rejected | 工具の実測誤差を補償する画像補正 | 2回目の補正で関節追従の時間上限 |
| both_grasped_separation_rejected | 関節P×4／D×2、Iと50 Nm上限は維持。右保持、左110 mm支持点の再観測・閉指まで通過 | 協調上昇の工具間120 mm制限で計画を拒否 |
| support140_tracking_rejected | 支持点140 mm。協調上昇までの保存画像による幾何検査は通過 | 左接近後、関節6の誤差0.010248 radが0.01 rad未満に達せず停止。左閉指前 |
| support140_settle30_dual_hold | 左接近後の待機だけ20→30秒。両腕100 mm上昇・4秒保持を通過 | この試験範囲では成功。治具搬送・持ち替え・基板挿入は未実行 |

20秒待機の140 mm試験では右の最初の閉指状態から6319個の新規観測を連続評価し、最大相対移動0.278 mm・回転0.008674 rad。右の把持継続を示すが、左の把持や双腕上昇の成功ではない。ログ再計算でも一致を確認した。`scripts/replay_grip_evidence.py` は記録の整合性と作業成功を別々に出力する。前の110 mm試験では連続監視の基準が右の保持後だったため、現在の最初の閉指基準による全工程検証とは区別する。

30秒待機の試験では左の到達条件を満たし、再撮影の工具誤差0.668 mm／0.01008 rad、閉指開口8.000 mmを通過した。双腕上昇後の最小上昇は左99.735 mm・右99.984 mm、持ち上げによる滑りは左0.024 mm・右0.188 mm、姿勢変化は左0.00541 rad・右0.00226 rad。各1042観測の4秒保持を通過し、意図しない腕同士／設備接触は0件。最初の閉指からの連続評価は右12200観測で最大0.323 mm／0.01024 rad、左3438観測で最大0.013 mm／0.000199 rad。`dual_hold_grip_replay.json` で生ログから再計算した。単一固定条件・単発の開発試験であり、全体の接続成功や再現成功率を示さない。

`saved_dual_hold_transfer.json` はこの成功試験の最後の保持中の実測関節と、最初の画像に由来するコネクタ／工具の関係を用いた次工程の検査。停止パレットを含み、実測指位置±0.5 mmで箱を構築した。4候補目（高さ0.35 m、先に回転してから並進、支持点の世界Z回り軌道0）で治具への全5工程が箱検査を通過した。固定治具の幾何を目標にし、物体真値を使ってコネクタ目標を作っていない。これは保存状態のオフライン結果で、実際の搬送・治具着座ではない。実行前に新規の保持画像を取り、経路を作り直し、指範囲と最初の閉指からの把持を監視する必要がある。

`saved_rgbd_transit` は保存した画像による探索で、物理試験ではない。110 mmで通ったのは6関節の同時補間。従来の腰軸先行補間では干渉していた。実装は同時補間を先に試し、必要なら有限回数・固定seedのRRT-Connectで迂回する。通った実例でランダム木の迂回を使ったとは主張しない。離散的な腕・固定設備の箱検査であり、自己衝突やワイヤー全体の予測を保証しない。

## 再実行

Docker、MELFAメッシュの準備はリポジトリの手順に従う。worldのメッシュ参照 `/workspace/outputs/melfa_install` は `scripts/setup_melfa.sh` の生成先。Gazeboとコントローラは同じROSドメイン内で各一つだけ起動する。以下は保存された物理モデルから新しい試験を行う手順であり、成功済みデモではない。終了・プロセス不在を確認してからシーンを初期化する。

```bash
docker compose restart gui
docker compose exec -T gui python3 scripts/start_desktop.py --wait-only
docker compose exec -d gui bash -lc 'source /opt/ros/jazzy/setup.bash && gz sim -s -r docs/evaluation/dual_pickup_20261016/support140_tracking_rejected/case/world.sdf > outputs/dual_replay_scene.log 2>&1'
docker compose exec -T gui bash -lc 'source /opt/ros/jazzy/setup.bash && python3 scripts/dual_pickup.py --world docs/evaluation/dual_pickup_20261016/support140_tracking_rejected/case/world.sdf --models docs/evaluation/dual_pickup_20261016/support140_tracking_rejected/models --drawing-plan docs/evaluation/dual_pickup_20261016/support140_tracking_rejected/pickup/connection_plan.json --wire-distance-mm 140 --output outputs/dual_replay/pickup'
docker compose run --rm dev python3 scripts/replay_grip_evidence.py --pickup docs/evaluation/dual_pickup_20261016/support140_tracking_rejected/pickup --output outputs/dual_grip_replay.json
```

コードには左接近の到達後待機30秒への比較変更を含む。上の過去試験は20秒だった。位置2 mm・角度0.025 rad・関節0.01 rad未満の到達条件、把持5 mm・0.05 rad未満、工具間120 mm、腕同士5 mm・設備1 mmの包絡余裕は維持する。開指状態の箱は実測指位置±0.5 mmを含み、実行中に範囲を外れれば停止する。未知の指位置を都合よく固定して通す方式ではない。
