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
