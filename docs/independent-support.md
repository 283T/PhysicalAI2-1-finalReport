# ワイヤーを変形させる双腕搬送の比較

37583では、両工具の相対位置・姿勢を固定して回転させた後、水平搬送中から未把持P1／P2が共用治具へ入り込んだ。元の失敗と接触記録は [追加監査](evaluation/inactive_payload_contacts_20261016/README.md) に保持した。

次の比較では左の支持工具の世界座標での姿勢を維持し、右工具の回転に応じてP3側のワイヤーを曲げる。工具間距離を保つ既存の支持移動モデルを使い、実際の曲げ・保持・未把持端子の挙動は物理で検証する。経路計画だけで干渉回避を保証しない。

## オフライン比較

保存済みのロボット関節・指フィードバックとRGB-D由来のコネクタ／工具関係を使用。物体の評価真値は目標へ使わない。実行時は新しい画像から計画を作り直す。

| 候補 | 計算結果 | 検証範囲 |
| --- | --- | --- |
| 非反転・相対姿勢固定 | 10候補拒否 | 左腕の到達性・連続性など |
| 非反転・支持姿勢を独立 | 40候補拒否 | 到達性・連続性・腕同士など |
| 支持のつかむ面・反転を変更 | 8候補で把持〜搬送の一貫した経路を得られず | [仮定と全結果](evaluation/unflipped_grasp_20261016/README.md) |
| 反転・支持姿勢を独立 | 4候補目で幾何検査を通過 | 物理成功ではない |

[要求・全候補・計算コードのSHA](evaluation/support_transfer_20261016/manifest.json)を保存した。採用候補は高さ0.35 m、回転してから並進、支持orbit=0・rise=0の5工程。各工程で腕同士5 mm・設備1 mmの包絡余裕と指の全ストロークを検査。工具間距離の計画誤差は最大約0.092 mm、最小工具間距離120 mm以上の既存条件を維持した。ケーブル・他コネクタの接触や物理把持はこの計算では保証しない。

採用候補の再計算（MELFAメッシュの準備は [固定試験の再実行手順](reproduce-fixed-trial.md) に従う）：

```bash
docker compose run --rm dev python3 scripts/plan_held_wire_transfer.py \
  --world docs/evaluation/cooperative_nest_20261016/centred_unplanned_connector_contact/case/world.sdf \
  --models docs/evaluation/cooperative_nest_20261016/centred_unplanned_connector_contact/models \
  --feedback-request docs/evaluation/support_transfer_20261016/independent_flipped/request.json \
  --output /tmp/independent-support-plan.json
```

`--nest-orientation` は `either`／`unflipped`／`flipped` を指定でき、限定した向きで見つからなければ他の向きへ自動変更しない。既定の候補順は維持する。

## 次の物理比較

新しい初期状態から同じworld・モデルを読み、以下を指定する。

```text
--place-in-nest --feedback-paced-transfer --nest-orientation flipped
--after-nest-parking --top-regrasp --insert-after-regrasp --centred-grips
```

支持工具の姿勢を独立させるため、この比較では `--rigid-nest-transfer` を指定しない。フィードバック進行制御は、その時点の計画された両工具姿勢と実測を比較するため、変化する相対姿勢にも適用できる。位置1 mm・角度0.01 rad・関節0.01 rad、相対位置1.5 mm・相対角度0.015 radの進行条件は変えない。異なる工具姿勢を含む経路が正しく進行し、条件外で停止する回帰テストで確認する。

未把持部品と腕／共用治具の接触は即時失敗。把持の滑り5 mm・回転0.05 rad、最後の着座・解放・退避、最終12試行の条件も維持する。ゲイン・摩擦・力上限・ワイヤーの物理・形状を変更しない。計画段階の通過を物理成功に数えない。
