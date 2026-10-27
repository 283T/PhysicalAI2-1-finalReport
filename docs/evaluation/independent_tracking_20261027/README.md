# 独立支持搬送の追従停止を切り分ける（10/27開始・10/28継続）

元試験は設計101・配置2・P3、開始コード67633a0、`nest_transfer_1` の時間予算終了で失敗。
元の判定、ゲイン、摩擦、工具・把持・着座の基準は変更しない。

## 保存ログの再解析

`sampled_gates.json` は0.1物理秒程度おきに保存した616行を、同じ行の指令と照合した結果。
計画の工具順序は左→右。相対位置誤差の計算は、この順序を維持する必要がある。
工具姿勢の誤差がある場合、逆順で計算した相対位置誤差のノルムは同じになるとは限らない。

- 左の工具位置・姿勢・関節、右の姿勢・関節は全616行で進行ゲート内。
- 右の工具位置は16行、左基準の相対工具位置は594行でゲート超過。
- 実制御側の記録は進行614／待機67,674標本、最終進行率9.71%。
- この疎な履歴は全pacer呼出しの再現ではなく、姿勢と関節の非同期性もあり得る。摩擦・力・根本原因は示さない。

再実行：

```bash
python3 scripts/analyze_feedback_gates.py \
  --pickup outputs/independent_support_p3_hd64_20261016/pickup \
  --output /tmp/sampled-gates.json
```

`grip_tracking.json` は別の既存解析器による終端把持診断。
そこでの工具順序は右→左のため、相対位置1.221 mmを左→右の進行ゲートと直接比較しない。

## 関節ごとの運動学的寄与

`relative_residual.json` は終端の指令・関節・ネイティブ工具姿勢からの一次近似。
左第2関節の寄与ベクトルの大きさが約0.629 mm、左第6関節が約0.510 mmだった。
工具自身の位置だけでなく、相対位置を表す左工具座標系の回転も寄与する。
各寄与はベクトルであり、大きさを足し合わせて全誤差とはしない。
中心差分との最大微分誤差9e-11 m/rad、一次近似誤差約5.53 µm、ネイティブとFKの相対位置差約0.418 µm。
これは力やトルクの測定ではなく、ゲイン・摩擦を原因と断定しない。

```bash
python3 docs/evaluation/independent_tracking_20261027/decompose.py \
  --world outputs/stepped_nest_contacts_hd_20261016/case/world.sdf \
  --models outputs/stepped_hd_shm64_20261016/models \
  --pickup outputs/independent_support_p3_hd64_20261016/pickup \
  --output /tmp/relative-residual.json
```

## 無負荷の単腕比較（実際のGazebo、10/28）

`unloaded_20261028/` に結果、実際の隔離world、ROS/Gazeboログを保存した。
元worldから各腕だけを取り出し、ハーネス・設備・他腕・描画センサーを除く。
物理刻み・重力・腕の質量・関節摩擦・PIDはコピー元のまま。初期worldから実際に腕を動かした。
目標は元試験の**ロボット指令**で、物体の保存姿勢を復元していない。

`check_reach.verify_arm` に明示的な検査値を指定する機能を追加し、既定値は維持した。
この診断だけは位置1 mm・角度0.01 rad・関節0.01 rad・FK差1 mm未満の5連続標本を要求する。
両腕の個別目標がこの基準内に入った。左は位置約0.999 mmで、余裕は小さい。
別world・別時刻の終端を合成した相対位置差は約0.753 mm、角度差は0.00748 rad。
**同時の双腕保持や同じ移動経路・積分器状態の比較ではない。**
ハーネス荷重だけが原因と断定せず、基板接続や搬送の成功にも数えない。

```bash
docker compose exec -T gui bash -lc 'source /opt/ros/jazzy/setup.bash && \
  python3 scripts/check_unloaded_tracking.py \
    --world outputs/stepped_nest_contacts_hd_20261016/case/world.sdf \
    --models outputs/stepped_hd_shm64_20261016/models \
    --pickup outputs/independent_support_p3_hd64_20261016/pickup \
    --output outputs/unloaded_tracking_new'
```

## 環境復旧・検証

10/27のDocker再構築後にMac容量不足が判明。初回イメージではXvfbが0バイトだった。
提出物と実験原本を維持し、提出物の再生成確認で作った重複コピー約235 MBだけを削除した。
その後、Macの空き61 GiBを確認してDockerを再起動し、キャッシュを使わず再構築した。
Xvfb起動とGazebo 8.15.0を確認。Docker内の全263テストは通過した。
Mac単体の全テスト実行はROS等の不足で失敗しており、通過扱いしない。

次は `scripts/record_dual_hold.py` で新しい初期状態から双腕把持・持上げ・保持だけを実行・録画する。
その成功は固定1接続や元Goalの完成とはしない。録画中は実行に使うソースを変更しない。
