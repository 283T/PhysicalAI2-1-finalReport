# 上面把持への持ち替え：治具・把持試験・下降経路

2026-10-11。横から把持した姿勢では基板への下降が包絡箱検査に通らなかったため、端子を一度治具へ置き、上面から把持し直す工程の開発を進めました。双腕による持ち替えの一連動作はまだ統合していません。

今回確認したのは、受動治具による端子保持、ケーブルの尾部を受け台で支えた直動試験台での上面把持・持ち上げ・姿勢保持、上面把持後を仮定したMELFA双腕の下降経路です。治具へ戻す動作は未成立で、成功扱いにしていません。

## 持ち替え治具

`regrasp_fixture.py`で、ハウジング両端の支持、端部の横ずれガイド、低いピン案内溝、ケーブル出口と根元のケーブル受けを生成します。中央の長軸方向±10 mmを把持用に空けています。ピン直径は4 mm、案内溝は8 mmです。ハウジングの横方向のすき間は1 mmです。

工場シーンでは治具中心を世界座標 `(0.08, -0.04)` m、受け面を高さ0.22 mに置きます。搬送パレットと基板の間の空間を使います。治具には固定ジョイントや端子の取り付け機構を使っていません。

一端子・12リンクの柔軟ケーブルを重力下で放し、3〜6秒の姿勢を観測しました。治具ありでは横ずれが最大約1.00 mm、着座高さの誤差は0.001 mm未満でした。治具なしでは約155 mm落下しました。ケーブルの高さ方向の広がりは治具ありで約185 mmあり、ケーブル全体を固定した試験ではありません。

```bash
docker compose exec -T gui bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 scripts/check_regrasp_nest.py --output outputs/regrasp_nest_test'
```

[治具あり・なしの検査結果](evaluation/regrasp_seed2/passive_validation.json)と、それぞれのSDF・観測履歴を保存しました。初期条件は治具の上に置いた端子であり、ロボットが治具へ置く工程を実行した試験ではありません。

## 上面把持の接触試験

既存の直動試験台を拡張し、ピン付き端子を治具からつかみ、100 mm持ち上げる指令で保持、治具への戻し、開放、指の退避を行います。グリッパーの指は接触面を二つに分け、長軸方向の別々の場所で接触させる試作版です。上面把持のTCPは端子中心から6 mm下で、ハウジング高さ20 mmのうち約16 mmを接触に使います。

保持判定には持ち上げ高さ、TCPとの位置関係、指の実際の隙間に加え、閉じた時からの端子の回転量0.05 rad未満を要求します。観測に回転行列がある場合、欠落・非有限・非剛体の行列も拒否します。回転を観測していない過去のデータでは `rotation_validated=false` と明記します。

ケーブル尾部を受け台で支える条件では、上面把持と姿勢保持が通過しました。受け台は固定された受動的な台であり、支持する右腕の代わりの限定的な機械試験です。支持なしでは、約100 mm持ち上がっても端子が数十度回転し、姿勢保持の判定を通りませんでした。

| 支持ありの最終試験 | 実測値・判定 |
|---|---:|
| 保持中の最小持ち上げ量 | 102.55 mm |
| 閉じた時からの位置関係の最大変化 | 0.0179 mm |
| 閉じた時からの最大回転 | 0.038° |
| 保持 | 通過 |
| 戻した後の着座高さの誤差 | 8.30 mm |
| 治具への戻し | 未成立 |

戻しの際に端子が傾いたため、持ち上げ成功だけを全工程の成功として扱っていません。

摩擦ゼロではTCPだけが上昇し、端子は治具に残りました。物体の取り付け、位置の書き換え、摩擦を無視した保持を使っていません。試験台の関節位置が収束したことを確認してから保持を判定し、過渡状態を保持窓に混ぜないようにしました。

```bash
# 支持ありの把持・戻し試験。現在、戻しが未成立の場合は終了コード1になります。
docker compose exec -T gui bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 scripts/grasp_benchmark.py --regrasp-nest --support-tail --output outputs/regrasp_top_test'

# 摩擦ゼロの負の比較。保持できないことを確認する試験です。
docker compose exec -T gui bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 scripts/grasp_benchmark.py --regrasp-nest --support-tail --friction 0 --expect-slip --output outputs/regrasp_top_zero'
```

保持と戻しの判定はレポートの `hold.success` と `reseat_success` に分かれます。`success` は保持・戻しを含む全試験の判定なので、保持だけ通っても戻しが失敗すれば偽です。摩擦ゼロの比較は `expectation_met` が判定です。

![測定結果](evaluation/regrasp_seed2/comparison.png)

[支持ありの結果](evaluation/regrasp_seed2/positive_validation.json)、[摩擦ゼロの結果](evaluation/regrasp_seed2/negative_validation.json)、10 Hzへ間引いたCSVを保存しています。支持なし、浅い案内溝、狭すぎる案内溝、ケーブル出口の干渉などの失敗は `failures/` に保存しました。

## 持ち替え後を仮定したMELFA下降経路

治具と分割接触パッドを含むMELFA RV-4FRL×2のシーンを生成しました。P1をJ2の上面から挿入する工具姿勢で、左右腕のIKを連続的に解き、両TCP間距離、左右腕の包絡箱、固定治具との干渉を評価しました。名目上の上面把持を開始条件とした下降経路が通過しています。

着座時はハウジング中心が受け側の縁から10 mm上、TCPはその6 mm下なので、指先と縁の名目上の距離は4 mmです。そのため固定治具との箱の余裕は各箱1 mmとし、双腕同士の余裕は各箱5 mmを維持しています。位置誤差や力・接触を含む実行の安全性を保証する値ではありません。

```bash
docker compose exec -T gui bash -lc \
  'source outputs/melfa_install/setup.bash && python3 scripts/build_factory.py --robot melfa_rv4frl --split-finger-contacts --output outputs/melfa_regrasp_worlds'

docker compose exec -T gui python3 scripts/build_case.py \
  --design-seed 101 --placement-seed 2 \
  --factory-world outputs/melfa_regrasp_worlds/factory_prototype.sdf \
  --regrasp-nest --output outputs/regrasp_factory_case

docker compose exec -T gui python3 scripts/regrasp_plan.py \
  --source docs/evaluation/melfa_dual_pick_seed2 \
  --transfer-report docs/evaluation/melfa_transfer_seed2/report.json \
  --target-world outputs/regrasp_factory_case/world.sdf \
  --target-models outputs/melfa_regrasp_worlds \
  --output outputs/top_regrasp_study.json
```

元の図面・AI接続計画・基板・モデルのハッシュを検証し、変更後のシーンでも基板と腕の順運動学が一致することを確認します。[下降の検査結果](evaluation/regrasp_seed2/top_descent.json)、治具付きworldとその生成manifest、治具設定を保存しています。元の把持状態から上面把持状態への関節角の飛びは実行しません。この計画には持ち替えが完了したという仮定があります。

一括シーン生成にも `--regrasp-nest` を追加しました。指定時は別の `outputs/melfa_regrasp_worlds` にモデルを生成し、従来のモデルを保持します。指定しない既存の双腕把持コマンドは従来のモデルを使います。このフラグは治具を加えるもので、持ち替えや挿入を自動実行するフラグではありません。

## 次の工程

治具へ戻す際には、持ち上げで変わったケーブルの形状が端子の位置・姿勢へ影響します。次は支持側の腕と一緒に戻し、ケーブルの余長と端子姿勢を監視して着座させる工程です。その後、横把持から治具への配置・開放・上面把持をMELFA双腕へ統合します。

全3端子の接続、直接のワイヤー支持、実測校正、張力・挿入力の制御、自己衝突・端子やケーブルと設備の衝突検査、完成動画は未完了です。今回は既存の接触把持試験の回帰と80件の単体テストが通過しました。
