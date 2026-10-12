# 上面把持への持ち替え：治具・把持試験・下降経路

2026-10-11。横から把持した姿勢では基板への下降が包絡箱検査に通らなかったため、端子を一度治具へ置き、上面から把持し直す工程の開発を進めました。双腕による持ち替えの一連動作はまだ統合していません。

2026-10-11に確認したのは、受動治具による端子保持、固定支持台を使った上面把持・持ち上げ・姿勢保持、上面把持後を仮定したMELFA双腕の下降経路です。この段階では治具へ戻す動作は未成立でした。2026-10-12に、独立して昇降するケーブル支持台と把持時の実測関節位置への戻しを組み合わせ、直動試験台で開放・退避後の着座まで通過しました。MELFA双腕での持ち替えは未統合です。

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

| 固定支持台の試験（2026-10-11） | 実測値・判定 |
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

### 昇降支持台と実測位置への戻し（2026-10-12）

尾部の支持台を独立した垂直関節で昇降させ、把持後に端子の持ち上げと一緒に上げ、戻しに合わせて下げる試験を追加しました。台はケーブルへ接触するだけで、ハーネスを取り付けるジョイントや位置の書き換えはありません。把持前と開放・退避時の支持台指令は0です。主軸に加えて支持台の実測位置も到達判定に使います。

初期試作では把持前に台を上げる指令の誤りと、動く台が固定ケーブル受けに重なる問題がありました。前者を工程別の指令で修正し、後者は台の端をX=-75 mmにして固定受け（端X=-70 mm）との間に5 mmの隙間を設けました。支持台自身の接触センサーも追加しています。

把持時に着座している端子では、名目の関節指令と実際の関節位置に約1.2 mmの差がありました。`--return-reference closed` は、閉じた際に観測した関節位置を戻し目標に使います。その際、閉じた工程の最後の約2秒で着座が安定していることと、持ち上げを含め関節範囲に収まることを検査します。目標計算に用いるのは関節観測で、物体を目標へ移動させる操作ではありません。

| 最終コードでの再試験 `paired_verified` | 測定結果 |
|---|---:|
| 保持中の最小端子上昇 | 101.64 mm |
| 閉じた時からの位置関係の最大変化 | 0.0424 mm |
| 閉じた時からの最大回転 | 約0.000113° |
| 開放・退避後の最大横ずれ | 0.0686 mm |
| 着座高さ誤差 | 0.001 mm未満 |
| 着座姿勢誤差 | 0.001°未満 |
| 退避後の着座観測 | 約1.99秒・90サンプル、通過 |

同じ構成で初回 `paired_closed` と再試験が通過しました。固定された一ケースのシミュレーション値であり、MELFA双腕・ランダム配置・実機の精度の評価は含みません。微小な誤差は接触ソルバーの数値結果です。

昇降台と同じ形状の固定台を使った `static_matched_closed` は、把持・持ち上げ・保持は通過しましたが、戻しで端子が約11.56 mm浮き、約6.86°傾いて到達待ちがタイムアウトしました。従来形状の固定台でも実測位置への戻しだけでは失敗しています。今回の固定ケースでは、支持台を動かす構成で戻しまで成立したという比較結果です。各試験の指令は壁時計に従うため、シミュレーションの進行速度の差も含む記録です。

![昇降支持・同形状の固定支持・摩擦ゼロの比較](evaluation/moving_support_study/comparison.png)

摩擦ゼロではTCPが少なくとも101.15 mm上がる一方、端子の最小上昇は25.83 mmでした。端子が支持台から力を受けて動いても、指の開口は約16 mmで、幅20 mmの端子の把持になっておらず、位置関係の変化も約93.50 mmありました。保持は拒否し、`hold_rejected` で停止しました。負の比較の `expectation_met=true` を確認しています。

```bash
docker compose exec -T gui bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 scripts/grasp_benchmark.py --regrasp-nest --moving-support --return-reference closed --output outputs/moving_support_study/paired_verified'

docker compose exec -T gui bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 scripts/grasp_benchmark.py --regrasp-nest --moving-support --return-reference closed --friction 0 --expect-slip --output outputs/moving_support_study/zero_friction'

# 同じ支持台形状を固定した対照。戻し未成立で終了コード1。
docker compose exec -T gui bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 scripts/grasp_benchmark.py --regrasp-nest --matched-static-support --return-reference closed --output outputs/moving_support_study/static_matched_closed'
```

[協調支持試験の記録](evaluation/moving_support_study/summary.json)には、途中の失敗も含め、world・判定結果・支持台の指令／実測値を含む10 Hz CSV・SHA-256一覧を保存しています。これは二つの直動試験台による機械試験で、支持側アームによる把持を実行したものではありません。次はこの知見をMELFAの支持側制御へ移し、横把持から治具への配置・開放・上面把持を統合します。

### 戻し動作の診断（2026-10-11）

戻し動作を単に8秒で終了していた方式と、実測関節位置の到達を待つ方式を比較しました。各サンプルに指令・実測関節位置・誤差、端子の回転、全ケーブルリンクの位置を追加しています。端子のハウジングと2本のピンの接触センサーで、接触相手と最大侵入深さも記録します。初回の配信設定には不備があり、`timed` と `settled` の試験には接触データがありません。設定を修正した `contact_settled` と `pin_relief` では配信を確認しています。

| 条件 | 戻しの観測 | 判定 |
|---|---|---|
| 従来の時間指定 `timed` | 開放直前の関節高さ誤差7.69 mm。退避後の端子高さ誤差11.92 mm | 戻し失敗 |
| 到達待ち・従来治具 `contact_settled` | 30秒待っても関節高さ誤差1.67 mm。端子高さ誤差12.13 mm、傾き5.53°。ピンと端部支持材、ハウジングと端部ストッパーの接触が継続 | 開放せず停止 |
| 到達待ち・中央にピン逃げを設けた治具 `pin_relief` | 関節高さ誤差1.71 mm。端子高さ誤差8.48 mm、傾き18.42°。ピンとケーブル受け、ハウジングと端部ストッパーの接触が継続 | 開放せず停止 |

接触深さは実測の挿入力や張力ではありません。各条件は単発試験で、物理シミュレーションに対する壁時計の指令タイミングも異なるため、成功率や定量的な改善率の比較ではありません。待ち時間だけで解消しない接触と姿勢の問題が残ることを確認した記録です。ピン逃げ形状は `--pin-relief` で有効にする比較試験用で、工場シーンの標準治具には適用していません。

既定の戻しは、関節誤差1 mm未満の新しい観測を10回連続で受け取ってから開放します。30秒で到達できなければ失敗として終了し、次の開放を実行しません。開放・退避後の着座も、横ずれ2 mm未満、高さ誤差1 mm未満、回転誤差0.05 rad未満、位置変動1 mm未満を1秒以上の観測で満たす必要があります。これらは直動試験台の暫定判定で、実機の安全機能ではありません。

摩擦ゼロの旧シーケンスでは、端子を持ち上げられないまま空の指が閉じ、戻しで端子と接触して目標より約16 mm上で停止しました（`zero_friction`）。保持失敗時にその後の搬送・戻しを実行しないガードを追加しています。負の比較では、TCPが70 mm以上上昇しても端子を保持できず、`motion_aborted=hold_rejected` となることを確認します。正の試験では同じ保持失敗を全工程の失敗とします。

ガード追加後の `zero_guarded` は負の比較を通過しました。保持窓の最小TCP上昇は101.91 mm、端子の上昇は0.001 mm未満で、戻し・開放へ進まず `hold_rejected` で終了しています。`expectation_met=true` は保持できないという期待の通過であり、正の作業の成功ではありません。

```bash
docker compose exec -T gui bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 scripts/grasp_benchmark.py --regrasp-nest --support-tail --output outputs/return_study/contact_settled'

# 比較用のピン逃げ形状。現在は戻しが未成立なので終了コード1。
docker compose exec -T gui bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 scripts/grasp_benchmark.py --regrasp-nest --support-tail --pin-relief --output outputs/return_study/pin_relief'
```

[診断結果と各工程の観測](evaluation/regrasp_return_diagnosis/summary.json)、各条件のworld、validation、10 HzのCSV、SHA-256一覧を保存しています。途中で停止した試験の保持判定は、保存した軌跡からも再計算しています。

この診断を受けて、上記の2026-10-12の昇降支持台試験へ進みました。MELFAでは支持する端子や分岐部が異なるため、ケーブルの曲げ・荷重、把持接触の滑り、治具への接触を引き続き比較します。

全3端子の接続、直接のワイヤー支持、実測校正、張力・挿入力の制御、自己衝突・端子やケーブルと設備の衝突検査、完成動画は未完了です。支持台の初期指令、固定治具の隙間、形状を揃えた対照の検査を含む86件の単体テストが通過しています。
