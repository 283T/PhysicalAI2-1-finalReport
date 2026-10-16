# 把持中心の指令モデル（物理試験ではない）

`ideal_model.json` は実測した閉指位置を入力とした理想PIの計算。物体・指が動かず、積分器初期値ゼロ、微分・飽和なしという仮定で150秒を計算した。指令中心の変化は約5.4e-20 m、理想出力の数値積分誤差は最大0.0006。実測の力、物理把持・接続成功や速度改善の証拠ではない。

再計算（既存結果を保護するため出力を別名にする）：

```sh
docker compose run --rm dev python3 docs/evaluation/centred_hold_20261016/reproduce.py --output /tmp/centred-model.json
```

入力ファイルのSHAは結果内に保存。新方式は `--centred-grips` 指定時だけ適用する。従来方式の失敗を上書きしない。[方式と制限](../../centred-grip.md)。

## 37583の物理途中証拠

`primary_hold_checkpoint.json` は進行中の試験から複写した途中報告。右単独の100 mm上昇・4秒保持は通過し、保持最小上昇99.851 mm、保持3,273標本。後続の左支持事前開口に進んだ途中報告時点で、閉指からの把持監視11,279標本、最大ずれ0.0808 mm／0.000963 rad。`primary_hold_snapshot.json` も左支持事前開口中の把持観測。`physical_control_sources.json` は実行コード989ffdbと全scriptsのSHA、`physical_session.json` は同じGazebo／制御プロセスの識別を保存する。

**右単独の途中成功のみ。** 双腕保持、治具搬送・着座、基板挿入はこの証拠では未検証。試験終了・最終成功とは扱わず、従来試験との搬送性能比較も未確定。`primary_hold_provenance.json` に複写元のSHAと変更していない公開worldの参照を保存した。元の把持参照を変更せず、指の力上限・物理条件・評価基準も維持している。
