# 把持中心の指令モデル（物理試験ではない）

`ideal_model.json` は実測した閉指位置を入力とした理想PIの計算。物体・指が動かず、積分器初期値ゼロ、微分・飽和なしという仮定で150秒を計算した。指令中心の変化は約5.4e-20 m、理想出力の数値積分誤差は最大0.0006。実測の力、物理把持・接続成功や速度改善の証拠ではない。

再計算（既存結果を保護するため出力を別名にする）：

```sh
docker compose run --rm dev python3 docs/evaluation/centred_hold_20261016/reproduce.py --output /tmp/centred-model.json
```

入力ファイルのSHAは結果内に保存。新方式は `--centred-grips` 指定時だけ適用する。従来方式の失敗を上書きしない。[方式と制限](../../centred-grip.md)。
