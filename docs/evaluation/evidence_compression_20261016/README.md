# 証拠ログの圧縮負荷だけの比較

治具下降後の待機時間切れについて、ログ圧縮のCPU負荷を切り分けた。完了済み失敗試験の先頭5,000観測、7,375,464バイトの同一JSONLを、gzipレベル1・6・9で各3回書いた。実行順は交代させ、全て展開後のバイト列が同一であることを検査した。観測の間引きや数値の丸めは行っていない。

| gzipレベル | CPU時間中央値 | 圧縮後バイト数 |
|---|---:|---:|
| 1 | 0.05045秒 | 2,120,128 |
| 6 | 0.20597秒 | 1,776,486 |
| 9（現行） | 0.37722秒 | 1,763,399 |

これはDocker ARM64・Python 3.12の圧縮処理だけの測定で、物理シミュレーションのRTFや全工程の高速化を示さない。JSONの生成、把持判定、ROS/Gazeboの配信・物理演算、ディスクへの書込みは切り分けている。先頭部分だけなので搬送全体の圧縮率の代表値とも扱わない。低速化の主因と断定できず、この結果だけでは実行中・次回の圧縮設定を変更しない。

再実行は新しい出力名を指定する。

```sh
docker compose run --rm dev nice -n 10 python3 docs/evaluation/evidence_compression_20261016/reproduce.py --pickup docs/evaluation/cooperative_nest_20261016/preshape_nest_seat_wall_limit/pickup --output outputs/reproduce_evidence_compression.json
```

元の分割gzipの全SHAと、測定したJSONLのSHA・全測定値は `validation.json` に保存した。物理把持・着座・接続の成功証拠ではない。
