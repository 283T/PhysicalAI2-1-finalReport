# 実測の開指・退避を要求する検証の負例

`preshape_negative.json` は、過去の終了済み失敗試験へ `release_witness.py` を実行した結果。元の試験は治具への下降を終えた後、開放前の待機時間切れで終了し、基板への挿入・開放・退避へ到達していない。

検証はsuccess=false、終了コード1となった。図面のP3目標が正しく与えられていても、対応する開放前・物理開指・退避・開放後の観測がないため通過しない。元の試験の失敗は変更していない。これは負例の動作確認であり、正常な実接続の証拠ではない。

```sh
docker compose run --rm dev nice -n 10 python3 scripts/release_witness.py --pickup docs/evaluation/cooperative_nest_20261016/preshape_nest_seat_wall_limit/pickup --case docs/evaluation/cooperative_nest_20261016/preshape_nest_seat_wall_limit/case --models docs/evaluation/cooperative_nest_20261016/preshape_nest_seat_wall_limit/models --output outputs/reproduce_release_negative.json
```

この負例では終了コード1が期待される。合成データの検証では、正常な測定、指令だけの開指・退避、片指の閉じ残り、着座ずれ、短い観測窓、退避観測の欠落を比較した。全体のテストは244件通過した。
