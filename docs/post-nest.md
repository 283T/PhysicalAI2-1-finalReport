# 治具着座後の開放・退避と監視

この工程は実装・単体テスト・名目姿勢での事前検査まで。実際の治具着座、支持の開放、退避、上面持ち替え、基板接続の成功証拠ではない。現在の物理試験と次の工程は [Goal記録](goal-status.md) を参照。

`dual_pickup.py --place-in-nest --after-nest-parking` は、右の物理開放・退避後の治具着座が実測で通過した場合だけ、次を実行する。

1. 治具上のP3へ独立した姿勢監視を登録する。
2. 左のワイヤー把持の監視終了時刻を保存し、左指を物理的に開く。3秒待機し、2秒以上の着座安定を検査する。
3. 新しいロボット状態から左腕の退避経路を計画し、物理実行して着座を再確認する。
4. さらに新しい状態から右腕の退避を計画・実行し、着座を再確認する。

`park_arm_plan.py` は物体真値を含む入力や、指が開いていない状態を拒否する。両腕の指は全ストロークを囲み、腕5 mm／設備1 mmの余裕を維持する。親プロセスは計画子プロセスの待機中も監視を続け、異常・時間切れなら子も停止する。実行前にrequest・world・URDFのSHAを検査する。

`seat_guard.py` は凍結済みの `configs/final_acceptance.json` の数値を読み、基準のSHAを保存する。各観測の位置・高さ・向き・最初の位置からのずれを検査し、一度の失敗を保持する。参照や分類の記録を編集しても実行中の基準は変わらず、ログ書込みに失敗しても検出済みの外れは消えない。非有限値は失敗とし、JSONにはnullを残す。

監視は二種類に分ける。`passive_nest` は新しい把持監視を登録して新規観測を評価した後に限り、意図的な持ち上げのため終了できる。`connected` はこの操作では終了できず、次の端子の作業中も残す。個々の観測が通っただけで接続完了とはしない。開放・退避後2秒以上、10標本以上の窓評価も別に必要。

座標保持の観測は `seat_observations.jsonl.gz`、把持は `grip_observations.jsonl.gz` へ分けて保存する。物体真値は独立評価にだけ使い、退避計画にはロボットの関節・指状態だけを渡す。

名目上の治具配置・右退避後を仮定した [退避の事前検査](evaluation/stepped_regrasp_20261016/parking_preflight.json) は、左・右の順で通過した。以前の異なるオフライン計画の関節値と、両指が開いているという仮定を組み合わせた検査であり、現在の実測状態の代用にはしない。実行時は毎回実測から再計画する。

```sh
docker compose run --rm dev python3 docs/evaluation/stepped_regrasp_20261016/parking_preflight.py --output outputs/reproduce_parking_preflight.json
```

この開発用CLIの `success` は要求した部分工程の成否。`final_acceptance_eligible` はfalseのままで、基板挿入・全3接続・最終12試行の成功回数には含めない。後段の例外は先行工程の成功にかかわらず全体を失敗にする回帰テストも追加した。

## 保存した着座観測の再計算

`replay_seat_evidence.py` は終了済み試験の `seat_observations.jsonl.gz` を読み、凍結した評価基準から各観測と集計を再計算する。大きなログの分割形式にも対応する。

```sh
docker compose run --rm dev python3 scripts/replay_seat_evidence.py --pickup outputs/finished_trial/pickup --output outputs/finished_trial/seat_replay.json
```

基準のSHA・内容、最初の参照時刻、単調な時刻、観測数・最大誤差、最初の失敗を照合する。`P3_connected` は `socket_after_release` の窓を別に検査し、2秒以上・10標本以上が必要。1観測の判定やreportのsuccessだけでは通過しない。治具監視の意図的終了には、最後の着座観測と、新しい把持参照・観測が必要。接続済み監視の終了は拒否する。

Python APIの `required_windows` で後の全端子用の窓を指定でき、`expected_seats` で別途図面から導いた目標・種類を照合できる。目標の計算表現差は1e-9以下で照合し、物理的な着座基準は変えない。デフォルトの目標は保存参照であり、これ単独では図面との一致・物理開放・退避・ネイティブ接触・記録間の連続性を証明しない。それらは全体監査で別に確認する。現在は実行時guardが生成する合成テストで検証済みで、実際の接続成功証拠はまだない。
