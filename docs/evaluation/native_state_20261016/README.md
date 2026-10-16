# 回転搬送中のGazebo状態の読み取り

37583の制御とGazeboを動かしたまま、公開されている `/world/multi_connector_factory/state` をEmpty要求で1回読んだ。応答型は `gz.msgs.SerializedStepMap`。物理時刻109.4765秒。物体姿勢や関節へ指令せず、プラグイン・物理・制御設定も変更していない。

`native_state.pbtxt.gz` は応答原本を可逆圧縮したもの。`decoded.json` は12個のアーム関節と4個の指関節の位置・速度を復号する。成分型IDは同じインストールの `gz::common::hash64`（FNV-1a 64 bit）と登録文字列から計算する。JointPositionとJointVelocityはVectorDoubleSerializer、JointForceCmdはその専用serializerを指定していなかった。

**全16関節でJointForceCmd成分は存在するが、シリアライズされた値が空だった。** これを0 N／0 Nmと解釈せず、力・トルク・指令飽和は未検証とする。このAPIから原因を断定しない。内部PID積分器の状態も取得できていない。読み取り自身の計算負荷もあるため、性能比較として扱わない。

この標本では両腕の第1〜3関節の速度の絶対値は1e-11 rad/s未満だった一方、一部の手首関節は動いていた。同じ保存worldのアーム関節には摩擦1・減衰1が設定されている。静止摩擦による微小誤差の停滞は原因候補だが、単一標本だけでは確定できない。摩擦・力上限・成功基準は変えず、必要なら別の隔離試験でPI制御の収束を比較する。進行中の37583へゲイン変更は加えない。

復号の再実行：

```sh
docker compose run --rm dev python3 docs/evaluation/native_state_20261016/reproduce.py \
  --state docs/evaluation/native_state_20261016/native_state.pbtxt.gz \
  --output /tmp/native-state-decoded.json
```

元の取得コマンド（同じworldが既に動いている場合の読み取りのみ）：

```sh
gz service -s /world/multi_connector_factory/state \
  --reqtype gz.msgs.Empty --reptype gz.msgs.SerializedStepMap \
  --timeout 5000 --req ''
```

この状態データは独立診断専用であり、搬送目標や接続成功の証拠へ流用しない。
