# 最終下降の位置追従残差の切り分け

58060の終了時のロボット関節・工具フィードバックと、同じURDF・worldから計算した。物体姿勢は使わず、操作も行わない。`result.json` に入力SHAと計算結果を保存する。

右工具の残差は約(−0.108, −0.426, +0.905) mm。ヤコビアンの一次近似で高さ方向への寄与は第2関節+0.585 mm、第3関節+0.137 mm、第5関節+0.180 mm。第6関節はこの工具位置にほぼ寄与しない。したがって、第6関節の角度誤差だけを理由に位置追従の調整対象とはしない。これは幾何的な切り分けであり、トルク飽和・重力・摩擦・把持の偏りのどれが原因かは未確定。

[Gazebo 8.15.0のJointPositionController](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.15.0/src/systems/joint_position_controller/JointPositionController.cc) の位置指令受信は目標値を更新し、PIDを初期化しない。力モードでは各PreUpdateで位置差からPID出力を計算する。[gz-math7のPID](https://github.com/gazebosim/gz-math/blob/gz-math7/src/PID.cc) は積分項と最終出力をそれぞれ制限する。これらは実装の確認であり、今回の積分器状態を測定したものではない。

実行中の37583は変更しない。把持中心保持の比較後も追従不足が残った場合は、この結果を使って調整対象を絞り、別の物理試験で検証する。追従の進行条件・最終着座条件は緩めない。

再計算：

```sh
docker compose run --rm dev python3 docs/evaluation/tracking_residual_20261016/reproduce.py --output /tmp/tracking-residual.json
```
