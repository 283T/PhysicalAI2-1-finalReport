# 正式評価シリーズ：固定P3の双腕取り出し・保持

`registry.json` を試行開始前に作成し、コミットして保存する。
候補コードf7812d5、承認済み仕様 `configs/submission_acceptance.json`。
設計101・配置2・P3、3試行、必要な一連成功は2試行以上。
すべて新規の初期worldから実行し、開発確認の成功は混ぜない。

## 実行

registryに各試行の完全なコマンド引数と新規出力先を保存した。
ROS環境をsourceしたguiコンテナ内で、r01→r02→r03の順に一つずつ実行する。
前試行の所有プロセスと終了報告を確認するまで次を起動しない。
実行中のソース・入力を変更しない。次の開始前にregistryの各SHAとDockerイメージIDを照合する。
修正が必要ならこのシリーズを保管し、別シリーズを実行前に登録する。

```bash
docker compose exec -T gui bash -lc 'source /opt/ros/jazzy/setup.bash && \
  python3 scripts/record_dual_hold.py \
    --world outputs/stepped_nest_contacts_hd_20261016/case/world.sdf \
    --models outputs/stepped_hd_shm64_20261016/models \
    --drawing-plan outputs/independent_support_p3_hd64_20261016/pickup/connection_plan.json \
    --inspection-calibration outputs/stepped_nest_contacts_hd_20261016/case/camera_calibration.json \
    --gui-config sim/harness_recording_gui.config --observation-timeout-wall-s 240 \
    --output outputs/pickup_hold_formal_20261028/r01'
```

## 変更と検証

物理・把持条件を変えず、`ArmSession.dwell` に開始・終了物理時刻、観測数、完了有無の記録を追加した。
ガード失敗はそのまま伝播し、部分区間はcompleted=falseで保存する。
4物理秒の保持を変えていないこと、途中失敗で成功を残さないことを含む全268テストが通過。

開発録画のPTSと単調時計に差があったため、新しい録画には位相名とレコーダー単調時計の経過を重ねる。
表示は1秒間隔の標本で、正確な物理時刻の代わりではない。動画の時間差は別途記録し、
差が大きい場合は位相表示と実フレームを確認してから切り出す。録画フィルタは物理を起動せず検査した。

## 終了後の判定

- `pickup/validation.json` の終了・成功と、右単独／双腕の持上げ・保持を確認する。
- `dwell_windows` のdual_holdがcompleted=true、4物理秒以上、異なる時刻10標本以上であることを確認する。
- 全把持ログを独立再計算し、保存した監視対象接触も検査する。
- registryのコード・入力・環境と一致することを確認してから、その試行を正式結果へ記録する。
- 動画の読取り・時間軸・内容を別に確認する。旧全接続Goalや挿入成功には数えない。

3試行の全結果と未実行を区別して示す。初回開始時点では0/3実行、成功数は未判定。
