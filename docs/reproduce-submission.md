# 提出対象の双腕取り出し・保持を再実行する

対象は設計101・配置2・P3の固定条件。右腕がコネクタ、左腕がワイヤーを把持して持上げ・保持する。
挿入や全端子接続の手順は使用しない。正式Bは事前登録した3試行で2成功、1失敗。
[登録・全結果](evaluation/pickup_hold_formal_20261028_b/README.md)と[提出仕様](../configs/submission_acceptance.json)を参照する。

## 環境

リポジトリのルートで作業する。制御候補は40496fb、結果資料を追加した後もregistry記載の制御ソースSHAを維持している。
Docker内はUbuntu 24.04、ROS 2 Jazzy、Gazebo 8.15.0。検証機はApple M3・16GB、Dockerの共有メモリ512MB。
別のPCでの速度・成功率は未測定。再ビルドしたイメージIDが登録IDと異なる場合は環境差として記録する。

```bash
docker compose --profile gui build
bash scripts/setup_melfa.sh
docker compose --profile gui up -d gui
docker compose exec -T gui python3 scripts/start_desktop.py --wait-only
```

MELFAのメッシュはsetup_melfa.shが固定した公式リビジョンから取得する。上流ライセンスは同梱の出典資料を参照する。
同じマシン上に実行中のGazebo・本プロジェクトの制御がないことを先に確認する。
終了していない試験を残したまま次を起動しない。記録のPID・ROS_DOMAIN_ID・GZ_PARTITIONと照合せずプロセスを停止しない。

## 公開アーカイブから固定入力を復元する

次のコードは候補ソースと全登録入力のSHAを照合する。既存入力は同一バイトだけを許容し、不一致なら上書きせず停止する。
保存済み初期world・URDF・図面・基板・校正・推論結果のみを復元し、終了時の物体や関節姿勢は復元しない。

```bash
python3 - <<'PY'
import hashlib
import json
import shutil
from pathlib import Path

base = Path('docs/evaluation/pickup_hold_formal_20261028_b')
registry = json.loads((base / 'registry.json').read_text())
source = Path('docs/evaluation/cooperative_nest_20261016/stepped_descent_tracking_failure')
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
for name, digest in registry['control_source_sha256'].items():
    assert sha(Path(name)) == digest, 'Control changed: ' + name
for name, digest in registry['input_sha256'].items():
    target = Path(name)
    if target.exists():
        assert sha(target) == digest, 'Existing input differs: ' + name
        continue
    assert name.startswith('outputs/'), 'Tracked input missing: ' + name
    matches = [p for p in source.rglob(target.name) if p.is_file() and sha(p) == digest]
    assert len(matches) == 1, 'No unique matching archived input: ' + name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(matches[0], target)
print('All registered control and inputs match')
PY
```

物理比較では監査済みのローカルQwen3-VL推論結果を再利用した。新規推論を行った試行として扱わない。
AIには図面画像のみを渡し、出力後に基板情報で検査する。今回P3の選択は固定仕様で、AIは接続対応を読む。
新規読解は別端末で `bash scripts/start_local_ai.sh` を起動し、プロジェクトの `.cache/ollama` にあるqwen3-vl:2b-instructを使用する。
初回のみモデルを用意する必要がある。別のモデル保存先を持つOllamaが11434で動いていないことを確認して次を実行する。誤答・auditも保存し、正解へ補正しない。
この新規推論結果を正式Bの入力へ上書きせず、別の確認として保管する。

```bash
python3 scripts/read_connection_plan.py \
  outputs/stepped_nest_contacts_hd_20261016/case/drawing.png \
  --board outputs/stepped_nest_contacts_hd_20261016/case/board.json \
  --output outputs/submission_fresh_ai/connection_plan.json
```

## 新しい初期状態から実行・録画する

次は出力先を新規作成する。既に存在する場合は別の名前を指定し、過去の失敗を消さない。
この追加再実行は正式Bの3試行へ後から加えない。

```bash
docker compose exec -T gui bash -lc 'source /opt/ros/jazzy/setup.bash && python3 scripts/record_dual_hold.py \
  --world outputs/stepped_nest_contacts_hd_20261016/case/world.sdf \
  --models outputs/stepped_hd_shm64_20261016/models \
  --drawing-plan outputs/independent_support_p3_hd64_20261016/pickup/connection_plan.json \
  --inspection-calibration outputs/stepped_nest_contacts_hd_20261016/case/camera_calibration.json \
  --gui-config sim/harness_recording_gui.config \
  --observation-timeout-wall-s 240 \
  --output outputs/submission_reproduction_r01'
```

GUIはlocalhost:6080。1試行に実時間数十分〜1時間以上かかる。保持の4秒は物理時計による。
recording.jsonとpickup/validation.jsonの両方がfinishedになるまで結果を確定しない。
Ctrl-Cによる中断も失敗として保持する。録画PTSと単調時計にずれがあるため、動画位置は実画像・字幕で照合する。
閉指・持上げ量や保持時間、全把持観測、監視対象接触を確認する。中間の成功フラグだけでは合格にしない。

## 保存済み正式結果を再監査する

各r01_result〜r03_result/manifest.jsonに原本SHAを保存した。encoding=gzipのNPYは展開すると原本と同一になる。
元からgzipのJSONLはencoding=unchangedで、そのまま読み込む。raw.mkvは容量のためローカル保管で、ハッシュを公開している。
動画を含む最終提出物は別途用意する。ログだけで映像の連続性を証明しない。

監査ツールはregistryの出力先を読むため、クリーンなcloneでは各manifestのarchive_pathから元ファイル名へ展開して
outputs/pickup_hold_formal_20261028_b/r01〜r03へ復元する。既存の実験原本を上書きしない。
PythonのNumPyを使える環境で次を実行する。r01は失敗として終了コード1、r02/r03はpassedとなる。

```bash
python3 docs/evaluation/pickup_hold_formal_20261028_b/audit_trial.py \
  --trial-id pickup_hold_d101_p2_r03 --output outputs/recheck_b_r03.json
```

この手順で入力8ファイルのアーカイブ一致と全登録SHA、シェル構文、固定候補による正式3試行の完了を確認した。
クリーンcloneからDocker再ビルドを含めた一括再実行は未検証。元の全接続・12試行の完了を示す手順ではない。
