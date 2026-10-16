# 現在の固定P3開発試験を再実行する

**開発途中の試験を再現する手順。完成した接続デモではない。** 現在は右単独・双腕の100 mm上昇と4秒保持まで通過し、治具搬送以降を検証中。全3端末・最終12試行の入口ではない。進捗は [Goal記録](goal-status.md)、保持方式は [把持中心の制御](centred-grip.md) を参照する。

Dockerの `/workspace` マウントを使う。保存worldが参照するMELFAメッシュもこの配置へインストールする。[Docker準備](setup.md)後、リポジトリのルートで実行する。

```bash
docker compose --profile gui build
bash scripts/setup_melfa.sh
```

## 入力を保存済みの初期シーンから用意する

失敗試験のアーカイブから、初期world・URDF・図面・基板・校正と監査付きAI図面計画だけを複写する。軌跡や終了時姿勢は読み込まない。コピー前に全入力のSHAを公開manifestと照合し、既存の出力先を拒否する。シーンは新しくGazeboへ読み込み、初期条件から実行する。

```bash
python3 - <<'PY'
import hashlib
import json
import shutil
from pathlib import Path

source = Path('docs/evaluation/cooperative_nest_20261016/stepped_descent_tracking_failure')
output = Path('outputs/reproduce_p3')
if output.exists():
    raise RuntimeError('Use a new output directory; preserve previous experiments')
manifest = json.loads((source / 'manifest.json').read_text())
files = [p for folder in ('case', 'models') for p in (source / folder).rglob('*') if p.is_file()]
files += [source / 'pickup' / name for name in ('connection_plan.json', 'connection_plan.audit.json')]
for path in files:
    if hashlib.sha256(path.read_bytes()).hexdigest() != manifest[str(path.relative_to(source))]:
        raise RuntimeError('Archived input changed: ' + str(path))
for folder in ('case', 'models'):
    shutil.copytree(source / folder, output / folder)
for name in ('connection_plan.json', 'connection_plan.audit.json'):
    shutil.copyfile(source / 'pickup' / name, output / name)
print(output)
PY
```

これは同じ図面に対する既存のローカルQwen3-VL推論結果を使う比較条件。図面画像のSHAとAI監査は制御開始前にも検査する。新しい図面のAI読解は [複数端末の手順](multi-connector.md) に従う。この手順で新規推論を行ったとは記録しない。

## 物理シーンと制御を起動する

他の物理試験が動いていない環境で実行する。同じGazebo world名とROS topicを使うため、進行中の試験と同時に起動しない。進行中プロセスの終了確認なしに再起動しない。

端末A：

```bash
docker compose --profile gui up -d gui
docker compose exec -T gui python3 scripts/start_desktop.py --wait-only
docker compose exec -T gui bash -lc 'source /opt/ros/jazzy/setup.bash && gz sim -s -r outputs/reproduce_p3/case/world.sdf' > outputs/reproduce_p3.scene.log 2>&1
```

端末B：

```bash
docker compose exec -T \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/configs/fastdds_rgbd_hd64.xml \
  gui bash -lc 'source /opt/ros/jazzy/setup.bash && python3 scripts/dual_pickup.py \
    --world outputs/reproduce_p3/case/world.sdf \
    --models outputs/reproduce_p3/models \
    --drawing-plan outputs/reproduce_p3/connection_plan.json \
    --output outputs/reproduce_p3/pickup \
    --inspection-calibration outputs/reproduce_p3/case/camera_calibration.json \
    --nest-calibration configs/nest_camera_hd.json \
    --wire-distance-mm 140 --wire-tip-depth-mm 19 \
    --place-in-nest --rigid-nest-transfer --feedback-paced-transfer \
    --after-nest-parking --top-regrasp --insert-after-regrasp --centred-grips' \
    > outputs/reproduce_p3.pickup.log 2>&1
```

`progress.json` は途中段階、`validation.json` は終了時の結果。途中の持ち上げ成功を接続成功へ読み替えない。各前段が実際に成功した場合だけ後段へ進み、失敗すると停止する。`final_acceptance_eligible` はfalseのまま。物理計算は実時間より遅く、観測待ちだけを理由に別の試験を開始しない。

## この手順の検証範囲

入力20ファイルの複写・SHA照合、出力先が存在する場合の拒否、全シェルブロックの構文、コピーしたworldのSDF構文、参照する7種のメッシュの存在を検査した。[検査記録](evaluation/reproduce_fixed_20261016.json)。実行中の37583は元の同一world・URDF・AI計画から上記と同じ制御オプションで起動した試験。コピー先からの全工程再実行は、進行中の試験を終えた後に検証する。完全な再現性や最終完成を先に宣言しない。
