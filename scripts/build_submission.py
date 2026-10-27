"""Build an honest submission from recorded prototype video and finished physical evidence.

Requires Pillow and ReportLab; ffmpeg and ffprobe must be on PATH.
Does not start a simulator, infer new AI results, or change trial verdicts.
"""

import argparse
import hashlib
import json
import subprocess
import zipfile
from pathlib import Path
from xml.sax.saxutils import escape

from PIL import Image, ImageDraw, ImageFont, ImageOps
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas
from reportlab.platypus import Paragraph

ROOT = Path(__file__).resolve().parents[1]
TRIAL = ROOT / "outputs/independent_support_p3_hd64_20261016/pickup"
CASE = ROOT / "outputs/stepped_nest_contacts_hd_20261016/case"
ARCHIVE = ROOT / "docs/evaluation/cooperative_nest_20261016/stepped_descent_tracking_failure"
FONT_DEFAULT = "/System/Library/Fonts/Supplemental/Arial Unicode.ttf"
CURRENT_CAMERA = TRIAL / "nest_held_observation/camera.png"
REPO = "https://github.com/283T/PhysicalAI2-1-finalReport"
NAVY, TEAL, MUTED, RED = "#142C40", "#087F8C", "#52677A", "#AB3C39"


def read(path):
    return json.loads(path.read_text())


def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def run(args):
    subprocess.run([str(a) for a in args], check=True)


def image_card(path, title, lines, font_path, photo=None):
    image = Image.new("RGB", (1920, 1080), "#F3F6F9")
    draw = ImageDraw.Draw(image)
    fonts = {size: ImageFont.truetype(str(font_path), size) for size in (24, 30, 36, 42, 54)}
    draw.rectangle((0, 0, 1920, 108), fill=NAVY)
    draw.text((64, 26), title, font=fonts[42], fill="white")
    x = 80
    if photo:
        with Image.open(photo) as source:
            source = ImageOps.contain(source.convert("RGB"), (1050, 780))
            image.paste(source, (40 + (1050 - source.width) // 2, 160))
        x = 1140
    y = 176
    for text, size, colour in lines:
        draw.text((x, y), text, font=fonts[size], fill=colour)
        y += size + 32
    draw.rectangle((0, 1000, 1920, 1080), fill=NAVY)
    draw.text((64, 1020), "PAI 最終課題 | 実録・保存画像・計測結果を区別して報告", font=fonts[30], fill="white")
    image.save(path)


def make_pdf(path, account, font_path, summary):
    pdfmetrics.registerFont(TTFont("SubmissionJP", str(font_path)))
    c = canvas.Canvas(str(path), pagesize=(595.28, 841.89))
    c.setTitle("図面に基づくハーネス接続工程の検証")
    c.setAuthor(account)
    c.setSubject("初期版の実演と双腕柔軟ハーネスの物理実験・未達点")
    body = ParagraphStyle("body", fontName="SubmissionJP", fontSize=10, leading=16,
                          textColor=colors.HexColor(NAVY), wordWrap="CJK")

    def paragraph(text, y, size=10, colour=NAVY):
        style = ParagraphStyle("p", parent=body, fontSize=size, leading=size * 1.65,
                               textColor=colors.HexColor(colour))
        p = Paragraph(escape(text).replace("\n", "<br/>"), style)
        _, height = p.wrap(491, 750)
        if y - height < 53:
            raise ValueError("PDF page overflow: " + text[:40])
        p.drawOn(c, 52, y - height)
        return y - height - 12

    def start(page, title, subtitle):
        c.setFillColor(colors.HexColor(NAVY))
        c.rect(0, 747, 596, 95, fill=1, stroke=0)
        c.setFillColor(colors.white)
        c.setFont("SubmissionJP", 20)
        c.drawString(52, 796, title)
        c.setFont("SubmissionJP", 9)
        c.drawString(52, 774, subtitle)
        c.setFillColor(colors.HexColor(MUTED))
        c.setFont("SubmissionJP", 8)
        c.drawString(52, 29, f"Physical AI 応用1 | {account} | 2026-10-27")
        c.drawRightString(543, 29, f"{page} / 4")
        return 720

    def heading(text, y):
        return paragraph(text, y, 13, TEAL)

    def photo(path, y, width=491, height=230):
        with Image.open(path) as im:
            ratio = min(width / im.width, height / im.height)
            w, h = im.width * ratio, im.height * ratio
        c.drawImage(str(path), 52 + (width - w) / 2, y - h, w, h)
        return y - h - 12

    y = start(1, "図面からハーネス接続へ", "初期版の実演と、双腕・柔軟ハーネスの検証")
    y = paragraph("到達点：図面A・Bによる簡略接続の実演と、現行双腕の把持・持上げ・保持。"
                  "現行方式の基板挿入と全コネクタ接続は未達。", y, 12)
    y = heading("目的とAIの役割", y)
    y = paragraph("電機メーカーの多品種少量生産を想定し、図面に応じてハーネスの接続先を変更する。"
                  "自作図面の画像をローカルOllama / Qwen3-VL 2Bで読み、端末・ソケット・座標・向きをJSONへ変換する。"
                  "その結果を基板定義と照合し、誤った指示は実行前に拒否する。外部の有料AI APIは使用しない。", y)
    y = heading("二つの実装段階を区別する", y)
    y = paragraph("初期版：ROS 2 Jazzy・Gazeboの直交型ロボット。図面AではP1→J1、図面BではP1→J2。"
                  "固定ジョイントによる理想把持と、変形しない視覚用ケーブルを使う。録画はこの初期版の実際の動作である。", y)
    y = paragraph("現行版：MELFA RV-4FRLを2台配置し、右手でP3、左手でワイヤーを摩擦接触により支持する。"
                  "ワイヤーは関節で連結した分岐リンクで近似する。隠れた把持固定や物体の瞬間移動は使わない。"
                  "今回示す現行版の画像は保存された静止画であり、動作録画ではない。", y)
    y = photo(CURRENT_CAMERA, y, height=198)
    paragraph("現行試験のRGB画像：持上げ後のハーネス。カメラの死角があり、画像だけで全接触・着座は証明できない。", y, 8, MUTED)
    c.showPage()

    y = start(2, "実験結果と証拠", "設計D101・基板B101・配置seed 2・対象P3")
    y = heading("図面読解と動作の切り分け", y)
    y = paragraph("最新試験では保存済みのローカルAI読解結果を検証して再利用した。今回の成果物作成時に新たな推論は行っていない。"
                  "D101の対応はP1→J2、P2→J1、P3→J3。任意の工場図面に対する汎化性能は未評価。", y)
    for title, value in [
        ("初期版A・B", "保存済み評価では2条件とも解放後の簡略着座を確認。固定ジョイント・剛体ケーブルという制約付き。"),
        ("右単独持上げ・保持", f"最小持上げ {summary['primary_min_lift_mm']:.3f} mm、3,567標本。報告上通過。"),
        ("双腕持上げ・保持", f"左 {summary['left_min_lift_mm']:.3f} mm / 右 {summary['right_min_lift_mm']:.3f} mm、各5,752標本。報告上通過。"),
        ("保持姿勢の画像照合", "新規RGB-Dとの位置差0.442 mm、姿勢差0.00734 rad。把持力や着座の証拠ではない。"),
        ("治具搬送・基板接続", "搬送区間nest_transfer_1で停止。治具着座、持替え、基板挿入に到達せず。全体判定は失敗。"),
    ]:
        y = heading(title, y)
        y = paragraph(value, y)
    y = heading("終了後の独立再計算", y)
    y = paragraph("把持ログ全件の再計算は記録と一致し、右126,037／左90,375標本で記録上の許容範囲内。"
                  "最大位置ずれは右0.580／左0.265 mm、最大回転ずれは右0.03718／左0.03787 rad。"
                  "標本間の未記録挙動や基板接続の成功を保証するものではない。", y)
    paragraph("指定ソケットの接触監査は、対象接触工程まで到達せず必要な記録区間がないため不合格。"
              "保存された接触集計には監査対象の腕・治具への違反は見つからなかったが、挿入成功とは扱わない。", y)
    c.showPage()

    y = start(3, "失敗・工夫・残る課題", "成功基準を維持し、部分成功と全体失敗を同時に記録")
    y = heading("最新試験の停止", y)
    y = paragraph("支持腕の姿勢を独立させた搬送を試したが、nest_transfer_1で収束前に物理時間予算61.884秒を使い切った。"
                  "同区間の実時間は約1,211秒、指令経路の進行は9.71%。最大工具追従位置誤差1.034 mm、"
                  "最大相対追従位置誤差1.542 mmに対し、進行ゲートはそれぞれ1.0 mm／1.5 mm。"
                  "許容値は変更せず全体失敗として保存した。", y)
    y = paragraph("これは追従停止の観測であり、摩擦、ゲイン、ケーブル拘束のどれが根本原因かは未確定。"
                  "過去の別試験では未把持端子と治具・指の接触も検出した。最新試験へ同じ原因を断定せず、条件比較が必要である。", y)
    y = heading("実装上の工夫", y)
    y = paragraph("図面・AI出力・画像SHAを対応付け、指令生成と正解照合を分離した。接触前には画像による再照合を行う。"
                  "双腕の経路を幾何検査し、実行中は工具追従、把持ずれ、対象外端子との接触を監視する。"
                  "途中成功だけで全工程を成功にせず、失敗報告と圧縮観測ログを残して再計算できるようにした。", y)
    y = heading("モデルと実用化の限界", y)
    y = paragraph("ワイヤーは離散リンクによる曲げ近似で、実材料の曲げ剛性・ねじり・摩擦を同定していない。"
                  "端末は大きめの試作形状と識別マーカーを使用する。ロボットモデルの慣性・指の力は実機校正していない。"
                  "電気的導通、ラッチ係合、実機安全性を検証した成果ではない。", y)
    y = heading("未達の範囲と次の実験", y)
    paragraph("固定P3の接続、全3個の順次接続、既接続端子の張力・保持、図面・基板変更後の物理接続、"
              "配置ランダム化は未達。事前定義の設計2種×配置3種×2回＝12試行は未実施。"
              "まず単体腕とハーネス保持時の追従を比較し、固定1接続の開放・退避後安定性を検証する。", y)
    c.showPage()

    y = start(4, "再実行と提出物", "OmniCampusアカウント名：" + account)
    y = heading("提出コードと環境", y)
    y = paragraph(REPO + "\n物理試験の開始コード：67633a0。ROS 2 Jazzy / Gazebo 8.15.0、"
                  "Ubuntu 24.04系Docker、ローカルOllama / qwen3-vl:2b-instruct。"
                  "モデル重み・公式ロボットメッシュは配布物に含めず準備手順を示す。", y)
    y = heading("再実行の入口", y)
    y = paragraph("初期版：docs/setup.md と docs/local-ai.md に従って準備し、"
                  "python3 scripts/run_from_drawing.py drawings/assembly_a.png を実行する。"
                  "Bは assembly_b.png を指定する。現行版の初期入力準備・物理試験は docs/reproduce-fixed-trial.md、"
                  "今回の搬送オプションは提出手順書を参照する。現行版は失敗を再現する開発試験である。", y)
    y = paragraph("今回の再確認ではDockerデーモンが停止しており、新規推論・新規物理実行は行っていない。"
                  "クリーン環境からの全工程再現も未確認。保存ログの把持・接触再計算と、提出動画・PDFの生成を実施した。", y)
    y = heading("動画の読み方", y)
    y = paragraph("120秒・音声なし。00:00-00:08は説明、00:08-01:04は初期版A/Bの実録抜粋（各等速、区間間のカットあり）、"
                  "01:04-01:22は現行版の保存画像、01:22-02:00は結果と課題。"
                  "初期版の接続映像を、現行双腕による柔軟ハーネスの接続成功として提示しない。", y)
    y = heading("参考・使用モデル", y)
    y = paragraph("Mitsubishi Electric melfa_ros2_driver（melfa_description、Apache-2.0）。"
                  "取得元：https://github.com/MitsubishiElectric/melfa_ros2_driver\n"
                  "モデル取得revisionと使用条件は THIRD_PARTY_NOTICES.md および licenses/ に記載。"
                  "本課題では公式の実機通信ドライバは使用せず、記述モデルをGazeboへ変換している。", y)
    paragraph("本資料・動画は開発到達点と失敗の報告であり、元の全接続Goalの完成報告ではない。"
              "YouTubeへの公開とOmniCampusへの提出は、成果物確認後に行う。", y)
    c.save()


def make_video(output, work, font_path, summary):
    cards = [
        ("intro", "図面からハーネス接続へ", [
            ("初期版の実演と、双腕・柔軟ハーネスの検証", 54, NAVY),
            ("ROS 2 Jazzy / Gazebo / ローカル Qwen3-VL 2B", 36, TEAL),
            ("初期版：図面A・Bで接続先を変更", 42, NAVY),
            ("現行版：双腕把持・持上げ・保持まで確認", 42, NAVY),
            ("現行版の基板接続は未達。完成デモではありません。", 36, RED),
            ("実録は等速。抜粋区間の間にカットがあります。", 30, MUTED),
        ], None, 8),
        ("current", "現行版｜保存した実験画像（動作動画ではありません）", [
            ("MELFA RV-4FRL × 2", 36, TEAL),
            ("右手：P3コネクタ", 36, NAVY),
            ("左手：ワイヤー支持", 36, NAVY),
            ("分岐リンクで柔軟材を近似", 30, NAVY),
            ("D101 / 配置 seed 2", 30, MUTED),
            ("AI読解：P3 → J3", 36, TEAL),
            ("基板への挿入は未達", 36, RED),
        ], CURRENT_CAMERA, 18),
        ("results", "現行版｜保存ログの計測・再計算", [
            (f"双腕持上げ・保持：左 {summary['left_min_lift_mm']:.3f} mm / 右 {summary['right_min_lift_mm']:.3f} mm", 42, NAVY),
            ("保持窓：各5,752標本。単位はmm、全体成功ではありません。", 30, MUTED),
            ("把持ログ再計算：右126,037 / 左90,375標本", 42, TEAL),
            ("最大位置ずれ：右0.580 mm / 左0.265 mm", 36, NAVY),
            ("搬送途中で時間切れ。全体判定は失敗。", 42, RED),
            ("ソケット接触工程の記録なし。接続成功とは判定しません。", 30, MUTED),
        ], None, 18),
        ("conclusion", "到達点と残る課題", [
            ("確認：図面変更による初期版の接続先変更", 42, TEAL),
            ("確認：現行双腕の接触把持・持上げ・保持", 42, TEAL),
            ("未達：現行版の基板挿入・全3コネクタ接続", 42, RED),
            ("未実施：複数基板・ランダム配置での最終12試行", 36, RED),
            ("次の工程：追従停止の切分け → 固定1接続の実証", 36, NAVY),
            ("失敗ログと再実行コードを保存し、未達点を報告します。", 30, MUTED),
        ], None, 20),
    ]
    common = ["-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
              "-pix_fmt", "yuv420p", "-r", "15", "-movflags", "+faststart"]
    clips = {}
    for key, title, lines, photo, duration in cards:
        image = work / (key + ".png")
        image_card(image, title, lines, font_path, photo)
        clip = work / (key + ".mp4")
        run(["ffmpeg", "-v", "error", "-y", "-loop", "1", "-framerate", "15", "-i", image,
             "-t", duration, *common, clip])
        clips[key] = clip
    for name, offset, label in [("a", 8, "図面A：P1 → J1"), ("b", 72, "図面B：P1 → J2")]:
        overlay = Image.new("RGBA", (1920, 1080), (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        draw.rectangle((0, 0, 1920, 100), fill=NAVY)
        draw.rectangle((0, 1000, 1920, 1080), fill=NAVY)
        font = ImageFont.truetype(str(font_path), 36)
        draw.text((40, 26), "初期版｜実際のGazebo録画・等速・抜粋　" + label, font=font, fill="white")
        font = ImageFont.truetype(str(font_path), 28)
        draw.text((40, 1022), "固定ジョイント把持／剛体ケーブル。現行双腕の接続成功ではありません。", font=font, fill="white")
        overlay_path = work / ("overlay_" + name + ".png")
        overlay.save(overlay_path)
        clip = work / (name + ".mp4")
        run(["ffmpeg", "-v", "error", "-y", "-ss", offset, "-i", ROOT / "recordings/harness_connection_demo.mp4",
             "-i", overlay_path, "-filter_complex", "[0:v]pad=1920:1080:0:100:color=white[bg];[bg][1:v]overlay=0:0[v]",
             "-map", "[v]", "-t", "28", *common, clip])
        clips[name] = clip
    concat = work / "concat.txt"
    concat.write_text("".join(f"file '{clips[k].name}'\n" for k in ["intro", "a", "b", "current", "results", "conclusion"]))
    run(["ffmpeg", "-v", "error", "-y", "-f", "concat", "-safe", "0", "-i", concat,
         "-c", "copy", "-map_metadata", "-1", "-movflags", "+faststart", output])


def make_code_bundle(path, evidence):
    """All program sources plus relevant reproduction inputs; omit historical bulk logs."""
    if (ROOT / ".git").exists():
        names = subprocess.check_output(["git", "ls-files", "-z"], cwd=ROOT).decode().split("\0")
    else:
        names = [str(p.relative_to(ROOT)) for folder in (
            "scripts", "tests", "sim", "configs", "drawings", "examples", "licenses", "web"
        ) for p in (ROOT / folder).rglob("*") if p.is_file() and "__pycache__" not in p.parts]
        names += [p.name for p in ROOT.iterdir() if p.is_file()]
    chosen = {ROOT / name for name in names if name and (
        name.split("/")[0] in {"scripts", "tests", "sim", "configs", "drawings", "examples", "licenses", "web"}
        or "/" not in name
    )}
    chosen.add(Path(__file__))
    chosen.discard(ROOT / "SUBMISSION_README.txt")
    chosen.update((ROOT / "docs").glob("*.md"))
    chosen.update(p for p in (ROOT / "docs/submission/20261027").rglob("*")
                  if "artifacts" not in p.parts)
    for folder in ("case", "models"):
        chosen.update((ARCHIVE / folder).rglob("*"))
    for name in ("connection_plan.json", "connection_plan.audit.json"):
        chosen.add(ARCHIVE / "pickup" / name)
    chosen.add(ARCHIVE / "manifest.json")
    chosen.add(ROOT / "docs/evaluation/video_validation.json")
    chosen.add(ROOT / "recordings/harness_connection_demo.mp4")
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for source in sorted(chosen):
            if source.is_file():
                z.write(source, str(source.relative_to(ROOT)))
        for source in evidence.rglob("*"):
            if source.is_file():
                z.write(source, "submission_evidence/" + str(source.relative_to(evidence)))
        z.writestr("SUBMISSION_README.txt", "Start at docs/submission/20261027/README.md.\n"
                   "All program sources are included. Historical regression datasets are not all bundled;\n"
                   "use the full GitHub repository for the historical test suite.\n"
                   "Current trial raw grip/trajectory logs are in submission_evidence/.\n"
                   "The physical connector insertion has not succeeded.\n")


def main():
    global TRIAL, CASE, CURRENT_CAMERA
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--account", required=True)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/submission_20261027")
    parser.add_argument("--font", type=Path, default=Path(FONT_DEFAULT))
    parser.add_argument("--source-evidence", type=Path,
                        help="Use the submission_evidence directory from the code ZIP")
    args = parser.parse_args()
    if not args.account or any(c in args.account for c in '/\\\n\r'):
        parser.error("Account name must be a filename-safe string")
    out = args.output.resolve()
    work, delivery, evidence = out / "work", out / "deliverables", out / "evidence"
    for path in (work, delivery, evidence / "pickup", evidence / "case"):
        path.mkdir(parents=True, exist_ok=True)
    if args.source_evidence:
        import shutil
        source_evidence = args.source_evidence.resolve()
        if source_evidence == evidence:
            parser.error("Source evidence must differ from the output evidence directory")
        TRIAL, CASE = source_evidence / "pickup", source_evidence / "case"
        CURRENT_CAMERA = source_evidence / "current_camera.png"
        for name in ("latest_grips_audit.json", "latest_contacts_audit.json"):
            shutil.copyfile(source_evidence / name, work / name)
    trial = read(TRIAL / "validation.json")
    if trial.get("trial_state") != "finished" or trial.get("success") is not False:
        raise ValueError("This report template requires the finished failed trial")
    grips, contacts = read(work / "latest_grips_audit.json"), read(work / "latest_contacts_audit.json")
    if not grips["evidence_consistent"] or not grips["all_recorded_grips_within_limits"]:
        raise ValueError("Grip audit differs from report template")
    if contacts["all_requested_target_contacts_recorded"] or contacts["evidence_consistent"]:
        raise ValueError("Contact result differs from report template; review wording")
    video = ROOT / "recordings/harness_connection_demo.mp4"
    if sha(video) != read(ROOT / "docs/evaluation/video_validation.json")["video_sha256"]:
        raise ValueError("Prototype footage differs from its saved evaluation")
    summary = {
        "created_date": "2026-10-27", "account": args.account,
        "repository": REPO, "physical_start_commit": "67633a0",
        "current_trial_success": False, "phase": trial["phase"], "error": trial["error"],
        "primary_min_lift_mm": trial["primary_hold"]["minimum_lift_m"] * 1000,
        "left_min_lift_mm": trial["dual_holds"]["left_arm"]["minimum_lift_m"] * 1000,
        "right_min_lift_mm": trial["dual_holds"]["right_arm"]["minimum_lift_m"] * 1000,
        "grip_audit": grips, "contact_audit": contacts,
        "video_scope": "Prototype actual footage at 1x, current dual-arm saved still, and result cards; no current insertion success",
        "video_segments": [{"start_s": a, "end_s": b, "kind": k} for a, b, k in [
            (0, 8, "title"), (8, 36, "prototype_A_actual_1x_source_8_to_36"),
            (36, 64, "prototype_B_actual_1x_source_72_to_100"),
            (64, 82, "current_trial_saved_still"), (82, 100, "measured_results"), (100, 120, "limitations")]],
        "new_physical_run_performed": False,
        "docker_check": "Docker daemon unavailable on 2026-10-27; no new simulation was run",
    }
    write(out / "summary.json", summary)
    import shutil
    for name in ["validation.json", "connection_plan.json", "connection_plan.audit.json", "trajectory.json.gz",
                 "grip_observations.jsonl.gz", "carried_contacts.jsonl.gz", "seat_observations.jsonl.gz"]:
        shutil.copyfile(TRIAL / name, evidence / "pickup" / name)
    for name in ("drawing.png", "board.json"):
        shutil.copyfile(CASE / name, evidence / "case" / name)
    checkpoint = TRIAL.parent / "control_sources_checkpoint.json"
    if checkpoint.exists():
        shutil.copyfile(checkpoint, evidence / checkpoint.name)
    shutil.copyfile(CURRENT_CAMERA, evidence / "current_camera.png")
    for name in ("latest_grips_audit.json", "latest_contacts_audit.json"):
        shutil.copyfile(work / name, evidence / name)
    write(evidence / "manifest.json", {str(p.relative_to(evidence)): sha(p) for p in sorted(evidence.rglob("*"))
                                      if p.is_file() and p.name != "manifest.json"})
    make_pdf(delivery / f"説明資料_PAI最終課題_{args.account}.pdf", args.account, args.font, summary)
    make_video(delivery / f"PAI最終課題_{args.account}.mp4", work, args.font, summary)
    make_code_bundle(delivery / f"コード_PAI最終課題_{args.account}.zip", evidence)
    write(delivery / "SHA256SUMS.json", {p.name: sha(p) for p in sorted(delivery.iterdir())
                                        if p.is_file() and p.name != "SHA256SUMS.json"})
    print(json.dumps({"delivery": str(delivery), "trial_success": False}, ensure_ascii=False))


if __name__ == "__main__":
    main()
