"""Rebuild the supplementary failure video; never a grasp/insertion success demo."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def main():
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=root / "outputs/dual_hold_recording_20261028")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--font", default="/System/Library/Fonts/Supplemental/Arial Unicode.ttf")
    args = parser.parse_args()
    manifest = json.loads((Path(__file__).parent / "observation_timeout/manifest.json").read_text())
    raw = args.source / "raw.mkv"
    raw_sha = hashlib.sha256(raw.read_bytes()).hexdigest()
    if raw_sha != manifest["raw.mkv"]["original_sha256"]:
        parser.error("Footage differs from the archived failed trial")
    if args.output.exists():
        parser.error("Use a new directory; preserve earlier review videos")
    out = args.output
    out.mkdir(parents=True)

    def card(name, lines):
        image = Image.new("RGB", (1920, 1080), "#142c40")
        draw = ImageDraw.Draw(image)
        for i, text in enumerate(lines):
            draw.text((100, 140 + i * 120), text, fill="white",
                      font=ImageFont.truetype(args.font, 42 if i else 54))
        image.save(out / name)

    card("intro.png", ["現行MELFA双腕シーン：接近動作の実録",
         "設計101・配置2・P3。ローカルAI読解結果を検証して再利用。",
         "把持前の画像確認で停止した失敗試験です。",
         "画面録画を2倍速で抜粋。把持・挿入の成功映像ではありません。"])
    card("result.png", ["停止理由：安定画像が必要数に達しなかった",
         "有効2枚／必要3枚。90秒の待機で終了。",
         "画像は約30秒おき。認識精度や必要枚数は緩めません。",
         "実時間の待機予算を240秒へ延長し、再試験で検証します。"])
    overlay = Image.new("RGBA", (1920, 1080), (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    draw.rectangle((0, 0, 1920, 90), fill="#142c40")
    draw.rectangle((0, 990, 1920, 1080), fill="#142c40")
    draw.text((70, 22), "実際のGazebo画面｜接近区間の抜粋・画面録画を2倍速",
              font=ImageFont.truetype(args.font, 36), fill="white")
    draw.text((70, 1010), "この試験では把持・挿入に到達していません。失敗と修正点を記録しています。",
              font=ImageFont.truetype(args.font, 32), fill="white")
    overlay.save(out / "overlay.png")
    common = ["-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
              "-pix_fmt", "yuv420p", "-r", "15"]

    def run(command):
        subprocess.run(["ffmpeg", "-v", "error", "-y", *map(str, command)], check=True)

    for key, duration in [("intro", 8), ("result", 15)]:
        run(["-loop", "1", "-framerate", "15", "-i", out / (key + ".png"), "-t", duration,
             *common, out / (key + ".mp4")])
    run(["-ss", "552.6", "-t", "150.4", "-i", raw, "-i", out / "overlay.png",
         "-filter_complex", "[0:v]setpts=0.5*(PTS-STARTPTS),pad=1920:1080:320:90:color=0x142c40[bg];[bg][1:v]overlay=0:0[v]",
         "-map", "[v]", *common, out / "approach.mp4"])
    (out / "concat.txt").write_text("file 'intro.mp4'\nfile 'approach.mp4'\nfile 'result.mp4'\n")
    final = out / "現行MELFA_接近動作と認識待機失敗_2倍速.mp4"
    run(["-f", "concat", "-safe", "0", "-i", out / "concat.txt", "-c", "copy",
         "-movflags", "+faststart", final])
    print(final)


if __name__ == "__main__":
    main()
