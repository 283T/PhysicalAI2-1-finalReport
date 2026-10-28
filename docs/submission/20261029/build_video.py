"""Build an explicitly cut 2x excerpt of formal B r03; never synthesize robot motion."""
import hashlib
import json
import subprocess
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / 'outputs/submission_20261029'
WORK = OUT / 'video_work'
FONT = '/System/Library/Fonts/Supplemental/Arial Unicode.ttf'
SOURCE = ROOT / 'outputs/pickup_hold_formal_20261028_b/r03/raw.mkv'
CLIPS = [
    (1880, 1900, '01 右腕でP3コネクタを把持'),
    (2050, 2090, '02 右腕で持上げ開始'),
    (2160, 2200, '03 右腕の持上げ終盤（途中省略）'),
    (2630, 2670, '04 左腕を分岐ワイヤーへ接近'),
    (2810, 2850, '05 左腕でワイヤーを把持'),
    (2975, 3015, '06 双腕で持上げ開始'),
    (3070, 3110, '07 双腕の持上げ終盤（途中省略）'),
    (3280, 3300, '08 双腕保持区間の抜粋'),
]


def run(args):
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', *args], check=True)


def font(n):
    return ImageFont.truetype(FONT, n)


def card(path, title, lines, drawing=False):
    im = Image.new('RGB', (1280, 900), '#101e32')
    d = ImageDraw.Draw(im)
    d.text((50, 45), title, font=font(38), fill='#76daca')
    if drawing:
        pic = Image.open(ROOT / 'outputs/stepped_nest_contacts_hd_20261016/case/drawing.png').convert('RGB')
        pic.thumbnail((1160, 455)); im.paste(pic, ((1280-pic.width)//2, 120))
    y = 610 if drawing else 160
    for line in lines:
        d.text((50, y), line, font=font(27), fill='white'); y += 48
    d.text((50, 845), 'PAI最終課題 | 283t_123 | ROS 2 Jazzy・Gazebo・ローカルAI', font=font(22), fill='#b2c1d5')
    im.save(path)


def main():
    WORK.mkdir(parents=True, exist_ok=True); (OUT/'deliverables').mkdir(exist_ok=True)
    sha = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    assert sha == '43806adac2c650d261ee051ff63a7caa14af25439d1335e26e94abc36390e1bc'
    parts = []
    cards = [
      ('title', 10, '図面読解AI × 双腕ハーネス取り出し・保持', [
        '多品種少量生産の接続作業に先立つ、取り出し支援の試作',
        '図面D101・配置seed 2・P3固定。基板への挿入は未達。',
        '図面と基板は整合。今回は固定した1条件を物理検証。'], True),
      ('method', 8, 'AI・認識・柔軟物の役割', [
        'ローカルQwen3-VL 2B：画像からP1→J2、P2→J1、P3→J3を読解',
        'P3の選択は固定仕様。把持位置は新規RGB-Dで認識。',
        '右腕：端子コネクタ　左腕：分岐ワイヤーを支持',
        '柔軟ワイヤーはリンク・曲げ関節で近似。実材料は未同定。',
        '次は正式B r03の実録。実時間に対して2倍速、区間を省略。',
        '録画PTS 22.6→921.0秒に欠落。以下の抜粋は欠落後。',
        '物理時計は実時間より低速。4物理秒は保存ログで検証。'], False),
    ]
    for name, duration, title, lines, drawing in cards:
        png=WORK/(name+'.png'); card(png,title,lines,drawing)
        part=WORK/(name+'.mp4'); run(['-loop','1','-i',str(png),'-t',str(duration),'-r','10','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p',str(part)]); parts.append(part)
    for i,(start,end,label) in enumerate(CLIPS):
        overlay=Image.new('RGBA',(1280,900),(0,0,0,0)); d=ImageDraw.Draw(overlay)
        d.rectangle((0,0,1280,75),fill=(12,25,43,245)); d.text((24,12),label,font=font(32),fill='white')
        d.rectangle((0,777,1280,858),fill=(12,25,43,245))
        d.text((24,782),'実録2倍速 | 区間の前後・途中を省略 | 正式B r03',font=font(25),fill='#76daca')
        d.text((24,817),f'原録画PTS {start}–{end}秒 | 保持4物理秒の成立は全ログで独立確認',font=font(22),fill='white')
        png=WORK/f'overlay_{i}.png'; overlay.save(png); part=WORK/f'clip_{i}.mp4'
        run(['-ss',str(start),'-i',str(SOURCE),'-loop','1','-i',str(png),'-filter_complex',f'[0:v]trim=duration={end-start},setpts=(PTS-STARTPTS)/2,fps=10[v];[v][1:v]overlay=0:0:shortest=1[out]', '-map','[out]','-t',str((end-start)/2),'-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p',str(part)]); parts.append(part)
    png=WORK/'results.png'; card(png,'固定条件での結果と、残る課題',[
      '事前登録した正式B：2成功／3試行（66.7%）。失敗も保存。',
      'r01：左閉指前のLost feedbackで停止。r02・r03：一連動作成功。',
      'r03双腕保持：4.0004物理秒、最小上昇 左99.035／右99.233 mm',
      '全把持ログを再計算。監視対象の接触違反なし。',
      '別シリーズAは録画失敗で中断。開発試行を成功数へ混ぜない。',
      '未達：挿入・全端子接続・ランダム配置・複数基板の物理検証。',
      '実機・導通・材料特性・未計測接触は評価していない。'])
    part=WORK/'results.mp4'; run(['-loop','1','-i',str(png),'-t','12','-r','10','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p',str(part)]); parts.append(part)
    concat=WORK/'concat.txt'; concat.write_text(''.join("file '"+str(p)+"'\n" for p in parts))
    output=OUT/'deliverables/PAI最終課題_283t_123.mp4'; run(['-f','concat','-safe','0','-i',str(concat),'-c','copy','-movflags','+faststart',str(output)])
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_format','-show_streams','-of','json',str(output)],text=True))
    assert 60 <= float(probe['format']['duration']) <=180
    run(['-i',str(output),'-f','null','-'])
    meta={'scope':'Formal B r03 actual recording excerpts; 2x wall capture playback; explicit cuts; no insertion claim','source_sha256':sha,'clips':[{'start_pts_s':a,'end_pts_s':b,'label':c,'speed':2} for a,b,c in CLIPS],'source_packet_gap':{'previous_pts_s':22.6,'next_pts_s':921.0,'delta_s':898.4},'duration_s':float(probe['format']['duration']),'full_decode_passed':True,'video_sha256':hashlib.sha256(output.read_bytes()).hexdigest(),'video_bytes':output.stat().st_size,'visual_qa':'pending review of output frames and all clip boundaries'}
    (OUT/'video_validation.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n'); print(json.dumps(meta,ensure_ascii=False))

if __name__=='__main__': main()
