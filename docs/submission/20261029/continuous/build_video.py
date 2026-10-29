"""Render one uninterrupted recorded motion interval at 16x; preserve submitted 2x edit."""
import hashlib
import json
import subprocess
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[4]
BASE = ROOT/'docs/submission/20261029/continuous'
OUT = ROOT/'outputs/continuous_video_20261029'
SOURCE = ROOT/'outputs/pickup_hold_formal_20261028_b/r03/raw.mkv'
FONT = '/System/Library/Fonts/Supplemental/Arial Unicode.ttf'
START, END, SPEED = 921.0, 3321.6, 16
SOURCE_SHA = '43806adac2c650d261ee051ff63a7caa14af25439d1335e26e94abc36390e1bc'

def run(args):
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y',*args],check=True)

def font(size):return ImageFont.truetype(FONT,size)

def card(path,title,lines):
    im=Image.new('RGB',(1280,900),'#101e32');d=ImageDraw.Draw(im)
    d.text((45,70),title,font=font(40),fill='#76daca')
    for i,line in enumerate(lines):d.text((45,190+i*60),line,font=font(28),fill='white')
    d.text((45,835),'283t_123 | 改善確認用・提出済み動画とは別版',font=font(24),fill='#b2c1d5');im.save(path)

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest()==SOURCE_SHA
    packet=json.loads((SOURCE.parent/'video_packet_timing.json').read_text())
    for gap in packet['gaps_over_1s']:
        assert not (START < gap['next_pts_s'] and END > gap['previous_pts_s'])
    intro=OUT/'intro.png';card(intro,'双腕ハーネス取り出し・保持：連続版',[
      '右腕の接近開始前から、双腕保持の終了までを連続表示。',
      '動作中の区間カットなし。実時間記録に対して16倍速。',
      '正式B r03の実録。シミュレーションの物理時計は低速。',
      '動作前の原録画に欠落あり。この連続区間には欠落なし。',
      '課題の「2倍速再生可」を超えるため、改善確認用の別版。'])
    end=OUT/'end.png';card(end,'固定条件で双腕保持まで完了',[
      '正式B：2成功／3試行。失敗も保存して報告。',
      'r03：双腕保持4.0004物理秒（全ログで独立検証）。',
      '基板への挿入・全端子接続・ランダム配置は未達。',
      'この16倍速版では、提出済み動画を差し替えていない。'])
    for stem,duration in [('intro',10),('end',8)]:
        run(['-loop','1','-i',str(OUT/f'{stem}.png'),'-t',str(duration),'-r','30','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p',str(OUT/f'{stem}.mp4')])
    overlay=Image.new('RGBA',(1280,900),(0,0,0,0));d=ImageDraw.Draw(overlay)
    d.rectangle((0,0,1280,75),fill=(12,25,43,245));d.text((24,15),'双腕の動作を連続表示 | 実録16倍速 | 区間カットなし',font=font(29),fill='white')
    d.rectangle((0,782,1280,858),fill=(12,25,43,245))
    d.text((24,787),'接近 → 右把持・持上げ → 左ワイヤー支持 → 双腕持上げ・保持',font=font(25),fill='#76daca')
    d.text((24,822),'正式B r03 | 元録画PTS 921.0–3321.6秒 | 改善確認用・提出版は変更なし',font=font(21),fill='white')
    overlay.save(OUT/'overlay.png')
    run(['-ss',str(START),'-i',str(SOURCE),'-loop','1','-i',str(OUT/'overlay.png'),'-filter_complex',f'[0:v]trim=duration={END-START},setpts=(PTS-STARTPTS)/{SPEED},fps=30[v];[v][1:v]overlay=0:0:shortest=1[out]','-map','[out]','-t',str((END-START)/SPEED),'-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p',str(OUT/'motion.mp4')])
    concat=OUT/'concat.txt';concat.write_text(''.join("file '"+str(OUT/(s+'.mp4'))+"'\n" for s in ['intro','motion','end']))
    output=OUT/'双腕ハーネス_連続動作_16倍速_283t_123.mp4'
    run(['-f','concat','-safe','0','-i',str(concat),'-c','copy','-movflags','+faststart',str(output)])
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_format','-show_streams','-of','json',str(output)],text=True));duration=float(probe['format']['duration']);assert 60<=duration<=180
    run(['-i',str(output),'-f','null','-'])
    meta={'status':'built_visual_review_pending','purpose':'Improvement review only; 16x exceeds the existing submission 2x constraint. Submitted video is unchanged.','source_sha256':SOURCE_SHA,'source_interval_pts_s':[START,END],'recorded_interval_s':END-START,'one_contiguous_interval':True,'within_motion_interval_cuts':0,'playback_speed_against_wall_capture':SPEED,'source_capture_fps':5,'output_fps':30,'frame_processing':'Standard frame dropping/duplication for 16x playback at 30fps; no generated/interpolated robot motion','source_gap_before_interval':[22.6,921.0],'gap_cause':'Unknown. Earlier missing recording not reconstructed.','source_phase_at_interval_start':'primary_observation, before primary_hover/motion start, verified visible caption','duration_s':duration,'full_decode_passed':True,'sha256':hashlib.sha256(output.read_bytes()).hexdigest(),'file_name':output.name,'youtube_changed':False,'coursework_resubmitted':False}
    (OUT/'validation.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n');print(json.dumps(meta,ensure_ascii=False))

if __name__=='__main__':main()
