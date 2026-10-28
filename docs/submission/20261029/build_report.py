"""Generate the final pickup/hold report from the preserved formal evaluation."""
import hashlib
import json
from html import escape
from pathlib import Path
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.platypus import Paragraph, Table, TableStyle
from PIL import Image

ROOT = Path(__file__).resolve().parents[3]
BASE = ROOT / 'docs/submission/20261029'
EVAL = ROOT / 'docs/evaluation/pickup_hold_formal_20261028_b'
FONT = '/System/Library/Fonts/Supplemental/Arial Unicode.ttf'
NAVY = '#14253b'; TEAL = '#087f80'


def main():
    out = BASE/'deliverables/説明資料_PAI最終課題_283t_123.pdf'
    pdfmetrics.registerFont(TTFont('JP', FONT))
    c = canvas.Canvas(str(out), pagesize=(595.28, 841.89), invariant=1)
    c.setTitle('図面読解AIを用いた双腕ロボットによる柔軟ハーネスの取り出し・保持')
    c.setAuthor('283t_123'); c.setSubject('固定条件の正式3試行と実録・限界・再実行手順')
    style = ParagraphStyle('JP',fontName='JP',fontSize=10,leading=16,wordWrap='CJK',textColor=colors.HexColor(NAVY))
    audits = {n:json.loads((EVAL/f'r0{n}_result/formal_audit.json').read_text()) for n in (2,3)}

    def para(text,y,size=10,colour=NAVY):
        s=ParagraphStyle('p',parent=style,fontSize=size,leading=size*1.6,textColor=colors.HexColor(colour))
        p=Paragraph(escape(text).replace('\n','<br/>'),s); _,h=p.wrap(491,740)
        assert y-h>53, text[:60]
        p.drawOn(c,52,y-h);return y-h-10

    def head(n,title,sub):
        c.setFillColor(colors.HexColor(NAVY));c.rect(0,748,596,94,fill=1,stroke=0)
        c.setFillColor(colors.white);c.setFont('JP',19);c.drawString(52,797,title)
        c.setFont('JP',9);c.drawString(52,775,sub)
        c.setFillColor(colors.HexColor('#607080'));c.setFont('JP',8)
        c.drawString(52,29,'Physical AI 応用1 | 283t_123 | 2026-10-29');c.drawRightString(543,29,f'{n} / 4')
        return 724

    def section(text,y):return para(text,y,13,TEAL)
    def photo(path,y,height):
        with Image.open(path) as im:r=min(491/im.width,height/im.height);w,h=im.width*r,im.height*r
        c.drawImage(str(path),52+(491-w)/2,y-h,w,h);return y-h-8

    y=head(1,'双腕によるハーネス取り出し・保持','図面読解AIとRGB-D認識を用いた固定条件の物理デモ')
    y=para('電機メーカーの多品種少量生産を想定し、接続作業に先立つハーネスの取り出し・保持支援を試作した。正式評価は固定1条件で2成功／3試行。基板への挿入・全端子接続は未達。',y,11)
    y=photo(EVAL/'r03_result/video_review_end.png',y,275)
    y=para('正式B r03の実録末尾。右腕が黄色のP3コネクタ、左腕が分岐ワイヤーを把持。図面と整合する基板は奥に配置するが、今回は挿入を行わない。',y,9)
    y=section('目的と今回の範囲',y)
    y=para('図面に応じて接続先を変えられる工場工程を目指す。最終課題では範囲を取り出し・保持へ絞り、設計D101、配置seed 2、P3、停止した供給台を固定した。任意図面や配置への汎化を示した作品ではない。',y)
    y=section('独自に実装・工夫した点',y)
    y=para('公式MELFAモデルを2本の独立した腕として構成。図面読解と把持対象のRGB-D認識を分離し、端子とワイヤーを異なる腕で支持する。柔軟リンクの物理計算を維持し、全把持観測・物理時刻・監視対象接触から実行結果を独立再検査した。',y)
    c.showPage()

    y=head(2,'AI・認識・柔軟物の構成','画像から対応を読み、カメラから把持位置を求める')
    y=section('図面読解：ローカルQwen3-VL 2B',y)
    y=para('ローカルOllama / qwen3-vl:2b-instructへ自作図面画像と共通指示を渡し、端子・接続先・X/Y・向きをJSON化する。基板JSONは推論後の照合にだけ使い、誤答を正解へ補正しない。P3は試験仕様で固定し、AIが任意の作業を選択したとはしない。',y)
    y=photo(ROOT/'docs/evaluation/cooperative_nest_20261016/stepped_descent_tracking_failure/case/drawing.png',y,225)
    y=para('D101の新規読解を10/29に確認：P1→J2 (54,43 mm / 90°)、P2→J1 (164,128 mm / 90°)、P3→J3 (52,128 mm / 0°)。全3行の照合を通過した。正式Bは以前に監査済みの同じ図面の推論を再利用し、新規読解を正式試行へ後付けしていない。',y,9)
    y=section('双腕の実行と柔軟ワイヤー',y)
    y=para('新規RGB-Dの安定3枚から端子の位置・向きを認識する。右把持→右単独100 mm持上げ・4物理秒保持→左腕のワイヤー把持→双腕100 mm持上げ・4物理秒保持の順に実行。物体真値は独立評価専用で、把持目標の生成には用いない。',y)
    y=para('1本の3分岐ハーネスを、31リンク・27曲げ関節の木構造で近似する。端子を持ち上げると他の枝にも力が伝わる。実材料の曲げ剛性・摩擦・張力を同定したモデルではなく、実機での作業性能は未検証。',y)
    c.showPage()

    y=head(3,'事前登録した3試行の全結果','同じ候補・初期入力・物理条件を固定し、失敗も保持')
    y=para('正式シリーズB：候補40496fb、事前登録7064a92。設計101・配置2で新しい初期状態から順番に3回実行した。コード・入力のSHAを各回で照合。開発成功や別シリーズの成功を追加していない。',y)
    rows=[['試行','結果','右単独／双腕保持（物理秒）','双腕の最小上昇 左／右'],['r01','失敗','右 4.0001 ／ 双腕未実施','左閉指前 Lost feedback']]
    for n,a in audits.items():
        h=a['hold_results'];w=a['hold_windows'];rows.append([f'r0{n}','成功',f"{w['primary_hold']['elapsed_simulation_s']:.4f} ／ {w['dual_hold']['elapsed_simulation_s']:.4f}",f"{h['left_arm']['minimum_lift_m']*1000:.3f} ／ {h['right_arm']['minimum_lift_m']*1000:.3f} mm"])
    table=Table([[Paragraph(escape(v),ParagraphStyle('cell',parent=style,fontSize=8,leading=13)) for v in row] for row in rows],colWidths=[35,35,205,216]);table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#dcefed')),('GRID',(0,0),(-1,-1),.4,colors.HexColor('#b4c8cc')),('VALIGN',(0,0),(-1,-1),'TOP'),('TOPPADDING',(0,0),(-1,-1),8),('BOTTOMPADDING',(0,0),(-1,-1),8)]));_,h=table.wrap(491,600);table.drawOn(c,52,y-h);y-=h+16
    y=section('判定基準と独立検証',y)
    y=para('各試行で右単独・双腕の一連完了を要求。上昇70 mm超、相対・持上げ時の滑り5 mm未満、回転変化0.05 rad未満を維持した。4物理秒の開始・終了時計と10個以上の異なる物理時刻の観測を要求。成功条件を後から緩めていない。',y)
    y=para('r02：全把持観測 右67,520／左18,188件。r03：右67,666／左18,547件。全件再計算は保存判定と一致し基準内。双腕保持窓は各8,512／9,443標本。監視対象の腕・治具・他端子との意図しない接触は記録上なし。未計測接触まで保証しない。',y)
    y=section('2成功／3試行（66.7%）と残した失敗',y)
    y=para('承認済み固定条件の物理評価（2/3以上）を満たした。r01はLost feedbackで停止した。どの情報が遅れたかを特定できず、ネットワーク障害などとは断定しない。右保持の部分通過を一連成功へ読み替えていない。',y)
    y=para('別シリーズAは字幕再読込エラーで1試行中断・失敗、残り2試行は未実行のまま保存。字幕を同一inodeの固定長更新へ修正してBを新規登録した。全270テストとFFmpeg字幕再読込329更新を確認した後にBを実行した。',y)
    c.showPage()

    y=head(4,'実録・再実行・未達点','提出版は到達点と制約を区別して説明する')
    y=section('提出動画：170秒、実録の抜粋',y)
    y=para('正式B r03の右把持、右持上げ、左接近・把持、双腕持上げ・保持を8区間で示す。実時間記録に対する2倍速と区間省略を動画内に明記。説明画像を実動作と混同せず、動作の補間・生成は行わない。物理計算が低速のため、保持4物理秒の成立は全ログで確認する。',y)
    y=para('原録画のPTS 22.6→921.0秒に898.4秒の間隔がある。原因は未特定で、動画内に欠落を明記した。全8抜粋はこの区間より後。実画像・位相・単調時計字幕を照合して選択した。全編デコードと各区間の始端・終端を確認したが、映像の連続性を全実行時間にわたり保証しない。',y)
    y=section('公開コードと再実行',y)
    y=para('https://github.com/283T/PhysicalAI2-1-finalReport\n入口：docs/reproduce-submission.md\n評価登録・全結果：docs/evaluation/pickup_hold_formal_20261028_b/\n提出版・生成コード：docs/submission/20261029/',y,9)
    y=para('Ubuntu 24.04 / ROS 2 Jazzy / Gazebo 8.15.0をDockerで使用。検証機はApple M3・16 GB。公式MELFAリビジョンを取得し、公開アーカイブから初期入力をSHA照合して復元する。新しい出力先でrecord_dual_hold.pyを起動し、finishedと独立監査を確認する。既存実験を上書き・同時起動しない。クリーンcloneから環境再ビルドを含む一括再実行は未検証。',y)
    y=section('未達・今後の工程',y)
    y=para('基板挿入、全コネクタ接続、複数基板・ランダム配置での物理検証、搬送中認識、所定場所への置き戻しは未達。ラッチ係合・導通・実機・材料特性も評価していない。旧全接続12試行Goalは未完了のまま保存。次は固定1接続を成立させてから、全端子・設計変更・配置変動へ段階的に拡張する。',y)
    y=section('利用元と公開範囲',y)
    y=para('MitsubishiElectric/melfa_ros2_driver：rev 10098bed6bcca4e4bef46e36389a839065f8cb22、melfa_description 1.1.1（Apache-2.0）。公式の形状・運動学を使用し、双腕化・把持器・Gazebo制御・認識と監査を独自実装した。実機ドライバは動かしていない。\nQwen3-VLはローカル推論。第三者ライセンスはTHIRD_PARTY_NOTICES.mdとlicenses/に記載。氏名・所属・連絡先は記載せず、OmniCampusアカウント名のみ使用。',y,9)
    c.save()
    print(out)
    print('SHA256',hashlib.sha256(out.read_bytes()).hexdigest())

if __name__=='__main__':main()
