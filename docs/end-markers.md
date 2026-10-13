# 端面マーカーと把持直前の再観測

既存のハウジング側面マーカーは、指・前後ストッパーで隠れることがありました。`--end-markers` は40×20×20 mmの試作端子の長軸両端に16 mmのArUcoマーカーを加えます。P1はID 0/1、P2は2/3、P3は4/5で、偶数が正のX面、奇数が負のX面です。白地と黒いセルは表示のみで、衝突形状・質量・ケーブルのジョイントを変えません。

`end_markers.py` は、識別したマーカー四隅とRGB-Dの深度平面から端子中心・回転を復元します。画像の拡大は検出を助けるための処理であり、四隅を元の画素座標へ戻してCameraInfoを使います。深度なし、既知寸法との不整合、同じ端子の複数検出を拒否します。端子の真値は復元に与えません。ArUcoの辞書・四隅の識別は[OpenCV公式資料](https://docs.opencv.org/4.13.0/d5/dae/tutorial_aruco_detection.html)を参照しています。実行環境のOpenCV 4.6では、検出器パラメータの初期化に旧APIのファクトリを使用します。

## 測定結果（2026-10-13）

| 条件 | 対象 | 中心誤差 | 回転誤差 | 結果 |
|---|---|---:|---:|---|
| パレット上の静止姿勢・カメラ高さ0.50 m | P3 | 0.44 mm | 約0.40° | 独立したGazebo姿勢との照合を通過 |
| 把持直前・カメラ高さ0.32 m | P3 | 0.25 mm | 約0.62° | 同期した記録済み姿勢との比較を通過 |

[静止姿勢の記録](evaluation/end_markers/p3_static/validation.json)、[把持直前の姿勢比較](evaluation/end_markers/preclose_guarded/preclose_pose_evaluation.json)に、画像・深度・校正・world・判定・SHA-256を保存しました。後者の画像と真値の時間差は約5 msです。いずれもD101／配置seed2／P3、理想校正の単発試験であり、全端子・ランダム配置・実機の精度を評価していません。静止試験の旧判定ファイルの `xy_error_m` は端面表面と中心の差を含んでおり、復元精度には `center_error_m` を用います。現在の検証器では端面マーカーのXYも復元中心から計算します。

## 閉指前の判定

`--verify-before-close` は把持位置への到達後、指を開いたまま端面を再観測します。画像の端子姿勢と初回認識で選んだ工具との位置関係から、必要な工具姿勢を計算します。実測工具との差が2 mmまたは0.025 rad以上、画像が古い、あるいは要求した端子の観測が欠けていれば閉指しません。この工程は再確認であり、画像に合わせて工具を補正する処理はまだありません。

実行したP3の単腕試験では、工具とのずれが約3.29 mm・約2.77°となり、[閉指前に停止](evaluation/end_markers/preclose_guarded/validation.json)しました。開いた状態で再観測する工程まで通過した結果であり、把持・持ち上げ・保持・挿入の成功ではありません。`final_open` は後片付けの開指で、試験の閉指工程 `close` は実行されていません。

保持中の照合と把持直前の再確認には、同じ非同期の観測処理を使います。観測待ちの間も指令とフィードバックを維持します。把持前の認識は従来どおりP1/P2/P3すべてを要求します。診断時の `--plugs` は指定した対象すべてを要求し、今回の単腕検証はP3だけです。双腕の場合は選んだ2端子の観測を要求します。P1・P2の端面認識や双腕での再確認は未評価です。

## 待機腕を含む経路検査

最初のP3単腕試験は、右指と待機中の左上腕が[接触して停止](evaluation/end_markers/single_arm_contact/validation.json)しました。単腕モードでは待機腕が事前の箱検査に含まれていませんでした。修正後は両腕の関節を観測し、待機腕の実測位置を一定経路としてIK候補と実行経路の検査に含めます。待機腕の移動やフィードバック消失も拒否します。IK・箱検査の間に止まっていたROSコールバックは、指令前に処理して新しい観測を確認します。

修正後の[同じシーンでの再試験](evaluation/end_markers/preclose_guarded/validation.json)は接近・下降・端面再観測まで通過し、腕同士の接触記録は0件でした。待機腕を含む包絡箱の検査であり、自己・ケーブル・物体・搬送台との衝突をすべて保証するものではありません。

```bash
python3 scripts/run_perception_demo.py --design-seed 101 --placement-seed 2 \
  --evaluate --pick-connector P3 --regrasp-nest \
  --gripper-p-gain 3000 --capture-gripper --front-fence-side right \
  --end-markers --inspection-calibration configs/end_inspection_camera.json \
  --verify-before-close --verify-held-pose --headless --release-ai-memory \
  --output outputs/end_marker_preclose_p3
```

最終検証は110件の単体テストが通過しています。次は新しい画像姿勢に基づく小さな工具補正を検討し、閉指直前に再度確認します。P1を見られる視点も追加し、双腕の安定把持へ統合します。
