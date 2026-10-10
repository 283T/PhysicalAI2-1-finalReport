# 図面によるハーネス接続先の切り替え

| 図面 | ハーネス / プラグ | 接続先 | 中心X | 中心Y | 向き |
|---|---|---|---:|---:|---:|
| A | WH1 / P1 | J1 | 60 mm | 50 mm | 0° |
| B | WH1 / P1 | J2 | 100 mm | 90 mm | 0° |

図面のCONNECT注記からハーネス・端末プラグ・接続先を読む。共通パネルに両方の受け側コネクタを用意し、未指定のソケットは空ける。実行ごとに新しいパネルとハーネスで開始し、接続済みのプラグを瞬間移動させない。

![接続図A](../drawings/assembly_a.png)

![接続図B](../drawings/assembly_b.png)

図面のケーブル線は模式図。PNGをモデルに入力し、正解JSONや配置一覧はモデルに渡さない。基板上の電子部品配置ではなく、ハーネス端と受け側の接続関係を扱う。

図面の生成元は`scripts/build_drawings.py`、配置は`configs/board.json`、正解比較用の指示は`examples/task_a.json`と`task_b.json`。[ローカルAI](local-ai.md)で読解し、[接続動作](insertion-demo.md)へ渡す。
