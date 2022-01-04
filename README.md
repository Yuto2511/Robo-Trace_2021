# Robo-Trace
## ロボットの種類
   -  基板シャーシ
      - 回路の面積が増える。
   -  スキッドステア
      - 4輪ステアリングは設計に時間がかかってしまうので却下、対向二輪はカーブで有利になるが直線での安定性に不安が残る。
   -  センサステア型
      - ステアリングモータから角速度を取得することで駆動モータの出力比を決定できる。センサ固定よりもセンサステアのほうが角度の精度が高い。

## ロボットサイズ
   -  トレッド  115mm
   -  ホイールベース中心からセンサーバー稼働軸  40mm
   -  センサーバー   70mm
   -  動輪直径  27mm  <= 2019 2位参考
   -  動輪　幅  12mm  <= 最初

## 性能
   - 速度 v = 9.0 m/s
   - 加速度 a = 10.0 m/s
   - 質量 m <= 140 g

## パーツ
   - 動輪部分
      - 平歯車　M0.3,　歯数 80,　直径 24.6mm,　厚さ 2.0mm,　ピッチ円 24mm
      - ピニオン　M0.3,　歯数 28,　直径 9mm,　ピッチ円 8.4mm
      - モータ　maxon DCX 10 L　(公称電圧 4.5V)　シャフトの長さ 4.9mm
      - ベアリング 外形 7mm　内径 4mm [詳しくはこれ](https://jp.misumi-ec.com/vona2/detail/110300116230/?CategorySpec=00000228895%3a%3ab%0900000228694%3a%3amig00000002484781%0900000229286%3a%3amig00000002491660%0900000228562%3a%3ag&clkid=clkid_basic_shape_template&list=PageCategory)
      ***
   - センサーバー周り
      - モータ　maxon DCX 6 M　(公称電圧 4.5V)　シャフトの長さ 8.2mm
      - ピニオン(モータ側)　M0.3,　歯数 11,　直径 3.9mm,　厚さ 2.0mm, ピッチ円 3.3mm
      - 平歯車(モータ側)　M0.3,　歯数 77,　直径 mm,　厚さ 2.0mm, ピッチ円 23.1mm
      - ベアリング(モータ側)　外形 5.0mm　内径 2.0mm
      - ピニオン(センサ側)　M0.3,　歯数 11,　直径 3.9mm,　厚さ 2.0mm, ピッチ円 3.3mm
      - 平歯車(センサ側)　M0.3,　歯数 88,　直径 mm,　厚さ 2.0mm, ピッチ円 26.4mm
      - ベアリング(センサ側)　外形 7.0mm　内径 4.0mm
      ***
   - その他
      - シャーシ基板　厚さ 1.2mm
      - センサ基板　厚さ　1.0mm
      - プリントパーツ　オニキスを使用

## CADデータ
   ![image](https://user-images.githubusercontent.com/83150974/141968967-4ca1aa63-3e5b-4090-8734-b0a225b22d6f.png)

## 電子パーツ
   - マイコン
      ***
   - 電源回路周り
      ***
   - 車輪モータ駆動部
      ***
   - センサステアモータ駆動部
      ***
   - ラインセンサ部
      - フォトトランジスタ  
         　コレクタエミッタブレークダウン電圧は、大きいほうがいい？ので3セルのリポを繋いでも大丈夫なようにする。レギュレータ等で減圧するのでなんとも言えないが...。暗電流について、ノイズとして制御に影響すると考えるため、できる限り小さいものを選ぶべきなのか？  
           　以上を考慮して以下の2つを選んだ。1つ目は、かっこよくないが性能はいいはず。2つ目は先輩が使っているので安心だが1つ目より暗電流が大きい。しかしカッコイイ！
         - [SFH 3400-2/3-Z](https://www.digikey.jp/ja/products/detail/osram-opto-semiconductors-inc/SFH-3400-2-3-Z/1989740)
         - [TEMT7000X01](https://www.digikey.jp/ja/products/detail/vishay-semiconductor-opto-division/TEMT7000X01/4075721)
      - 赤外線LED  
      　上記どちらのフォトトランジスタを選んでも、受光する光の波長は850nmなので以下に決定する。選ぶ条件として、850nmに近い波長を出せることと、カッコよさで選ぶ。
         - [15406085BA300](https://www.digikey.jp/ja/products/detail/w%C3%BCrth-elektronik/15406085BA300/8557170)
      ***
   - サイドセンサ部  
         　部品はラインセンサ部と同じものを使用するが、フォトトランジスタの動作モードをスイッチモードとして使おうと考えている。マーカーの有無の評価すればよいので、アクティブモードを使用する必要はない。  
         　モード切替は負荷抵抗Rの値によって決まる。アクティブモードは、Vcc > R * Ic のときで、スイッチモードは、Vcc < R * Ic のときとなる。このときの Vc,Ic はそれぞれ、電源電圧、最大予測電流となる。くわしくは[ここ](https://www.digikey.jp/ja/articles/how-to-use-photodiodes-and-phototransistors-most-effectively)。
      ***
   - デバックとその他
      - LED
        - [LED Red](https://akizukidenshi.com/catalog/g/gI-03978/)
        - [LED Green](https://akizukidenshi.com/catalog/g/gI-06417/)
        - [LED RGB](https://akizukidenshi.com/catalog/g/gI-06418/)
      - IMU  
        調べた限り、以下の3つしかよさげなものがなかった。もし先輩達が在庫を持っていたら...欲しいですね。  
        今のところ IAM-20680HT が候補。
        - [IAM-20680HT](https://www.digikey.jp/ja/products/detail/tdk-invensense/IAM-20680HT/15792840)
        - [FSP200](https://www.digikey.jp/ja/products/detail/ceva-technologies-inc/FSP200/10283874)
        - [MPU-6500](https://www.digikey.jp/ja/products/detail/tdk-invensense/MPU-6500/4385412)
