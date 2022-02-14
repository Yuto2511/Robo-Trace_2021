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
   ![iOS の画像 (1)](https://user-images.githubusercontent.com/83150974/153847095-abf01e89-e148-4ad2-ac0c-91faad6aa42d.jpg)

## 電子パーツ
   - マイコン  
     - 候補  
       - [STM32F446](https://www.digikey.jp/ja/products/detail/stmicroelectronics/STM32F446RET6/5175962)
       - [STM32F405](https://www.digikey.jp/ja/products/detail/stmicroelectronics/STM32F405RGT7/5051343) <= これ使う
     - 必要なピン数
       - ADC   5個以上
       - PWM   3個以上
       - GPIO  あるだけいい、どうせ足りる　<= 64pinじゃ足りなかったので次は100pinかな
     - メモリに関して  
     　メモリを食うのは主にコース記憶に関してで、コースの長さは最長60mでもし1cm間隔でデバックしていった場合、1つのデバッカーで単純計算6,000個のデータが必要になる。1データ4bitで計算すると24KBの容量を食う。もしIMUとエンコーダを使った場合、機体全体(センサーバーも含む)で4個、つまり96KB使うこととなる。また0.5cm間隔でデバックした場合2倍の192KBの容量を食うことになる。  
     　[STM32F446](https://www.digikey.jp/ja/products/detail/stmicroelectronics/STM32F446RET6/5175962)、[STM32F405](https://www.digikey.jp/ja/products/detail/stmicroelectronics/STM32F405RGT7/5051343)のどちらを選んでもRAMが足りないので、フラッシュメモリに書き込むことになる。いっそのことSDカードを積むのも手だと思う。その場合、速度の速い[STM32F446](https://www.digikey.jp/ja/products/detail/stmicroelectronics/STM32F446RET6/5175962)を使うことになる。
      ***
   - 電源回路周り  
   　今回初めての回路設計なので、DC/DCコンバータと3端子レギュレータの両方を使ってみたい。なのバッテリーからDC/DCコンバータで減圧して、そこから3端子レギュレータで減圧したいと思う。 
      - DC/DCコンバータ  
        DC/DCコンバータとは何かについて、[この動画](https://www.youtube.com/watch?v=SFsMFGFOkWY&list=PLmhf46XKsdmTrVYV_fUMa2XLk81oa_NDB)で勉強した。　　
         - [LTC2624IMSE-25](https://www.digikey.jp/ja/products/detail/analog-devices-inc/LTC3624IMSE-25-PBF/5699826)
         - [MPLCG0630L1R5(インダクタ)](https://www.digikey.jp/ja/products/detail/kemet/MPLCG0630L1R5/4291043?s=N4IgTCBcDaILIAUAyBhA4gBgGwGYNIEYAlAVhAF0BfIA) <= 在庫があればこれ
         - [XAL4020-152MEC(インダクタ)](https://www.mouser.jp/ProductDetail/Coilcraft/XAL4020-152MEC?qs=zCSbvcPd3pbTTY9oGElImA%3D%3D) <= これ在庫いっぱい
         - [SMAZ15-13-F(ツェナー)](https://www.digikey.jp/ja/products/detail/diodes-incorporated/SMAZ15-13-F/775753) <= 正直どれ使えばいいかわからない、とりあえず公称15Vにした。
      - 三端子レギュレータ
         - [LP38690DTX-3.3/NOPB](https://www.mouser.jp/ProductDetail/Texas-Instruments/LP38690DTX-33-NOPB?qs=1FNqv8aZn1TYRyMpXrH3Aw%3D%3D)
      - 共通部品
         - [アルミ電解コンデンサ 100uF(丸)](https://akizukidenshi.com/catalog/g/gP-14452/) <= 低ESRで、ノイズに強いらしい。
      ***
   - 車輪モータ駆動部  
     どこのサイトでも在庫がなく入荷も一年後と入手できないため、先輩からもらうことにする。
      - [DRV8874](https://www.digikey.jp/ja/products/detail/texas-instruments/DRV8874PWPR/11502339)
      - [アルミ電解コンデンサ 100uF(四角)](https://akizukidenshi.com/catalog/g/gP-06855/) <= これもノイズに強いらしい、だが定格12.5Vだから3セル積んだら終わるかも...
      ***
   - センサステアモータ駆動部  
     モタドラ系の回路は車輪モータと同じ。
     - エンコーダ  
     センサーバーの位置制御用にロータリーエンコーダを用いる  
         - [RDC503013A](https://jp.misumi-ec.com/vona2/detail/222005900303/?HissuCode=RDC503013A)
      ***
   - ラインセンサ部
      - フォトトランジスタ  
         　コレクタエミッタブレークダウン電圧は、大きいほうがいい？ので3セルのリポを繋いでも大丈夫なようにする。レギュレータ等で減圧するのでなんとも言えないが...。暗電流について、ノイズとして制御に影響すると考えるため、できる限り小さいものを選ぶべきなのか？  
           　以上を考慮して以下の2つを選んだ。1つ目は、かっこよくないが性能はいいはず。2つ目は先輩が使っているので安心だが1つ目より暗電流が大きい。しかしカッコイイ！
         - [SFH 3400-2/3-Z](https://www.digikey.jp/ja/products/detail/osram-opto-semiconductors-inc/SFH-3400-2-3-Z/1989740)
         - [TEMT7000X01](https://www.digikey.jp/ja/products/detail/vishay-semiconductor-opto-division/TEMT7000X01/4075721)
      - 赤外線LED  
      　上記どちらのフォトトランジスタを選んでも、受光する光の波長は850nmなので以下に決定する。選ぶ条件として、850nmに近い波長を出せることと、カッコよさで選ぶ。
         - [15406085BA300](https://www.digikey.jp/ja/products/detail/w%C3%BCrth-elektronik/15406085BA300/8557170) <= 在庫切れ  
        欲しかったLEDの在庫がないので、先輩と同じものを使う…
         - [SIR19-21C/TR8](https://www.digikey.jp/ja/products/detail/everlight-electronics-co-ltd/SIR19-21C-TR8/2676159)
      ***
   - サイドセンサ部  
     赤外線LEDはラインセンサと同じものを使う。
      - [S7136](https://akizukidenshi.com/catalog/g/gI-02425/) <= 先輩と同じ。HIGHLOW出力
      ***
   - デバックとその他
      - スイッチ
        - [SKRHAAE010](https://akizukidenshi.com/catalog/g/gP-14676/)
        - [SKRPABE010](https://akizukidenshi.com/catalog/g/gP-06184/)
      - LED
        - [LED Red](https://akizukidenshi.com/catalog/g/gI-03978/)
        - [LED Green](https://akizukidenshi.com/catalog/g/gI-06417/)
        - [LED RGB](https://akizukidenshi.com/catalog/g/gI-06418/)
      - IMU  
        調べた限り、以下の3つしかよさげなものがなかった。もし先輩達が在庫を持っていたら...欲しいですね。  
        在庫があるとのことなので、ICM-20648を使うことにする。
        - [ICM-20648](https://www.digikey.jp/ja/products/detail/tdk-invensense/ICM-20648/5872876) <= これを使う
        - [IAM-20680HT](https://www.digikey.jp/ja/products/detail/tdk-invensense/IAM-20680HT/15792840)
        - [FSP200](https://www.digikey.jp/ja/products/detail/ceva-technologies-inc/FSP200/10283874)
        - [MPU-6500](https://www.digikey.jp/ja/products/detail/tdk-invensense/MPU-6500/4385412)
      - SDカード  
      めんどくさいので、先輩のと同じものを使う。  
        - [ヒロセ・マイクロSDカードコネクタ DM3AT-SF-PEJM5](https://akizukidenshi.com/catalog/g/gC-02395/)
        - [KIOXIA マイクロSDカード(microSDHC)EXCERIA 16GB 100MB/s](https://akizukidenshi.com/catalog/g/gS-15845/)
      - LCD  
        - [I2C接続小型LCDモジュール(8*2行)](https://akizukidenshi.com/catalog/g/gP-06669/)
      - コンデンサ、抵抗  
        こいつらは、適当に選んだ。サイズは「0603」「0402」の2種類のみ。

## 回路図＆PDB  
  - 回路図  
  回路図のデータを以下に示す。mainとsubのどちらの回路も一つのpdfにまとめてある。  
  [RoboTrace_回路図](https://github.com/Yuto2511/Robo-Trace/files/8059954/RoboTrace_2021.pdf)
  - PCB  
  ![スクリーンショット 2022-02-06 200413](https://user-images.githubusercontent.com/83150974/153848313-383bf9f2-6549-4da8-8a45-d2c32222067a.png)

