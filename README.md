# -STM32-Arduino-ICSservo
STM32 x Arduino にて近藤科学ICS 3.5 サーボを使用するライブラリです。

近藤科学のICSサーボは最大1.25Mbpsと高い通信速度で処理を行えますが、STM32F103等は48～72MHzの高い処理能力を持っており、1.25Mbpsでの処理が可能です。
当ライブラリではその基本機能を提供します。
接続回路は、1線式UARTを想定しています。
プリメイドAIハックで初めてロボットいじりの楽しさを知ったので、是非いろんな方に使ってほしいです。

●注意！
私の環境では全機能を利用できていますが、EEPROM書き替えについては書き込みデータ内容を誤るとサーボの基板の修理が必要になる恐れがあります。
EEPROM書き込みに関しては、可及的にデータの整合性をチェックしたコードを意識しました。
しかし、利用にあたりできましたら、まずはEEPROM書き替え以外の機能から試しにご利用頂き（サーボ移動やパラメータ系、EEPROM読み出し）、その後サーボ一台でEEPROM書き込みをお試しください！

●機能
・サーボ移動、脱力
・（独自）サーボ脱力後に同位置で即動作（現在位置確認用）
・全パラメータ読み書き（EEPROM含む）
・ID書き換え（通常コマンド、ホスト－サーボを１対１接続時）
・（独自）ID書き換え（EEPROM書き替えにより、複数接続時でも可能）


●動作確認
当方、プリメイドAI（STM32F102）、BluePill（同F102）

●利用する想定回路
プリメイドAIであれば、回路はそのままで利用できます。
BluePill（STM32F103）等であれば、
STM32F103 - レベルコンバータ（3.3V-5V） - ICSサーボ
接続するpin番号 は、
Serial2: PA2
Serial3: PB10
となっております。(Serial1: PA9)　当ライブラリではSerial1はデバッグ用として利用想定しています。Serial2-3を利用下さい。
電源・GNDは別途用意してください。HV系(9-12V)、MV系（6-7.4V）


●利用方法
https://github.com/devemin/Pre-maiduino
をご覧いただき、Arduino IDE にSTM32ボード情報をインストールしてください。
STM32とPCの接続は、ST Linkケーブルを利用します。
その後、当ライブラリのコードをArduino IDE にて開いてビルド、STM32に転送してもらえば利用できるかと思います。

●コード例
コード loop 内処理サンプルをご覧ください。
あえて１ファイルにクラスのコードをまとめてありますので、必要に応じ適宜別ファイルに移すなどしてください。
IcsCommunication ics2(Serial2);
void setup() {
  ics2.begin(SERIAL2_BAUDRATE, 50, true);   //初期化
}
void loop() {
    int retval = set_position(1, 7500);     //サーボ移動（ID, ポジション）
} 


●補足
未確認ですが、内部のtransceive 関数と通信速度等を書き替えれば、他プラットフォームでも使えるのではないかと思います。（他Arduino機種）
また、ICS 3.6規格には現在位置取得コマンドが存在しますが、それもtranscieve 関数を利用してもらえば、簡単に実装できると思います。


●作成者
devemin

●ライセンス
MIT Liscence

●注意事項
当ライブラリから発生する事象に対し、責任は全て利用者にあります。
EEPROMの書き換え不良や高いパラメータによる破損等、コードをお読みになってご利用下さい。


●更新履歴
ver 0.50:  publish
