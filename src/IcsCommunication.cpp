#include "IcsCommunication.hpp"

// コンストラクタ
IcsCommunication::IcsCommunication(UnbufferedSerial &ser, PinName icsPinName)
    : icsPin(icsPinName) {
  refSer = &ser;
}

// 初期化関数
// 引数：　ボーレート(115200, 625000,
// 1250000)、タイムアウト値、500msecウェイト有無
//  initHighで、500msecウェイトを最初に実行するかどうか決めます。
// （サーボ通信をシリアルのみとする手続き。EEPROMフラグに書き込み済なら不要）
bool IcsCommunication::begin(uint32_t brate, bool initFlag) {
  baudrate = brate;
  initHigh = initFlag;

  if (initHigh) {
    // ICSサーボ設定　信号線を起動時500msec
    // Highにすることで、シリアル通信になる（誤ってPWMにならないよう）
    // さらに心配なら、別途 EEPROMのPWMINHフラグを１にする事。
    icsPin = 1;
    // wait_ms(550); // 長めに550msec.
    wait_us(550 * 1000);
    icsPin = 0;
  }

  // ICSサーボ通信設定 (8bit, EVEN)
  refSer->baud(baudrate);
  refSer->format(8, SerialBase::Even, 1);
  return true;
}

// ボーレート変更関数　いつでも変更可能
//  ICSでは115200, 625000, 1250000 のみ対応
void IcsCommunication::change_baudrate(uint32_t brate) {
  //  serial baud関数を使う
  refSer->baud(brate);
}

// 基本的なサーボとのデータ送受信関数　すべてのベース
// 引数：　送信バッファ、受信バッファ、送信サイズ、受信サイズ
int IcsCommunication::transceive(uint8_t *txbuf, uint8_t *rxbuf, uint8_t txsize,
                                 uint8_t rxsize) {

  int retLen;
  uint8_t tmpbyte = txbuf[0]; // デバッグ用

  // 送信前にICS信号線をHighにする
  icsPin = 1;
  // 送信
  retLen = refSer->write(txbuf, txsize);
  // 送信後にICS信号線をLowにする
  icsPin = 0;

  // 送信終わったのでtxbuf をゼロに（安全のため）
  for (int a = 0; a < txsize; a++) {
    txbuf[a] = 0;
  }

  if (retLen != txsize) {
    // error
    return RETCODE_ERROR_ICSREAD;
  }

  /*
  // TXビット　オフ　/ RXビット　オン
  regmap->CR1 = (regmap->CR1 & 0b111111111111111111111111111110011) |
                0b000000000000000000000000000000100;
  */
  // 受信前なのでrxbuf をゼロに
  for (int a = 0; a < rxsize; a++) {
    rxbuf[a] = 0;
  }

  // ここの空読みは、どうも必要だったり必要なかったり・・・
  // 自分の回路の最初の環境では、これを抜くとループバックバイト列を余分に受信してしまうことがあった。
  // しかしプルアップ抵抗をしっかりブレッドボードにさしたら、逆にこれが邪魔でデータを消してしまった。
  // 近藤科学に問い合わせたときは、プルアップ抵抗の値を調整してみてはどうでしょう、とのことだった。
  // プリメイドＡＩではとりあえず空読みしておく
  while (refSer->readable() > 0) // 受信バッファの空読み
  {
    //  todo 必要か要確認
    uint8_t tmp = 0;
    refSer->read(&tmp, 1);
  }

  retLen = refSer->read(rxbuf, rxsize);

  if (retLen != rxsize) {
    return RETCODE_ERROR_ICSWRITE;
  }

  return RETCODE_OK;
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
// サーボ制御系

// 補足：サーボポジションの戻り値は、3500-11500範囲より少しだけ超える値が返ることもある(3481,
// 11541とか)

// サーボ位置移動
// 引数：サーボＩＤ、ポジション値(3500-11500)
// 戻り値に現在位置が入ります。
int IcsCommunication::set_position(uint8_t servolocalID, int val) {
  int txsize = 3; // この関数固有の、送信データバイト数
  int rxsize = 3; // この関数固有の、受信データバイト数
  uint8_t txbuf[3];
  uint8_t rxbuf[3];

  for (int a = 0; a < sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a = 0; a < sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }

  // 引数チェック
  if ((servolocalID < ID_MIN) || (servolocalID > ID_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  if ((val < POS_MIN) || (val > POS_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }

  // 送信データ作成
  txbuf[0] = 0x80 | servolocalID;
  txbuf[1] = val >> 7;
  txbuf[2] = val & 0b0000000001111111;

  // ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  // 受信データ確認
  if (retcode == RETCODE_OK) {
    if ((rxbuf[0] & 0b01111111) == servolocalID) {
      retval = (rxbuf[1] << 7) + (rxbuf[2]);
      return retval;
    } else {
      return RETCODE_ERROR_IDWRONG;
    }
  } else {
    // error
    return retcode;
  }
}

// サーボ脱力
// 戻り値に現在位置が入ります。
int IcsCommunication::set_position_weak(uint8_t servolocalID) {
  int txsize = 3; // この関数固有の、送信データバイト数
  int rxsize = 3; // この関数固有の、受信データバイト数
  uint8_t txbuf[3];
  uint8_t rxbuf[3];

  for (int a = 0; a < sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a = 0; a < sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }

  // 引数チェック
  if ((servolocalID < ID_MIN) || (servolocalID > ID_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }

  // 送信データ作成
  txbuf[0] = 0x80 | servolocalID;
  txbuf[1] = 0;
  txbuf[2] = 0;

  // ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  // 受信データ確認
  if (retcode == RETCODE_OK) {
    if ((rxbuf[0] & 0b01111111) == servolocalID) {
      retval = (rxbuf[1] << 7) + (rxbuf[2]);
      return retval;
    } else {
      return RETCODE_ERROR_IDWRONG;
    }
  } else {
    // error
    return retcode;
  }
}

// 脱力後、即時動作（現在位置確認用）
// 戻り値に現在位置が入ります。
int IcsCommunication::set_position_weakandkeep(uint8_t servolocalID) {
  int txsize = 3; // この関数固有の、送信データバイト数
  int rxsize = 3; // この関数固有の、受信データバイト数
  uint8_t txbuf[3];
  uint8_t rxbuf[3];

  for (int a = 0; a < sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a = 0; a < sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }

  // 引数チェック
  if ((servolocalID < ID_MIN) || (servolocalID > ID_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }

  // １回目の送信データ作成
  txbuf[0] = 0x80 | servolocalID;
  txbuf[1] = 0;
  txbuf[2] = 0;

  // ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  // 受信データ確認
  if (retcode == RETCODE_OK) {
    if ((rxbuf[0] & 0b01111111) == servolocalID) {
      retval = (rxbuf[1] << 7) + (rxbuf[2]);

      // ２回目の送信データ作成
      txbuf[0] = 0x80 | servolocalID;
      txbuf[1] = retval >> 7;
      txbuf[2] = retval & 0b0000000001111111;

      // ２回目のICS送信
      retcode = transceive(txbuf, rxbuf, txsize, rxsize);

      // ２回目の受信データ確認
      if (retcode == RETCODE_OK) {
        if ((rxbuf[0] & 0b01111111) == servolocalID) {
          retval = (rxbuf[1] << 7) + (rxbuf[2]);
          return retval;
        } else {
          return RETCODE_ERROR_IDWRONG;
        }
      } else {
        // error
        return retcode;
      }
    } else {
      return RETCODE_ERROR_IDWRONG;
    }
  } else {
    // error
    return retcode;
  }
}

// EEPROM以外のパラメータ読み取りコマンド
// 引数：サーボＩＤ、ポジション値(3500-11500)
// sccode は、SC_CODE_STRETCH, SC_CODE_SPEED, SC_CODE_CURRENT,
// SC_CODE_TEMPERATURE のどれかが入ります。 SC_CODE_EEPROM
// はエラーとなります。EEPROM用の関数get_EEPROMを使ってください。
// 戻り値に指定パラメータ値、またはエラーコード（負の値）が入ります。
int IcsCommunication::read_Param(uint8_t servolocalID, uint8_t sccode) {
  int txsize = 2;
  int rxsize = 3;
  uint8_t txbuf[2];
  uint8_t rxbuf[3];

  for (int a = 0; a < sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a = 0; a < sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }

  if ((sccode != SC_CODE_STRETCH) && (sccode != SC_CODE_SPEED) &&
      (sccode != SC_CODE_CURRENT) && (sccode != SC_CODE_TEMPERATURE)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }

  // 引数チェック
  if ((servolocalID < ID_MIN) || (servolocalID > ID_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }

  // 送信データ作成
  txbuf[0] = 0xA0 | servolocalID;
  txbuf[1] = sccode;

  // ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  // 受信データ確認
  if (retcode == RETCODE_OK) {
    if ((rxbuf[0] == (0x20 | servolocalID)) && (rxbuf[1] == sccode)) {
      // バッファチェック　ID, SC
      return rxbuf[2];
    } else {
      return RETCODE_ERROR_RETURNDATAWRONG;
    }
  } else {
    // error
    return retcode;
  }
}

// EEPROM以外のパラメータ書き込みコマンド
// 戻り値にRETCODE_OK（1の値）、またはエラーコード（負の値）が入ります。
// sccode は、
// SC_CODE_STRETCH, SC_CODE_SPEED, SC_CODE_CURRENT, SC_CODE_TEMPERATURE
// のどれかが入ります。
// SC_CODE_EEPROM はエラーとなります。EEPROM用の関数set_EEPROMを使ってください。
int IcsCommunication::write_Param(uint8_t servolocalID, uint8_t sccode,
                                  int val) {
  int txsize = 3;
  int rxsize = 3;
  uint8_t txbuf[3];
  uint8_t rxbuf[3];

  for (int a = 0; a < sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a = 0; a < sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }

  if ((sccode != SC_CODE_STRETCH) && (sccode != SC_CODE_SPEED) &&
      (sccode != SC_CODE_CURRENT) && (sccode != SC_CODE_TEMPERATURE)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }

  // 引数チェック
  if ((servolocalID < ID_MIN) || (servolocalID > ID_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }

  // 送信データ作成
  txbuf[0] = 0xC0 | servolocalID;
  txbuf[1] = sccode;
  txbuf[2] = val;

  // ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  // 受信データ確認
  if (retcode == RETCODE_OK) {
    if ((rxbuf[0] == (0x40 | servolocalID)) && (rxbuf[1] == sccode)) {
      // バッファチェック　ID, SC
      return RETCODE_OK;
    } else {
      return RETCODE_ERROR_RETURNDATAWRONG;
    }
  } else {
    // error
    return retcode;
  }
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
// パラメータ読み込み系

// 全て、戻り値にパラメータ値、またはエラーコード（負の値）が入ります。

int IcsCommunication::get_stretch(uint8_t servolocalID) {
  uint8_t sccode = SC_CODE_STRETCH;
  retval = read_Param(servolocalID, sccode);
  return retval;
}

int IcsCommunication::get_speed(uint8_t servolocalID) {
  uint8_t sccode = SC_CODE_SPEED;
  retval = read_Param(servolocalID, sccode);
  return retval;
}

int IcsCommunication::get_current(uint8_t servolocalID) {
  uint8_t sccode = SC_CODE_CURRENT;
  retval = read_Param(servolocalID, sccode);
  return retval;
}

int IcsCommunication::get_temperature(uint8_t servolocalID) {
  uint8_t sccode = SC_CODE_TEMPERATURE;
  retval = read_Param(servolocalID, sccode);
  return retval;
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
// パラメータ書き込み系

// 全て、戻り値にRETCODE_OK（1の値）、またはエラーコード（負の値）が入ります。

int IcsCommunication::set_stretch(uint8_t servolocalID, int val) {
  uint8_t sccode = SC_CODE_STRETCH;
  if ((val < 1) || (val > 127)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  retcode = write_Param(servolocalID, sccode, val);
  return retcode;
}

int IcsCommunication::set_speed(uint8_t servolocalID, int val) {
  uint8_t sccode = SC_CODE_SPEED;
  if ((val < 1) || (val > 127)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  retcode = write_Param(servolocalID, sccode, val);
  return retcode;
}

int IcsCommunication::set_currentlimit(uint8_t servolocalID, int val) {
  uint8_t sccode = SC_CODE_CURRENT;
  if ((val < 1) || (val > 63)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  retcode = write_Param(servolocalID, sccode, val);
  return retcode;
}

int IcsCommunication::set_temperaturelimit(uint8_t servolocalID, int val) {
  uint8_t sccode = SC_CODE_TEMPERATURE;
  if ((val < 1) || (val > 127)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  retcode = write_Param(servolocalID, sccode, val);
  return retcode;
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
// EEPROM系

// EEPROM読み取り（生バイト）
// 引数：　サーボＩＤ、受信バッファ
// 通常は、次のget_EEPROM関数を使ってください。
// 戻り値にRETCODE_OK（1の値）、またはエラーコード（負の値）が入ります。
int IcsCommunication::read_EEPROMraw(uint8_t servolocalID, uint8_t *rxbuf) {
  int txsize = 2;
  int rxsize = 66;
  uint8_t txbuf[2];

  for (int a = 0; a < sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }

  uint8_t sccode = SC_CODE_EEPROM;

  // 引数チェック
  if ((servolocalID < ID_MIN) || (servolocalID > ID_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }

  // 送信データ作成
  txbuf[0] = 0xA0 | servolocalID;
  txbuf[1] = sccode;

  // ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  // 受信データ確認
  if (retcode != RETCODE_OK) {
    return retcode;
  }

  // バッファチェック　ID, SC, 0x5A
  if ((rxbuf[0] != (0x20 | servolocalID)) || (rxbuf[1] != sccode) ||
      (rxbuf[2] != 0x5) || (rxbuf[3] != 0xA)) {
    // debugPrint("ID,SC,0x5A error.\r\n");
    return RETCODE_ERROR_RETURNDATAWRONG;
  }

  return RETCODE_OK;
}

// EEPROM読み取り
// 引数：　サーボＩＤ、ＥＥＰＲＯＭデータ構造体
// 戻り値にRETCODE_OK（1の値）、またはエラーコード（負の値）が入ります。
int IcsCommunication::get_EEPROM(uint8_t servolocalID, EEPROMdata *r_edata) {
  uint8_t rxbuf[66];

  retcode = read_EEPROMraw(servolocalID, rxbuf);

  // 受信データ確認
  if (retcode != RETCODE_OK) {
    // debugPrint("get_EEPROM inside read_EEPROMraw error. retcode: " +
    // String(retcode) + "\r\n");
    return retcode;
  }

  r_edata->stretch =
      combine_2byte(rxbuf[4], rxbuf[5]) / 2; // 2倍値で収納されてる
  r_edata->speed = combine_2byte(rxbuf[6], rxbuf[7]);
  r_edata->punch = combine_2byte(rxbuf[8], rxbuf[9]);
  r_edata->deadband = combine_2byte(rxbuf[10], rxbuf[11]);
  r_edata->dumping = combine_2byte(rxbuf[12], rxbuf[13]);
  r_edata->safetimer = combine_2byte(rxbuf[14], rxbuf[15]);

  r_edata->flag_slave = (rxbuf[16] >> 3) & 0b00000001;
  r_edata->flag_rotation = (rxbuf[16]) & 0b00000001;
  r_edata->flag_pwminh = (rxbuf[17] >> 3) & 0b00000001;
  r_edata->flag_free = (rxbuf[17] >> 1) & 0b00000001;
  r_edata->flag_reverse = (rxbuf[17]) & 0b00000001;

  r_edata->poslimithigh = (combine_2byte(rxbuf[18], rxbuf[19]) << 8) |
                          combine_2byte(rxbuf[20], rxbuf[21]);
  r_edata->poslimitlow = (combine_2byte(rxbuf[22], rxbuf[23]) << 8) |
                         combine_2byte(rxbuf[24], rxbuf[25]);

  int commflagval = combine_2byte(rxbuf[28], rxbuf[29]);
  if (commflagval == 0x00) {
    r_edata->commspeed = 1250000;
  } else if (commflagval == 0x01) {
    r_edata->commspeed = 625000;
  } else if (commflagval == 0x0A) {
    r_edata->commspeed = 115200;
  } else {
    r_edata->commspeed = EEPROM_NOTCHANGE;
  }

  r_edata->temperaturelimit = combine_2byte(rxbuf[30], rxbuf[31]);
  r_edata->currentlimit = combine_2byte(rxbuf[32], rxbuf[33]);
  r_edata->response = combine_2byte(rxbuf[52], rxbuf[53]);

  uint8_t tmpoffset = combine_2byte(rxbuf[54], rxbuf[55]);
  // ICSマネージャ挙動では、正負反対に収納されているのでそれに合わせる
  if ((tmpoffset >> 7) == 0b00000001) { // 負ビット
    r_edata->offset = ((~(tmpoffset)&0b01111111) + 1);
  } else { // 正ビット
    r_edata->offset = 0 - tmpoffset;
  }

  r_edata->ID = combine_2byte(rxbuf[58], rxbuf[59]);

  r_edata->charstretch1 =
      combine_2byte(rxbuf[60], rxbuf[61]) / 2; // 2倍値で収納されてる
  r_edata->charstretch2 =
      combine_2byte(rxbuf[62], rxbuf[63]) / 2; // 2倍値で収納されてる
  r_edata->charstretch3 =
      combine_2byte(rxbuf[64], rxbuf[65]) / 2; // 2倍値で収納されてる

  retcode = check_EEPROMdata(r_edata);

  if (retcode != RETCODE_OK) {
    // debugPrint("get_EEPROM inside check_EEPROMdata error. retcode: " +
    // String(retcode) + "\r\n");
    return retcode;
  }

  return RETCODE_OK;
}

//(beta test)
// EEPROM書き込み
// 引数：　サーボＩＤ、ＥＥＰＲＯＭデータ構造体
// 戻り値にRETCODE_OK（1の値）、またはエラーコード（負の値）が入ります。
// 書き込みコマンドは時間がややかかるようで、この関数内のみ別途タイムアウト時間を長くしています。
// https://twitter.com/devemin/status/1165875204419010561
int IcsCommunication::set_EEPROM(uint8_t servolocalID, EEPROMdata *w_edata) {
  // w_edata 内のデータのうち、EEPROM_NOTCHANGE でないものだけ書き込む
  int txsize = 66;   // 処理2つ目のEEPROM書き込みでのサイズ
  int rxsize = 2;    // 処理2つ目のEEPROM書き込みでのサイズ
  uint8_t txbuf[66]; // 処理2つ目のEEPROM書き込みでのサイズ
  uint8_t rxbuf
      [66]; // 処理1つ目のEEPROM読み取り、処理2つ目のEEPROM書き込み、の両方で使うバッファなのでsize:66としてる

  uint8_t sccode = SC_CODE_EEPROM;

  for (int a = 0; a < sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a = 0; a < sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }

  // 引数チェック
  if ((servolocalID < ID_MIN) || (servolocalID > ID_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  // 変更禁止部分以外の、設定値の正当性チェック
  retcode = check_EEPROMdata(w_edata);
  if (retcode != RETCODE_OK) {
    return RETCODE_ERROR_EEPROMDATAWRONG;
  }

  //////////////////////////////////////////////////////////////////////
  // 関数内1つ目の処理　EEPROM読み取り
  // 書き込む前に必ず最新のEEPROM読み取り！
  retcode = read_EEPROMraw(servolocalID, rxbuf);
  if (retcode != RETCODE_OK) {
    return retcode;
  }

  // EEPROMデータ先頭の0x5Aチェック
  int checkdata = combine_2byte(rxbuf[2], rxbuf[3]);
  if (checkdata != 0x5A) {
    return RETCODE_ERROR_EEPROMDATAWRONG;
  }

  //////////////////////////////////////////////////////////////////////
  // 関数内2つ目の処理　EEPROM書き込み
  //  EEPROMに書き込むデータの準備
  for (int a = 0; a < txsize; a++) {
    txbuf[a] =
        rxbuf[a]; // 元々の値を全てにコピー。この後、必要な項目のみ変更する
  }

  // 送信データ先頭の作成
  txbuf[0] = 0xC0 | servolocalID;
  txbuf[1] = sccode;

  // 送信バッファtxbuf に、引数で受け取ったEEPROMデータの変更部分のみコピーする
  if (w_edata->stretch != EEPROM_NOTCHANGE) {
    txbuf[4] =
        (uint8_t)(w_edata->stretch * 2) >> 4; // 2倍値で収納されてるので処理
    txbuf[5] = (uint8_t)(w_edata->stretch * 2) & 0b0000000000001111;
  }

  if (w_edata->speed != EEPROM_NOTCHANGE) {
    txbuf[6] = (uint8_t)(w_edata->speed) >> 4;
    txbuf[7] = (uint8_t)(w_edata->speed) & 0b0000000000001111;
  }
  if (w_edata->punch != EEPROM_NOTCHANGE) {
    txbuf[8] = (uint8_t)(w_edata->punch) >> 4;
    txbuf[9] = (uint8_t)(w_edata->punch) & 0b0000000000001111;
  }
  if (w_edata->deadband != EEPROM_NOTCHANGE) {
    txbuf[10] = (uint8_t)(w_edata->deadband) >> 4;
    txbuf[11] = (uint8_t)(w_edata->deadband) & 0b0000000000001111;
  }
  if (w_edata->dumping != EEPROM_NOTCHANGE) {
    txbuf[12] = (uint8_t)(w_edata->dumping) >> 4;
    txbuf[13] = (uint8_t)(w_edata->dumping) & 0b0000000000001111;
  }
  if (w_edata->safetimer != EEPROM_NOTCHANGE) {
    txbuf[14] = (uint8_t)(w_edata->safetimer) >> 4;
    txbuf[15] = (uint8_t)(w_edata->safetimer) & 0b0000000000001111;
  }

  if (w_edata->flag_slave != EEPROM_NOTCHANGE) {
    if (w_edata->flag_slave == 1) {
      txbuf[16] = txbuf[16] | 0b00001000;
    }
    if (w_edata->flag_slave == 0) {
      txbuf[16] = txbuf[16] & 0b11110111;
    }
  }
  if (w_edata->flag_rotation != EEPROM_NOTCHANGE) {
    if (w_edata->flag_rotation == 1) {
      txbuf[16] = txbuf[16] | 0b00000001;
    }
    if (w_edata->flag_rotation == 0) {
      txbuf[16] = txbuf[16] & 0b11111110;
    }
  }
  if (w_edata->flag_pwminh != EEPROM_NOTCHANGE) {
    if (w_edata->flag_pwminh == 1) {
      txbuf[17] = txbuf[17] | 0b00001000;
    }
    if (w_edata->flag_pwminh == 0) {
      txbuf[17] = txbuf[17] & 0b11110111;
    }
  }
  if (w_edata->flag_free != EEPROM_NOTCHANGE) {
    // freeフラグは参照のみとのこと。ここを書き込む事は、マニュアル等に説明は無いので挙動不明
    // if (w_edata->flag_free == 1) { txbuf[17] = txbuf[17] | 0b00000010 ; }
    // if (w_edata->flag_free == 0) { txbuf[17] = txbuf[17] & 0b11111101 ; }
  }
  if (w_edata->flag_reverse != EEPROM_NOTCHANGE) {
    if (w_edata->flag_reverse == 1) {
      txbuf[17] = txbuf[17] | 0b00000001;
    }
    if (w_edata->flag_reverse == 0) {
      txbuf[17] = txbuf[17] & 0b11111110;
    }
  }

  if (w_edata->poslimithigh != EEPROM_NOTCHANGE) {
    txbuf[18] =
        (w_edata->poslimithigh >> 12) & 0b00000000000000000000000000001111;
    txbuf[19] =
        (w_edata->poslimithigh >> 8) & 0b00000000000000000000000000001111;
    txbuf[20] =
        (w_edata->poslimithigh >> 4) & 0b00000000000000000000000000001111;
    txbuf[21] = (w_edata->poslimithigh) & 0b00000000000000000000000000001111;
  }

  if (w_edata->poslimitlow != EEPROM_NOTCHANGE) {
    txbuf[22] =
        (w_edata->poslimitlow >> 12) & 0b00000000000000000000000000001111;
    txbuf[23] =
        (w_edata->poslimitlow >> 8) & 0b00000000000000000000000000001111;
    txbuf[24] =
        (w_edata->poslimitlow >> 4) & 0b00000000000000000000000000001111;
    txbuf[25] = (w_edata->poslimitlow) & 0b00000000000000000000000000001111;
  }

  if (w_edata->commspeed != EEPROM_NOTCHANGE) {
    if (w_edata->commspeed == 115200) {
      txbuf[28] = 0x0;
      txbuf[29] = 0xA;
    } else if (w_edata->commspeed == 625000) {
      txbuf[28] = 0x0;
      txbuf[29] = 0x1;
    } else if (w_edata->commspeed == 1250000) {
      txbuf[28] = 0x0;
      txbuf[29] = 0x0;
    }
  }

  if (w_edata->temperaturelimit != EEPROM_NOTCHANGE) {
    txbuf[30] = (uint8_t)(w_edata->temperaturelimit) >> 4;
    txbuf[31] = (uint8_t)(w_edata->temperaturelimit) & 0b0000000000001111;
  }
  if (w_edata->currentlimit != EEPROM_NOTCHANGE) {
    txbuf[32] = (uint8_t)(w_edata->currentlimit) >> 4;
    txbuf[33] = (uint8_t)(w_edata->currentlimit) & 0b0000000000001111;
  }
  if (w_edata->response != EEPROM_NOTCHANGE) {
    txbuf[52] = (uint8_t)(w_edata->response) >> 4;
    txbuf[53] = (uint8_t)(w_edata->response) & 0b0000000000001111;
  }
  if (w_edata->offset !=
      EEPROM_NOTCHANGE) { // ICSマネージャ挙動では、正負反対に収納されているのでそれに合わせる
    uint8_t tmpoffset = ~(w_edata->offset) + 1;
    txbuf[54] = (uint8_t)(tmpoffset) >> 4;
    txbuf[55] = (uint8_t)(tmpoffset)&0b0000000000001111;
  }
  if (w_edata->ID != EEPROM_NOTCHANGE) {
    // IDは専用のコマンドがあるので、本来はそちらで行う。こちらから書き込んでも私の環境ではID書き換えできているが、リファレンスマニュアルには記載無し
    txbuf[58] = (uint8_t)(w_edata->ID) >> 4;
    txbuf[59] = (uint8_t)(w_edata->ID) & 0b0000000000001111;
  }

  if (w_edata->charstretch1 != EEPROM_NOTCHANGE) {
    txbuf[60] = (uint8_t)(w_edata->charstretch1 * 2) >>
                4; // 2倍値で収納されてるので処理
    txbuf[61] = (uint8_t)(w_edata->charstretch1 * 2) & 0b0000000000001111;
  }
  if (w_edata->charstretch2 != EEPROM_NOTCHANGE) {
    txbuf[62] = (uint8_t)(w_edata->charstretch2 * 2) >>
                4; // 2倍値で収納されてるので処理
    txbuf[63] = (uint8_t)(w_edata->charstretch2 * 2) & 0b0000000000001111;
  }
  if (w_edata->charstretch1 != EEPROM_NOTCHANGE) {
    txbuf[64] = (uint8_t)(w_edata->charstretch3 * 2) >>
                4; // 2倍値で収納されてるので処理
    txbuf[65] = (uint8_t)(w_edata->charstretch3 * 2) & 0b0000000000001111;
  }

  // ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  if (retcode != RETCODE_OK) {
    // debugPrint("error: " + String(retcode));
    return retcode;
  }

  // 受信データ確認
  // バッファチェック　ID, SC
  if ((rxbuf[0] != (0x40 | servolocalID)) || (rxbuf[1] != sccode)) {
    // debugPrint("return data is incorrect.\r\n");
    return RETCODE_ERROR_RETURNDATAWRONG;
  }

  return RETCODE_OK;
}

// データがEEPROM用として正当かどうかを確認する関数
// 元々のサーボ内の変更禁止バイト部分の読み込み等は別で行っている
// 内容がEEPROM_NOTCHANGE（初期状態）だと、書き込み不適と判断しエラーを返す
int IcsCommunication::check_EEPROMdata(EEPROMdata *edata) {
  int checkdata;
  int tmpretcode = RETCODE_OK;

  checkdata = edata->stretch;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < 1) || (checkdata > 127)) {
    printf("EEPROMdata error: stretch\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  checkdata = edata->speed;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < 0x01) ||
      (checkdata > 0x7F)) {
    printf("EEPROMdata error: speed\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  checkdata = edata->punch;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < 0x00) ||
      (checkdata > 0x0A)) {
    printf("EEPROMdata error: punch\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  checkdata = edata->deadband;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < 0x00) ||
      (checkdata > 0x10)) {
    printf("EEPROMdata error: deadband\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  checkdata = edata->dumping;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < 0x01) ||
      (checkdata > 0xFF)) {
    printf("EEPROMdata error: dumping\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  checkdata = edata->safetimer;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < 0x01) ||
      (checkdata > 0xFF)) {
    printf("EEPROMdata error: safetimer\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }

  // フラグ
  if ((edata->flag_slave != EEPROM_NOTCHANGE) && (edata->flag_slave != 0) &&
      (edata->flag_slave != 1)) {
    printf("EEPROMdata error: flag_slave\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  if ((edata->flag_rotation != EEPROM_NOTCHANGE) &&
      (edata->flag_rotation != 0) && (edata->flag_rotation != 1)) {
    printf("EEPROMdata error: flag_rotation\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  if ((edata->flag_pwminh != EEPROM_NOTCHANGE) && (edata->flag_pwminh != 0) &&
      (edata->flag_pwminh != 1)) {
    printf("EEPROMdata error: flag_pwminh\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  if ((edata->flag_free != EEPROM_NOTCHANGE) && (edata->flag_free != 0) &&
      (edata->flag_free != 1)) {
    printf("EEPROMdata error: flag_free\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  if ((edata->flag_reverse != EEPROM_NOTCHANGE) && (edata->flag_reverse != 0) &&
      (edata->flag_reverse != 1)) {
    printf("EEPROMdata error: flag_reverse\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }

  checkdata = edata->poslimithigh;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < 8000) ||
      (checkdata > POS_MAX)) {
    printf("EEPROMdata error: poslimithigh\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  checkdata = edata->poslimitlow;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < POS_MIN) ||
      (checkdata > 7000)) {
    printf("EEPROMdata error: poslimitlow\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }

  if ((edata->commspeed != EEPROM_NOTCHANGE) && (edata->commspeed != 115200) &&
      (edata->commspeed != 625000) && (edata->commspeed != 1250000)) {
    printf("EEPROMdata error: commspeed\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }

  checkdata = edata->temperaturelimit;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < 0x01) ||
      (checkdata > 0x7F)) {
    printf("EEPROMdata error: temperaturelimit\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  checkdata = edata->currentlimit;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < 0x01) ||
      (checkdata > 0x3F)) {
    printf("EEPROMdata error: currentlimit\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  checkdata = edata->response;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < 0x01) ||
      (checkdata > 0x05)) {
    printf("EEPROMdata error: response\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  checkdata = edata->offset;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < -128) ||
      (checkdata > 127)) {
    printf("EEPROMdata error: offset\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  checkdata = edata->ID;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < 0x00) ||
      (checkdata > 0x1F)) {
    printf("EEPROMdata error: ID\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  checkdata = edata->charstretch1;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < 1) || (checkdata > 127)) {
    printf("EEPROMdata error: charstretch1\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  checkdata = edata->charstretch2;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < 1) || (checkdata > 127)) {
    printf("EEPROMdata error: charstretch2\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }
  checkdata = edata->charstretch3;
  if ((checkdata != EEPROM_NOTCHANGE) && (checkdata < 1) || (checkdata > 127)) {
    printf("EEPROMdata error: charstretch3\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;
  }

  return tmpretcode;
}

// 上位下位4bitずつに分かれた2byteを、1byteにして取得する関数
uint8_t IcsCommunication::combine_2byte(uint8_t a, uint8_t b) {
  return (uint8_t)((a << 4) | b);
}

// バッファ内のEEPROM生バイトデータを表示する関数
void IcsCommunication::show_EEPROMbuffer(uint8_t *checkbuf) {}

// EEPROMデータの表示関数
void IcsCommunication::show_EEPROMdata(EEPROMdata *edata) {
  /*
  debugPrint("===================================================\r\n");
  debugPrint("----EEPROM data------------------------------------\r\n");

  debugPrint("stretch:          " + String(edata->stretch) + "[0x" +
             String(edata->stretch, HEX) + "] (1-127)\r\n");
  debugPrint("speed:            " + String(edata->speed) + "[0x" +
             String(edata->speed, HEX) + "] (1-127)\r\n");
  debugPrint("punch:            " + String(edata->punch) + "[0x" +
             String(edata->punch, HEX) + "] (0-10)\r\n");
  debugPrint("deadband:         " + String(edata->deadband) + "[0x" +
             String(edata->deadband, HEX) + "] (0-5)\r\n");
  debugPrint("dumping:          " + String(edata->dumping) + "[0x" +
             String(edata->dumping, HEX) + "] (1-255)\r\n");
  debugPrint("safetimer:        " + String(edata->safetimer) + "[0x" +
             String(edata->safetimer, HEX) + "] (10-255)\r\n");

  debugPrint("flag_slave:       " + String(edata->flag_slave) + " (0/1)\r\n");
  debugPrint("flag_rotation:    " + String(edata->flag_rotation) +
             " (0/1)\r\n");
  debugPrint("flag_pwminh:      " + String(edata->flag_pwminh) + " (0/1)\r\n");
  debugPrint("flag_free:        " + String(edata->flag_free) + " (0/1)\r\n");
  debugPrint("flag_reverse:     " + String(edata->flag_reverse) + " (0/1)\r\n");

  debugPrint("poslimithigh:     " + String(edata->poslimithigh) + "[0x" +
             String(edata->poslimithigh, HEX) + "] (8000-11500)\r\n");
  debugPrint("poslimitlow:      " + String(edata->poslimitlow) + "[0x" +
             String(edata->poslimitlow, HEX) + "] (3500-7000)\r\n");
  debugPrint("commspeed:        " + String(edata->commspeed) +
             " (115200/625000/1250000)\r\n");
  debugPrint("temperaturelimit: " + String(edata->temperaturelimit) + "[0x" +
             String(edata->temperaturelimit, HEX) + "] (1-127)\r\n");
  debugPrint("currentlimit:     " + String(edata->currentlimit) + "[0x" +
             String(edata->currentlimit, HEX) + "] (1-63)\r\n");
  debugPrint("response:         " + String(edata->response) + "[0x" +
             String(edata->response, HEX) + "] (1-5)\r\n");
  debugPrint("offset:           " + String(edata->offset) + "[0x" +
             String(edata->offset, HEX) + "] (-127-127)\r\n");
  debugPrint("ID:               " + String(edata->ID) + "[0x" +
             String(edata->ID, HEX) + "] (0-31)\r\n");
  debugPrint("charstretch1:     " + String(edata->charstretch1) + "[0x" +
             String(edata->charstretch1, HEX) + "] (1-127)\r\n");
  debugPrint("charstretch2:     " + String(edata->charstretch2) + "[0x" +
             String(edata->charstretch2, HEX) + "] (1-127)\r\n");
  debugPrint("charstretch3:     " + String(edata->charstretch3) + "[0x" +
             String(edata->charstretch3, HEX) + "] (1-127)\r\n");
  debugPrint("\r\n");
  debugPrint("    -4096: this mean NO_CHANGE.                    \r\n");
  debugPrint("---------------------------------------------------\r\n");
  debugPrint("===================================================\r\n");
  debugPrint("\r\n");
  */
}

// サーボの存在を確認
//  EEPROMからID読み込みを繰り返して、判断する。
//  EEPROM読み取りは各種データチェックが入ってるので、その処理を複数回乗り越えてれば、
// おそらくそのIDは本物だろう、という考え。
bool IcsCommunication::IsServoAlive(uint8_t servolocalID) {
  EEPROMdata tmped;
  int repnum = 10; // チェック回数

  for (int a = 0; a < repnum; a++) {
    retcode = get_EEPROM(servolocalID, &tmped);
    if ((retcode != RETCODE_OK) || (servolocalID != tmped.ID)) {
      // debugPrint(String(servolocalID) + "," + String(tmped.ID) + "\r\n");
      return false;
    }
  }

  return true;
}

// このID読み込みコマンドは、標準のものだが、ホストーサーボを１対１で接続して使用するもの。
// もし複数接続していた場合は、返信IDは不正なデータとなるので信用性がない。
int IcsCommunication::get_ID() {
  int txsize = 4; // この関数固有の、送信データバイト数
  int rxsize = 1; // この関数固有の、受信データバイト数
  uint8_t txbuf[4];
  uint8_t rxbuf[66]; // 重複IDチェックのため多バイト確保

  for (int a = 0; a < sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a = 0; a < sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }

  // 送信データ作成
  txbuf[0] = 0xFF;
  txbuf[1] = 0x00;
  txbuf[2] = 0x00;
  txbuf[3] = 0x00;

  // ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  // 受信データ確認
  if (retcode == RETCODE_OK) {
    if ((rxbuf[0] >> 5) == 0b00000111) {
      retval = (rxbuf[0] & 0b00011111);
      return retval;
    } else {
      return RETCODE_ERROR_IDWRONG;
    }
  } else {
    // error
    return retcode;
  }
}

// このID書き込みコマンドは、標準のものだが、ホストーサーボを１対１で接続して使用するもの。
// もし複数接続していた場合は、全てのサーボが同じIDに書き換わってしまうので注意。
// もし該当サーボのIDが既にわかってるのであれば、こちらを使わずEEPROM書き替え関数の方でID指定して書き替えられます。
//  20μsec 程で返信コマンドは来るが、再度書き込みする場合は、
//  EEPROM書き込みと同等時間待たないとエラーとなり返信来ないので、別途delay()が必要
//  KRS-4031HV で500μsec 程
//  https://twitter.com/devemin/status/1165865232318775296
int IcsCommunication::set_ID(uint8_t servolocalID) {
  int txsize = 4; // この関数固有の、送信データバイト数
  int rxsize = 1; // この関数固有の、受信データバイト数
  uint8_t txbuf[4];
  uint8_t rxbuf[66]; // 重複IDチェックのため多バイト確保

  for (int a = 0; a < sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a = 0; a < sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }

  // 送信データ作成
  txbuf[0] = 0xE0 | servolocalID;
  txbuf[1] = 0x01;
  txbuf[2] = 0x01;
  txbuf[3] = 0x01;

  // ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  // 受信データ確認
  if (retcode == RETCODE_OK) {
    if (((rxbuf[0] >> 5) == 0b00000111) &&
        ((rxbuf[0] & 0b00011111) == servolocalID)) {
      retval = (rxbuf[0] & 0b00011111);
      return retval;
    } else {
      return RETCODE_ERROR_IDWRONG;
    }
  } else {
    // error
    return retcode;
  }
}
