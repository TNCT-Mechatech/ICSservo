#ifndef _ICS_COMMUNICATION_HPP_
#define _ICS_COMMUNICATION_HPP_

#include "mbed.h"
#include "stdint.h"

// 定数
static const int ID_MIN = 0;
static const int ID_MAX = 31;
static const int ID_NUM = 32;

static const int EEPROM_NOTCHANGE = -4096;

static const int RETCODE_OK = 1;
static const int RETCODE_ERROR_ICSREAD =
    -1001;                                       // 読み取りエラー、おそらくタイムアウト
static const int RETCODE_ERROR_ICSWRITE = -1002; // 返信コマンド内容エラー
static const int RETCODE_ERROR_IDWRONG = -1003;
static const int RETCODE_ERROR_OPTIONWRONG = -1004;
static const int RETCODE_ERROR_RETURNDATAWRONG = -1005;
static const int RETCODE_ERROR_EEPROMDATAWRONG = -1006;

// IcsCommunicationクラスで用いるEEPROMデータ用構造体
struct EEPROMdata
{
  int stretch = EEPROM_NOTCHANGE;
  int speed = EEPROM_NOTCHANGE;
  int punch = EEPROM_NOTCHANGE;
  int deadband = EEPROM_NOTCHANGE;
  int dumping = EEPROM_NOTCHANGE;
  int safetimer = EEPROM_NOTCHANGE;

  int flag_slave = EEPROM_NOTCHANGE;
  int flag_rotation = EEPROM_NOTCHANGE;
  int flag_pwminh = EEPROM_NOTCHANGE;
  // ここを変更しても書き込まない（ICSマネージャによると、読み出し参照のみとのこと）
  int flag_free = EEPROM_NOTCHANGE;
  int flag_reverse = EEPROM_NOTCHANGE;

  int poslimithigh = EEPROM_NOTCHANGE;
  int poslimitlow = EEPROM_NOTCHANGE;
  int commspeed = EEPROM_NOTCHANGE;
  int temperaturelimit = EEPROM_NOTCHANGE;
  int currentlimit = EEPROM_NOTCHANGE;
  int response = EEPROM_NOTCHANGE;
  int offset = EEPROM_NOTCHANGE;
  int ID = EEPROM_NOTCHANGE;
  int charstretch1 = EEPROM_NOTCHANGE;
  int charstretch2 = EEPROM_NOTCHANGE;
  int charstretch3 = EEPROM_NOTCHANGE;
};

class IcsCommunication
{
  // パブリック変数
public:
  // プライベート変数
private:
  static const int POS_MIN = 3500;
  static const int POS_MAX = 11500;
  static const int SC_CODE_EEPROM = 0x00;
  static const int SC_CODE_STRETCH = 0x01;
  static const int SC_CODE_SPEED = 0x02;
  static const int SC_CODE_CURRENT = 0x03;
  static const int SC_CODE_TEMPERATURE = 0x04;

  UnbufferedSerial *refSer;
  DigitalOut icsPin;
  uint32_t baudrate = 115200;
  bool initHigh;

  int retcode;
  int retval;

  // パブリック関数
public:
  IcsCommunication(UnbufferedSerial &ser, PinName icsPinName);

  // 初期化
  bool begin(uint32_t brate = 115200, bool initFlag = true);
  void change_baudrate(uint32_t brate);

  // サーボ移動関係
  int set_position(uint8_t servolocalID,
                   int val); // サーボ動作する　　　　　位置が戻り値として来る
  int set_position_weak(
      uint8_t servolocalID); // サーボ脱力する　　　　　位置が戻り値として来る
  int set_position_weakandkeep(
      uint8_t servolocalID); // サーボ脱力後即動作する　位置が戻り値として来る

  // パラメータ関数系　電源切ると設定消える
  int get_stretch(uint8_t servolocalID);
  int get_speed(uint8_t servolocalID);
  int get_current(uint8_t servolocalID);
  int get_temperature(uint8_t servolocalID);

  int set_stretch(uint8_t servolocalID, int val);
  int set_speed(uint8_t servolocalID, int val);
  int set_currentlimit(uint8_t servolocalID, int val);
  int set_temperaturelimit(uint8_t servolocalID, int val);

  // EEPROM系　電源切っても設定消えない
  int get_EEPROM(uint8_t servolocalID, EEPROMdata *r_edata);
  int set_EEPROM(uint8_t servolocalID, EEPROMdata *w_edata);

  void show_EEPROMbuffer(uint8_t *checkbuf);
  void show_EEPROMdata(EEPROMdata *edata);

  // 標準のＩＤコマンド　ホストとサーボ１対１で使用する必要あり
  int get_ID();
  int set_ID(uint8_t servolocalID);

  bool IsServoAlive(uint8_t servolocalID);

  // プライベート関数
private:
  int transceive(uint8_t *txbuf, uint8_t *rxbuf, uint8_t txsize,
                 uint8_t rxsize);
  int read_Param(uint8_t servolocalID, uint8_t sccode);
  int write_Param(uint8_t servolocalID, uint8_t sccode, int val);
  int read_EEPROMraw(uint8_t servolocalID, uint8_t *rxbuf);
  int check_EEPROMdata(EEPROMdata *edata);
  uint8_t combine_2byte(uint8_t a, uint8_t b);
};

#endif