#include "Arduino.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "DS3231.h"
#include "time.h"

#define WT_CONTROL_NONE 0
#define WT_CONTROL_CONFIG 1000
#define WT_CONTROL_CAL_DRY 1001
#define WT_CONTROL_CAL_LOW 1002
#define WT_CONTROL_CAL_HIGH 1003
#define WT_SET_RTC 1004
#define WT_DEPLOY 1005
#define WT_DEBUG_VAlUES 1006 // will print out data that's logged
#define WT_CLEAR_MODES 1007 // clear config and debug without resetting

class WaterBear_Control
{
  private:
    /*Stream *mySerial;
    HardwareSerial * hs;
    Adafruit_BluefruitLE_UART * ble;
    */

  public:

    static int state;

    static int processControlCommands(Stream * myStream);
    static int processControlCommands(HardwareSerial &port);
    static int processControlCommands(Adafruit_BluefruitLE_UART &ble);
    static int processControlCommands(Adafruit_BluefruitLE_SPI &ble);

    static void * getLastPayload();

    static bool ready(Stream * myStream);
    static bool ready(HardwareSerial &port);
    static bool ready(Adafruit_BluefruitLE_UART &ble);
    static bool ready(Adafruit_BluefruitLE_SPI &ble);

    static void blink(int times, int duration);
    static time_t timestamp();
    static void setTime(time_t toSet);
    static void t_t2ts(time_t epochTS, char *humanTime); //Epoch TS to yyyy/mm/dd dow hh:mm:ss zzz

#if SOFTWARE_SERIAL_AVAILABLE
    static void processControlCommands(SoftwareSerial &port);
#endif
};
