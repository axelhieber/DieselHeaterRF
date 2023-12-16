#ifndef DieselHeaterRF_h
#define DieselHeaterRF_h

#include <Arduino.h>

#ifndef HEATER_SCK_PIN
    #define HEATER_SCK_PIN   18
#endif
#ifndef HEATER_MISO_PIN
    #define HEATER_MISO_PIN   19
#endif
#ifndef HEATER_MOSI_PIN
    #define HEATER_MOSI_PIN   23
#endif
#ifndef HEATER_SS_PIN
    #define HEATER_SS_PIN   5
#endif
 #ifndef HEATER_GDO2_PIN
    #define HEATER_GDO2_PIN   4
#endif

#define HEATER_CMD_WAKEUP 0x23
#define HEATER_CMD_MODE   0x24
#define HEATER_CMD_POWER  0x2b
#define HEATER_CMD_UP     0x3c
#define HEATER_CMD_DOWN   0x3e

#define HEATER_STATE_OFF            0x00
#define HEATER_STATE_STARTUP        0x01
#define HEATER_STATE_WARMING        0x02
#define HEATER_STATE_WARMING_WAIT   0x03
#define HEATER_STATE_PRE_RUN        0x04
#define HEATER_STATE_RUNNING        0x05
#define HEATER_STATE_SHUTDOWN       0x06
#define HEATER_STATE_SHUTTING_DOWN  0x07
#define HEATER_STATE_COOLING        0x08

#define HEATER_TX_REPEAT    10 // Number of times to re-transmit command packets
#define HEATER_RX_TIMEOUT   5000


enum class CC1101 { 
    IOCFG2          =0x00,
    IOCFG1          =0x01,
    IOCFG0          =0x02,  
    FIFOTHR         =0x03,
    SYNC1           =0x04,
    SYNC0           =0x05,
    PKTLEN          =0x06,
    PKTCTRL1        =0x07,
    PKTCTRL0        =0x08,
    ADDR            =0x09,
    CHANNR          =0x0A,
    FSCTRL1         =0x0B,
    FSCTRL0         =0x0C,
    FREQ2           =0x0D,  
    FREQ1           =0x0E,
    FREQ0           =0x0F,
    MDMCFG4         =0x10,
    MDMCFG3         =0x11,
    MDMCFG2         =0x12,
    MDMCFG1         =0x13,
    MDMCFG0         =0x14,
    DEVIATN         =0x15,
    MCSM2           =0x16,
    MCSM1           =0x17,
    MCSM0           =0x18,
    FOCCFG          =0x19,
    BSCFG           =0x1A,
    AGCCTRL2        =0x1B,
    AGCCTRL1        =0x1C,
    AGCCTRL0        =0x1D,
    WOREVT1         =0x1E,
    WOREVT0         =0x1F,
    WORCTRL         =0x20,
    FREND1          =0x21,
    FREND0          =0x22,
    FSCAL3          =0x23,
    FSCAL2          =0x24,
    FSCAL1          =0x25,
    FSCAL0          =0x26,
    RCCTRL1         =0x27,
    RCCTRL0         =0x28,
    FSTEST          =0x29,
    PTEST           =0x2A,
    AGCTEST         =0x2B,
    TEST2           =0x2C,
    TEST1           =0x2D,
    TEST0           =0x2E,

    // Status Register Details READ ONLY - Burst
    PARTNUM         =0x30, 
    VERSION         =0x31,
    FREQEST         =0x32,
    LQI             =0x32,
    RSSI            =0x34,
    MARCSTATE       =0x35,
    WORTIME1        =0x36,
    WORTIME0        =0x37,
    PKTSTATUS       =0x38,
    VCO_VC_DAC      =0x39,
    TXBYTES         =0x3A,
    RXBYTES         =0x3B,
    RCCTRL1_STATUS  =0x3C,
    RCCTRL0_STATUS  =0x3D

    // Command Strobes R/W - Single Byte
    SRES            =0x30, 
    SFSTXON         =0x31,
    SXOFF           =0x32,
    SCAL            =0x32,
    SRX             =0x34,
    STX             =0x35,
    SIDLE           =0x36,
    //              =0x37,
    SWOR            =0x38,
    SPWD            =0x39,
    SFRX            =0x3A,
    SFTX            =0x3B,
    SWORRST         =0x3C,
    SNOP            =0x3D
    };


typedef struct {
  uint8_t state       = 0;
  uint8_t power       = 0;
  float voltage       = 0;
  int8_t ambientTemp  = 0;
  uint8_t caseTemp    = 0;
  int8_t setpoint     = 0;
  uint8_t autoMode    = 0;
  float pumpFreq      = 0;
  int16_t rssi        = 0;
} heater_state_t;

typedef struct{       // Reset      R/W     Description 
  uint8_t partnum =0; // 0          R       Chip part number
  uint8_t version =0; // 20         R       Chip version number. Subject to change without notice.
  uint8_t rssi    =0  //            R       Received signal strength indicator 
  /* FREQOFF_EST
     LQI
     MARCSTATE 
     WORTIME1 
     WORTIME0 
     PKTSTATUS 
     VCO_VC_DAC
     TXBYTES
     RXBYTES */

}
cc1101 CC1101

class DieselHeaterRF
{

    uint8_t _pinSck;
    uint8_t _pinMiso;
    uint8_t _pinMosi;
    uint8_t _pinSs;
    uint8_t _pinGdo2;

    uint32_t _heaterAddr = 0;
    uint8_t _packetSeq = 0;

    void initRadio();

    void txBurst(uint8_t len, char *bytes);
    void txFlush();

    void rx(uint8_t len, char *bytes);
    void rxFlush();
    void rxEnable();  

    uint8_t writeReg(uint8_t addr, uint8_t val);
    void writeBurst(uint8_t addr, uint8_t len, char *bytes);
    void writeStrobe(uint8_t addr);

    void spiStart(void);
    void spiEnd(void);

    bool receivePacket(char *bytes, uint16_t timeout);
    uint32_t parseAddress(char *buf);

    uint16_t crc16_2(char *buf, int len);


    public:

        DieselHeaterRF():DieselHeaterRF(HEATER_SCK_PIN,HEATER_MISO_PIN,HEATER_MOSI_PIN,HEATER_SS_PIN,HEATER_GDO2_PIN) {}


        DieselHeaterRF(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss, uint8_t gdo2):
                         _pinSck {sck}
                         _pinMiso{miso}
                         _pinMosi{mosi}
                         _pinSs  {ss}
                         _pinGdo2{gdo2}{}

        void begin();
        void begin(uint32_t heaterAddr);

        void setAddress(uint32_t heaterAddr);
        bool getState(heater_state_t *state);
        bool getState(heater_state_t *state, uint32_t timeout);
        bool getState(uint8_t *state, uint8_t *power, float *voltage, int8_t *ambientTemp, uint8_t *caseTemp, int8_t *setpoint, float *pumpFreq, uint8_t *autoMode, int16_t *rssi, uint32_t timeout);
        void sendCommand(uint8_t cmd);
        void sendCommand(uint8_t cmd, uint32_t addr);
        void sendCommand(uint8_t cmd, uint32_t addr, uint8_t numTransmits);
        uint32_t findAddress(uint16_t timeout);


};

#endif
