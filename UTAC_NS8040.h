// file UTAC_NS8040.h
#include <HardwareSerial.h>
#include <Controllino.h>
#include "ModbusRtu.h"


// This MACRO defines Modbus master address.
// For any Modbus slave devices are reserved addresses in the range from 1 to 247.
// Important note only address 0 is reserved for a Modbus master device!
#define MasterModbusAdd  0
#define SlaveModbusAdd  1

// This MACRO defines number of the comport that is used for RS 485 interface.
// For MAXI and MEGA RS485 is reserved UART Serial3.
#define RS485Serial   3

// The object ControllinoModbuSlave of the class Modbus is initialized with three parameters.
// The first parametr specifies the address of the Modbus slave device.
// The second parameter specifies type of the interface used for communication between devices - in this sketch is used RS485.
// The third parameter can be any number. During the initialization of the object this parameter has no effect.
Modbus ControllinoModbusMaster(MasterModbusAdd, RS485Serial, 0);

//
// This uint16 array specified internal registers in the Modbus slave device.
// Each Modbus device has particular internal registers that are available for the Modbus master.
// In this example sketch internal registers are defined as follows:
// ModbusSlaveRegisters[0] and [1] Sv and Pv for rs485 id 1
// ModbusSlaveRegisters[2] and [3] Sv and Pv for rs485 id 2
// ModbusSlaveRegisters[4] and [5] Sv and Pv for rs485 id 3
// ModbusSlaveRegisters[6] and [7] Sv and Pv for rs485 id 4
//
// ModbusSlaveRegisters[8] and [9] return from write Sv for id 1
// ModbusSlaveRegisters[10] and [11] return from write Sv for id 2
// ModbusSlaveRegisters[12] and [13] return from write Sv for id 3
// ModbusSlaveRegisters[14] and [15] return from write Sv for id 4
//
//
// explanation for the count of ModbusSlaveRegisters
// - the last 2 ModbusSlaveRegisters are used by the 'set' call for all Ids as this
//    is temporary data from the set call response - i.e. don't care about the return
//    data, so all 'sets' can use the same ModbusSlaveRegisters
uint16_t ModbusSlaveRegisters[16];

// set values is the Sv for the Id from the input RS232 command
uint16_t  set_values[4]; 
bool      have_initial_sv = false;

// This is an structe which contains a query fo a slave device
#define MAX_SLAVE_IDS   4
modbus_t ModbusQuery[MAX_SLAVE_IDS];  // do the get SvPv command
modbus_t ModbusSet[MAX_SLAVE_IDS];    // do the Sv command

// SV_PV_DELTA is the absolute value (SV - PV) to be acheived to close the relay
#define SV_PV_DELTA       30

// SV versus PV history - used to determine whether to open or close relays
#define RELAY_HISTORY_DEPTH   3
int relay1Array[RELAY_HISTORY_DEPTH] = {0, 0, 0};
int relay2Array[RELAY_HISTORY_DEPTH] = {0, 0, 0};
int relay3Array[RELAY_HISTORY_DEPTH] = {0, 0, 0};
int relay4Array[RELAY_HISTORY_DEPTH] = {0, 0, 0};


typedef enum READ_STATES_e
{
  TX_READ_WAIT_STATE  = 0,
  TX_READ_REQ_STATE   = 1,
  RX_READ_RESP_STATE  = 2
} READ_STATES;


typedef enum WRITE_STATES_e
{
  TX_WRITE_WAIT_STATE  = 0,
  TX_WRITE_REQ_STATE   = 1,
  RX_WRITE_RESP_STATE  = 2
} WRITE_STATES;

const unsigned long read_period = 100;

// enable handling of large commands
#define MAX_CMD_BUFF_LENGTH   254

// enable handling of large responses
#define MAX_RSP_BUF_LENGTH    254

// there is no length member in the protocol, input pkt lenght is 8
#define UTAC_PKT_LENGHT       8

// define the default time out for rs232 read
#define UTAC_PKT_READ_TMO     2000

// RS_232C comm buffers - total of 255 bytes each - I like big buffers and I cannot lie
uint8_t rx_buff[MAX_CMD_BUFF_LENGTH + 1];
uint8_t tx_buff[MAX_RSP_BUF_LENGTH + 1];

// RS_232C serial port
HardwareSerial* rs232port = &Serial2;

// RS_232C speed
#define RS232_SPEED   9600

// set temperature cmd from UTAC
const uint16_t UTAC_SET_TEMP_CMD = 0x012C;

// struct to hold the cmd contents
typedef struct utac_cmd_s
{
  uint8_t   addr; // RS485 id
  uint8_t   msg;  // read or write
  uint16_t  cmd;  // the command
  uint16_t  val;  // the value for the command
  uint16_t  crc;  // the crc-16 for the pkt
} utac_cmd_t;


// function declarations
void runReadStateMachine(void);
bool runWriteStateMachine(int);
void dumpCurrentSvPvHistoryAndRelays(void);
int16_t readCmdFromRS232(uint8_t*, int32_t = UTAC_PKT_LENGHT,
                        unsigned long = UTAC_PKT_READ_TMO);
bool writeResponseToRS232(uint8_t*, uint16_t);
uint16_t getCRC16(uint16_t CRC, uint8_t byte);
uint16_t calcCRC16(uint8_t* pBuff, uint16_t length);
uint16_t getTimeStamp(void);
void logDataPoint(uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t,
                   int, int, int, int);


const uint16_t CRC16_table_C[256] = {
    // CRC-CCIT calculated for every byte between 0x0000 and 0x00FF
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};


// the Sv Pv values array
typedef struct data_point_s
{
  uint16_t  ts; // timestamp when log was created - fits in a 16bit value
  uint16_t  sv1; // Sv at the time log was created - in the fomat read from accu
  uint16_t  pv1; // Pv at the time the log was created - in the fomat read from accu
  uint16_t  sv2; // Sv at the time log was created - in the fomat read from accu
  uint16_t  pv2; // Pv at the time the log was created - in the fomat read from accu
  uint16_t  sv3; // Sv at the time log was created - in the fomat read from accu
  uint16_t  pv3; // Pv at the time the log was created - in the fomat read from accu
  uint16_t  sv4; // Sv at the time log was created - in the fomat read from accu
  uint16_t  pv4; // Pv at the time the log was created - in the fomat read from accu
  uint8_t   r1:1; // state of relay 1 // 0 = LOW, 1 = HIGH
  uint8_t   r2:1; // state of relay 2
  uint8_t   r3:1; // state of relay 3
  uint8_t   r4:1; // state of relay 4
} data_point_t;


// how many minutes of log data to hold in minutes
#define MINUTES_OF_DATA_POINTS  5

// sample period in seconds
#define DATA_POINT_LOG_PERIOD   5

// sample period in milliseconds
#define DATA_POINT_LOG_PERIOD_MS  (DATA_POINT_LOG_PERIOD * 1000)

// number of data points over the MINUTES_OF_DATA_POINTS and DATA_POINT_LOG_PERIOD
#define MAX_NUM_DATA_POINTS     ((MINUTES_OF_DATA_POINTS * 60) / MINUTES_OF_DATA_POINTS)

data_point_t  data_points[MAX_NUM_DATA_POINTS]; // too many , too little ?
int data_points_index = 0;   // log will wrap, need to keep track of last entry
bool  log_data_point  = false;  // true if should log a data point
static unsigned long long last_data_point_log_time = (DATA_POINT_LOG_PERIOD * 1000);


#define htons(x) ( ((x)<< 8 & 0xFF00) | ((x)>> 8 & 0x00FF) )
#define ntohs(x) htons(x)
#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                    ((x)<< 8 & 0x00FF0000UL) | \
                    ((x)>> 8 & 0x0000FF00UL) | \
                    ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)
