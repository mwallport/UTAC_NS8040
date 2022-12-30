#include "UTAC_NS8040.h"

#define DEBUG


/*******************************************************************************
*
*
*
*******************************************************************************/
void setup()
{
  // initialize serial communication at 9600 bits per second:
#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("-----------------------------------------");
  Serial.println("CONTROLLINO Modbus RTU Master Test Sketch");
  Serial.println("-----------------------------------------");
  Serial.println("");
#endif

  //
  // startup the RS_232 connection
  //
  rs232port->begin(RS232_SPEED);


  //
  // get the PV
  //
  ModbusQuery[0].u8id = 1; // slave address
  ModbusQuery[0].u8fct = 3; // function code (this one is registers read)
  ModbusQuery[0].u16RegAdd = 0x1000; // start address in slave
  ModbusQuery[0].u16CoilsNo = 2; // number of elements (coils or registers) to read
  ModbusQuery[0].au16reg = ModbusSlaveRegisters; // pointer to a memory array in the CONTROLLINO

  ModbusQuery[1].u8id = 2; // slave address
  ModbusQuery[1].u8fct = 3; // function code (this one is write a single register)
  ModbusQuery[1].u16RegAdd = 0x1000; // start address in slave
  ModbusQuery[1].u16CoilsNo = 2; // number of elements (coils or registers) to write
  ModbusQuery[1].au16reg = ModbusSlaveRegisters + 2; // pointer to a memory array in the CONTROLLINO

  ModbusQuery[2].u8id = 3; // slave address
  ModbusQuery[2].u8fct = 3; // function code (this one is registers read)
  ModbusQuery[2].u16RegAdd = 0x1000; // start address in slave
  ModbusQuery[2].u16CoilsNo = 2; // number of elements (coils or registers) to read
  ModbusQuery[2].au16reg = ModbusSlaveRegisters + 4; // pointer to a memory array in the CONTROLLINO

  ModbusQuery[3].u8id = 4; // slave address
  ModbusQuery[3].u8fct = 3; // function code (this one is write a single register)
  ModbusQuery[3].u16RegAdd = 0x1000; // start address in slave
  ModbusQuery[3].u16CoilsNo = 2; // number of elements (coils or registers) to write
  ModbusQuery[3].au16reg = ModbusSlaveRegisters + 6; // pointer to a memory array in the CONTROLLINO

  ControllinoModbusMaster.begin( 19200 ); // baud-rate at 19200
  ControllinoModbusMaster.setTimeOut( 5000 ); // if there is no answer in 5000 ms, roll over

  myState = TX_READ_WAIT_STATE;
  currentQuery = 0;

  pinMode(CONTROLLINO_R1, OUTPUT);
  pinMode(CONTROLLINO_R2, OUTPUT);
  pinMode(CONTROLLINO_R3, OUTPUT);
  pinMode(CONTROLLINO_R4, OUTPUT);
  pinMode(CONTROLLINO_D0, OUTPUT);
  digitalWrite(CONTROLLINO_R1, LOW);
  digitalWrite(CONTROLLINO_R2, LOW);
  digitalWrite(CONTROLLINO_R3, LOW);
  digitalWrite(CONTROLLINO_R4, LOW);

  startMillis = millis();
}

 
/*******************************************************************************
*
*
*******************************************************************************/
void loop()
{
  runReadStateMachine();
  runWriteStateMachine();
}


/*******************************************************************************
*
*
*******************************************************************************/
void runReadStateMachine(void)
{
  int i;


  switch ( myState )
  {
  case TX_READ_WAIT_STATE:
  {
    currentMillis = millis();

    if (currentMillis - startMillis >= period)
    myState = TX_READ_REQ_STATE;

    break;
  }

  case TX_READ_REQ_STATE:
  {
    ControllinoModbusMaster.query( ModbusQuery[currentQuery] ); // send query (only once)
    myState = RX_READ_RESP_STATE;
    currentQuery++;

    break;
  }

  case RX_READ_RESP_STATE:
  {
    ControllinoModbusMaster.poll(); // check incoming messages

    if (ControllinoModbusMaster.getState() == COM_IDLE)
    {
    // response from the slave was received

    digitalWrite(CONTROLLINO_D0, HIGH);

    myState = TX_READ_WAIT_STATE;

    startMillis = currentMillis;

    //
    // currentQuery == 4 means all the requests have been sent and
    // received
    //
    // update the results arrays and adjust relays
    //
    if (currentQuery == 4)
    {
      // reset for the next group
      currentQuery = 0;

/* this erases history  -- BUT IS HOW IT USED TO BE -- put this back maybe
      for (i = 0; i < 2; i++)
      {
      relay1Array[i + 1] = relay1Array[i];
      relay2Array[i + 1] = relay2Array[i];
      relay3Array[i + 1] = relay3Array[i];
      relay4Array[i + 1] = relay4Array[i];
      }
*/
      //
      // following will overwrite the last history at [2] with [1] then pull
      // [1] to [2], the [0] to [1], then below we update [0]
      //
      for (i = RELAY_HISTORY_DEPTH - 1; i < 0; i--)
      {
      relay1Array[i] = relay1Array[i - 1];
      relay2Array[i] = relay2Array[i - 1];
      relay3Array[i] = relay3Array[i - 1];
      relay4Array[i] = relay4Array[i - 1];
      }

      if (abs(ModbusSlaveRegisters[0] - ModbusSlaveRegisters[1]) <= SV_PV_DELTA)
      relay1Array[0] = 1;
      else
      relay1Array[0] = 0;

      if (abs(ModbusSlaveRegisters[2] - ModbusSlaveRegisters[3]) <= SV_PV_DELTA)
      relay2Array[0] = 1;
      else
      relay2Array[0] = 0;

      if (abs(ModbusSlaveRegisters[4] - ModbusSlaveRegisters[5]) <= SV_PV_DELTA)
      relay3Array[0] = 1;
      else
      relay3Array[0] = 0;

      if (abs(ModbusSlaveRegisters[6] - ModbusSlaveRegisters[7]) <= SV_PV_DELTA)
      relay4Array[0] = 1;
      else
      relay4Array[0] = 0;


      //
      // udpate the relays
      //
      if (relay1Array[0] == 1 && relay1Array[1] == 1 && relay1Array[2] == 1) 
      digitalWrite(CONTROLLINO_R1, HIGH);
      else
      digitalWrite(CONTROLLINO_R1, LOW);

      if (relay2Array[0] == 1 && relay2Array[1] == 1 && relay2Array[2] == 1)
      digitalWrite(CONTROLLINO_R2, HIGH);
      else
      digitalWrite(CONTROLLINO_R2, LOW);

      if (relay3Array[0] == 1 && relay3Array[1] == 1 && relay3Array[2] == 1)
      digitalWrite(CONTROLLINO_R3, HIGH);
      else
      digitalWrite(CONTROLLINO_R3, LOW);

      if (relay4Array[0] == 1 && relay4Array[1] == 1 && relay4Array[2] == 1)
      digitalWrite(CONTROLLINO_R4, HIGH);
      else
      digitalWrite(CONTROLLINO_R4, LOW);

#ifdef DEBUG
      dumpCurentSvPvHistoryAndRelays();
#endif
    }

    digitalWrite(CONTROLLINO_D0, LOW);
    }

    break;
  }
  }
}


/*******************************************************************************
*
* - pkt looks like this
* | Address | Message | Command | Value   | CRC-16  |
* |  8 bits | 8 bits  | 16 bits | 16 bits | 16 bits |
* |  01H  | 06H   | 01H.2CH | 03H.84H | xxxx  |
*
* - if read cmd from RS232
* - parse cmd and is valid
* - temperature set only for now ( 12/29/2022 )
* - send the cmd to the Address
* - return the result of the cmd
*
*******************************************************************************/
void runWriteStateMachine(void)
{
  int16_t buff_len        = 0;
  utac_cmd_t* p_utac_cmd  = 0;


  // read cmd from RS232
  memset(rx_buff, '\0', sizeof(rx_buff));  // MAX_CMD_BUFF_LENGTH + 1

  buff_len = getCmdFromRS232(rx_buff);

  if( (0 >= buff_len) )  // this is -1 or 0 return from getCmdFromRS232
  {
    #ifdef DEBUG
    Serial.println("runWriteStateMachine no rs232 pkt received");
    #endif

    // no pkt received
    return;
  }


  //
  // for now, we're only handling the 8 byte long cmd from email transaction
  //
  p_utac_cmd  = (utac_cmd_t*)rx_buff; // cast into the buffer

  if( (false == validUTACCmd(p_utac_cmd)) )
  {
    #ifdef DEBUG
    Serial.print("runWriteStateMachine invalid UTAC cmd received, flush rs232");
    #endif

    flushRS232();
  }

  // run the write state machine
}


/*******************************************************************************
*
*
*******************************************************************************/
void dumpCurentSvPvHistoryAndRelays(void)
{
  float pvF, svF;


  // registers read was proceed
  Serial.println("---------- READ RESPONSE RECEIVED ----");

  Serial.print("Slave ");
  Serial.print(0, DEC);
  Serial.print(" , Process Value: ");
  pvF = ModbusSlaveRegisters[0] / (float)10;
  Serial.print(pvF, 1);
  Serial.print(" , Set Value: ");
  svF = ModbusSlaveRegisters[1] / (float)10;
  Serial.println(svF, 1);
  Serial.print("history array: ");
  Serial.print(relay1Array[0]);
  Serial.print(", "); Serial.print(relay1Array[1]);
  Serial.print(", "); Serial.println(relay1Array[2]);
  Serial.print("CONTROLLINO_R1 is: ");
  if( (HIGH == digitalRead(CONTROLLINO_R1)) )
  Serial.println("HIGH");
  else
  Serial.println("LOW");
  Serial.println("-------------------------------------");

  Serial.println("");
  Serial.print("Slave ");
  Serial.print(1, DEC);
  Serial.print(" , Process Value: ");
  pvF = ModbusSlaveRegisters[2] / (float)10;
  Serial.print(pvF, 1);
  Serial.print(" , Set Value: ");
  svF = ModbusSlaveRegisters[3] / (float)10;
  Serial.println(svF, 1);
  Serial.print("history array: ");
  Serial.print(relay2Array[0]);
  Serial.print(", "); Serial.print(relay2Array[1]);
  Serial.print(", "); Serial.println(relay2Array[2]);
  Serial.print("CONTROLLINO_R2 is: ");
  if( (HIGH == digitalRead(CONTROLLINO_R2)) )
  Serial.println("HIGH");
  else
  Serial.println("LOW");
  Serial.println("-------------------------------------");
  Serial.print("Slave ");
  Serial.print(2, DEC);
  Serial.print(" , Process Value: ");
  pvF = ModbusSlaveRegisters[4] / (float)10;
  Serial.print(pvF, 1);
  Serial.print(" , Set Value: ");
  svF = ModbusSlaveRegisters[5] / (float)10;
  Serial.println(svF, 1);
  Serial.print(relay3Array[0]);
  Serial.print(", "); Serial.print(relay3Array[1]);
  Serial.print(", "); Serial.println(relay3Array[2]);
  Serial.print("CONTROLLINO_R3 is: ");
  if( (HIGH == digitalRead(CONTROLLINO_R3)) )
  Serial.println("HIGH");
  else
  Serial.println("LOW");
  Serial.println("-------------------------------------");
  Serial.println("");
  Serial.print("Slave ");
  Serial.print(3, DEC);
  Serial.print(" , Process Value: ");
  pvF = ModbusSlaveRegisters[6] / (float)10;
  Serial.print(pvF, 1);
  Serial.print(" , Set Value: ");
  svF = ModbusSlaveRegisters[7] / (float)10;
  Serial.println(svF, 1);
  Serial.print(relay4Array[0]);
  Serial.print(", "); Serial.print(relay4Array[1]);
  Serial.print(", "); Serial.println(relay4Array[2]);
  Serial.print("CONTROLLINO_R4 is: ");
  if( (HIGH == digitalRead(CONTROLLINO_R4)) )
  Serial.println("HIGH");
  else
  Serial.println("LOW");
  Serial.println("-------------------------------------");
  Serial.println("");
}


/*******************************************************************************
*
* - pkt looks like this
* | Address | Message | Command | Value   | CRC-16  |
* |  8 bits | 8 bits  | 16 bits | 16 bits | 16 bits |
* |  01H  | 06H   | 01H.2CH | 03H.84H | xxxx  |
*
* there is no length member, so until further notice, always read 8 bytes
*
* rx_bytes is the count of bytes expected to be read
*
*******************************************************************************/
int16_t getCmdFromRS232(uint8_t* buff, int32_t rx_bytes, unsigned long tmo)
{
  bool timedOut     = false;
  int32_t bytes_read = 0;
  unsigned long startTime = millis();


  #ifdef DEBUG
  Serial.print("getCmdFromRS232 tmo is: "); Serial.println(tmo);
  #endif


  //
  // if nothing on the wire, don't try to read
  //
  if( (0 == rs232port->available()) )
  {
    #ifdef DEBUG
    Serial.println("no data available in rs232");
    #endif

    return(0);
  }


  //
  // try to read a packet for a total of TimeoutMs milliseconds
  // but only start to read the bytes if data is present
  //
  while( (!timedOut) && (bytes_read < rx_bytes) ) 
  {
    if( ((millis() - startTime) > tmo) )
    {
      timedOut = true;
    } else
    {
      if( (rs232port->available()) )
      {
        buff[bytes_read++] = rs232port->read();

      } else
      {
        //
        // no data available, wait a bit before checking again withing the
        // tmo window
        //
        delay(128);
      }
    }
  }


  #ifdef DEBUG
  Serial.print("getCmdFromRS232 received bytes: "); Serial.print(bytes_read);
  Serial.print(", wanted rx_bytes: "); Serial.println(rx_bytes);
  Serial.println("byte received: ");
  for(uint8_t i = 0; i < bytes_read; i++)
  {
    Serial.print(" 0x"); Serial.print(rx_buff[i], 16);
  }
  Serial.println("");
  #endif

  if( (true == timedOut) )
  {
    #ifdef DEBUG
    Serial.print("getCmdFromRS232 timed out reading rs232");
    #endif

    // we timed out reading bytes, Rx buffer is emtpy, no need flush

    return(-1);
  }

  return(bytes_read);
}


/*******************************************************************************
*
* remove all bytes from the Rx buffer
*
* called when the last packet read from the buffer was junk, this will remove
* the rest of the junk
*
*******************************************************************************/
void flushRS232(void)
{
  uint8_t junk;


  while( (0 != rs232port->available()) )
  {
    junk = rs232port->read();
  }
}


/*******************************************************************************
*
*
/*
typedef struct utac_cmd_s
{
  uint8_t   addr; // RS485 id
  uint8_t   msg;  // read or write
  uint16_t  cmd;  // the command
  uint16_t  val;  // the value for the command
  uint16_t  crc;  // the crc-16 for the pkt
} utac_cmd_t;



*******************************************************************************/
bool validUTACCmd(utac_cmd_t* cmd)
{

  // rs486 id in range
  if( (4 < cmd->addr) )
  {
    #ifdef DEBUG
    #endif
    return(false);
  }

  // msg is the write command
  if( (MB_FC_WRITE_REGISTER != cmd->msg) )
  {
    #ifdef DEBUG
    #endif
    return(false);
  }

  // supported command
  if( (UTAC_SET_TEMP_CMD == cmd->cmd) )
  {
    #ifdef DEBUG
    #endif
    return(false);
  }
}


/*******************************************************************************
*
*
*******************************************************************************/
uint16_t getCRC16(uint16_t CRC, uint8_t byte)
{
    CRC = ( (CRC % 256) << 8 ) ^ ( CRC16_table_C[ (CRC >> 8) ^ byte ] );
    return (CRC);
}


/*******************************************************************************
*
*
*******************************************************************************/
uint16_t calcCRC16(uint8_t* pBuff, uint16_t length)
{
    uint16_t    CRC = 0;


    for(uint16_t i = 0; i < length; i++)
    {
        CRC = getCRC16(CRC, pBuff[i]);
    }

    return(CRC);
}


/* End of the example. Visit us at https://controllino.biz/ or contact us at info@controllino.biz if you have any questions or troubles. */

/* 2017-03-31: The sketch was successfully tested with Arduino 1.8.1, Controllino Library 1.1.0 and CONTROLLINO MAXI and MEGA. */
