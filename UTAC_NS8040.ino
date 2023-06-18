#include "UTAC_NS8040.h"

//
// UTAC_v1.03_230107
// - fixed the calcCRC to be htons on the send
// - enabled DEBUG
//

// debug the time stamping of the logged data
#define DEBUG_TS

// debug !
//#define DEBUG

// debug the I2C bus usage
#define DEBUG_I2C

volatile int32_t msgs_received = 0;
volatile int32_t bad_msgs = 0;

//
// set by the ISR for Wire I2C communication see 
// see void receiveEvent(int howMany)
//
bool process_cmd_buff = false;

/*******************************************************************************
*
*
*
*******************************************************************************/
void setup()
{
  // initialize serial communication at 9600 bits per second:
  #ifdef DEBUG_I2C
  Serial.begin(115200);
  while (!Serial);
  Serial.println("-----------------------------------------");
  Serial.println("CONTROLLINO Modbus RTU Master Test Sketch");
  Serial.println("-----------------------------------------");
  Serial.println("");
  #endif

  //
  // initialize
  //
  initialize();
}

 
/*******************************************************************************
*
*
*******************************************************************************/
void loop()
{
  // read the accuthermos and store their Sv and Pv
  runReadStateMachine();

  // if the ISR ran, process_cmd_buff will be true, go process the pkt
  if( (true == process_cmd_buff) )
  {
    process_cmd_buff = false;
    handleRS232Cmd();
  }

  //
  // check for any input on the Serial1 port, and if present write out the
  // stored temperature data
  //
  dumpTempData();

  // throttle - Bob don't like these
  delay(1000);
}


/*******************************************************************************
*
*
*******************************************************************************/
void runReadStateMachine(void)
{

  READ_STATES readState;
  uint8_t currentQuery;
  static unsigned long long readStateMachineStartMillis = millis();
  bool done;
  uint8_t last_error = 0;

  
  if (millis() - readStateMachineStartMillis < read_period)
  {
    #ifdef DEBUG
    Serial.println("runReadStateMachine wait time not expired, returning");
    #endif

    return;
  }

  #ifdef DEBUG
  Serial.println("runReadStateMachine wait time expired, entering state machine");
  #endif


  // enter the state machine
  currentQuery = 0;
  readState = TX_READ_REQ_STATE;  // initialize readState to 'begin'
  done = false;
  

  do
  {
    switch ( readState )
    {
      case TX_READ_REQ_STATE:
      {
        #ifdef DEBUG
        Serial.print("runReadStateMachine sending Modbus query: ");
        Serial.println(currentQuery);
        #endif

        ControllinoModbusMaster.query( ModbusQuery[currentQuery] ); // send query (only once)
        readState = RX_READ_RESP_STATE;
    
        break;
      }
    
      case RX_READ_RESP_STATE:
      {
        ControllinoModbusMaster.poll(); // check incoming messages
    
        if (ControllinoModbusMaster.getState() == COM_IDLE)
        {
          last_error = 0;
          last_error = ControllinoModbusMaster.getLastError();
          
          #ifdef DEBUG
          Serial.print("runReadStateMachine completed Modbus query: ");
          Serial.print(currentQuery); Serial.print(", last_error: "); Serial.println(last_error);
          #endif

          // response from the slave was received
          digitalWrite(CONTROLLINO_D0, HIGH);

          //
          // currentQuery == 3 means all the requests have been completed
          //
          // update the results arrays and adjust relays
          //
          if (currentQuery == MAX_SLAVE_IDS - 1)  // finished sending all Modbus queries..
          {
            #ifdef DEBUG
            Serial.println("runReadStateMachine completed all Modbus queries");
            #endif

            done = true;

            //
            // the relay array holds history of the whether the relay would be opend or not.
            // when the history is all 0 - open the relay
            // when the history is all 1 - close the relay
            // currently the history is 3 samples deep - which is ~3 seconds
            //

            /* following erases history  -- BUT IS HOW IT USED TO BE -- put this back maybe */
            for (int i = 0; i < 2; i++)
            {
              relay1Array[i + 1] = relay1Array[i];
              relay2Array[i + 1] = relay2Array[i];
              relay3Array[i + 1] = relay3Array[i];
              relay4Array[i + 1] = relay4Array[i];
            }

            //
            // these if blocks check the Pv versus Sv of each accuthermo
            // if the delta is within allowable range of SV_PV_DELTA - relayArrays get 1
            // if the delta is outside allowable range of SV_PV_DELTA - relayArrays get 0
            //
            if (abs((int16_t)(ModbusSlaveRegisters[0] - ModbusSlaveRegisters[1])) <= SV_PV_DELTA)
              relay1Array[0] = 1;
            else
              relay1Array[0] = 0;
      
            if (abs((int16_t)(ModbusSlaveRegisters[2] - ModbusSlaveRegisters[3])) <= SV_PV_DELTA)
              relay2Array[0] = 1;
            else
              relay2Array[0] = 0;

            if (abs((int16_t)(ModbusSlaveRegisters[4] - ModbusSlaveRegisters[5])) <= SV_PV_DELTA)
              relay3Array[0] = 1;
            else
              relay3Array[0] = 0;
      
            if (abs((int16_t)(ModbusSlaveRegisters[6] - ModbusSlaveRegisters[7])) <= SV_PV_DELTA)
              relay4Array[0] = 1;
            else
              relay4Array[0] = 0;
    
            //
            // udpate the relays - use the stored history to 'smooth' the curve
            // and avoid spike in either temperature direction
            //
            // - if all 1 enable relay
            // - if all 0 disable relay
            // - else do nothing 
            //
            if (relay1Array[0] == 1 && relay1Array[1] == 1 && relay1Array[2] == 1) 
              digitalWrite(CONTROLLINO_R1, HIGH);
            else if (relay1Array[0] == 0 && relay1Array[1] == 0 && relay1Array[2] == 0)
              digitalWrite(CONTROLLINO_R1, LOW);
    
            if (relay2Array[0] == 1 && relay2Array[1] == 1 && relay2Array[2] == 1)
              digitalWrite(CONTROLLINO_R2, HIGH);
            else if (relay2Array[0] == 0 && relay2Array[1] == 0 && relay2Array[2] == 0)
              digitalWrite(CONTROLLINO_R2, LOW);
    
            if (relay3Array[0] == 1 && relay3Array[1] == 1 && relay3Array[2] == 1)
              digitalWrite(CONTROLLINO_R3, HIGH);
            else if (relay3Array[0] == 0 && relay3Array[1] == 0 && relay3Array[2] == 0)
              digitalWrite(CONTROLLINO_R3, LOW);
    
            if (relay4Array[0] == 1 && relay4Array[1] == 1 && relay4Array[2] == 1)
              digitalWrite(CONTROLLINO_R4, HIGH);
            else if (relay4Array[0] == 0 && relay4Array[1] == 0 && relay4Array[2] == 0)
              digitalWrite(CONTROLLINO_R4, LOW);
    
            #ifdef DEBUG
            dumpCurrentSvPvHistoryAndRelays();
            #endif
  
            //
            // how often to log  - on a period or after the set cmd has been handled
            //
            if( (((millis() - last_data_point_log_time) > DATA_POINT_LOG_PERIOD_MS) ||
                  (true == log_data_point)) )
            {
              log_data_point            = false;
              last_data_point_log_time  = millis();
  
              // log add a data point 
              logDataPoint(
                ModbusSlaveRegisters[0],  ModbusSlaveRegisters[1],  // unit 1 sv, pv
                ModbusSlaveRegisters[2],  ModbusSlaveRegisters[3],  // unit 2 sv, pv
                ModbusSlaveRegisters[4],  ModbusSlaveRegisters[5],  // unit 3 sv, pv
                ModbusSlaveRegisters[6],  ModbusSlaveRegisters[7],  // unit 4 sv, pv
                digitalRead(CONTROLLINO_R1),  // realy 1 status
                digitalRead(CONTROLLINO_R2),  // relay 2 status
                digitalRead(CONTROLLINO_R3),  // relay 3 status
                digitalRead(CONTROLLINO_R4)   // relay 4 status
              );
            }
          } else //end if (currentQuery == 3)
          {
            // have not sent all mod bus queries
            // send the next ModbusQuery
            currentQuery++;
            readState = TX_READ_REQ_STATE;
          }
    
          digitalWrite(CONTROLLINO_D0, LOW);
        }

        break;
      } // end case RX_READ_RESP_STATE:
    } // end switch ( readState )

  } while( (false == done) );
}


/*******************************************************************************
*
*
*******************************************************************************/
void dumpCurrentSvPvHistoryAndRelays(void)
{
  float pvF, svF;


  // registers read was proceed
  #ifdef DEBUG
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
  Serial.print("history array: ");
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
  Serial.print("history array: ");
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
  
  #endif
}


/*******************************************************************************
*
* remove all bytes from the Rx buffer
*
* called when the last packet read from the buffer was junk, this will remove
* the rest of the junk
*
*******************************************************************************/
void flushWire(void)
{
  uint8_t junk;
  int     count = 0;

  while( (0 != Wire.available()) )
  {
    junk = Wire.read();
    count++;
  }

  Serial.print("flushWire() removed: "); Serial.print(count); Serial.println(" bytes");
  Serial.print("restarting Wire");
  restart_wire();
}

/*******************************************************************************
*
*
* - pkt looks like this
* | Address | Message | Command | Value   | CRC-16  |
* |  8 bits | 8 bits  | 16 bits | 16 bits | 16 bits |
* |  01H  | 06H   | 01H.2CH | 03H.84H | xxxx  |
*

*******************************************************************************/
bool validUTACCmd(utac_cmd_t* cmd)
{
  uint16_t  calc_crc = 0;


  // check the CRC
  calc_crc = calcCRC16((uint8_t*)cmd, sizeof(utac_cmd_t) - 2);

  if( (calc_crc != ntohs(cmd->crc)) )
  {
    #ifdef DEBUG
    Serial.print("validUTACCmd invalid crc :"); Serial.print(cmd->crc, 16);
    Serial.print(" expected: "); Serial.println(calc_crc, 16);
    #endif

    bad_msgs +=1;
    return(false);

  } else
  {
    #ifdef DEBUG
    Serial.print("validUTACCmd match on crc :"); Serial.print(ntohs(cmd->crc), 16);
    Serial.print(" expected: "); Serial.println(calc_crc, 16);
    #endif
  }

  // rs486 id in range
  if( (MAX_SLAVE_IDS < cmd->addr) )
  {
    #ifdef DEBUG
    Serial.print("validUTACCmd invalid id :"); Serial.println(cmd->addr);
    #endif
    return(false);
  }

  // msg is the write command
  if( (MB_FC_WRITE_REGISTER != cmd->msg) )
  {
    #ifdef DEBUG
    Serial.print("validUTACCmd invalid modbus cmd (want 6) :"); Serial.println(cmd->msg);
    #endif
    return(false);
  }


  // supported command
  if( (UTAC_SET_TEMP_CMD != ntohs(cmd->cmd)) )
  {
    #ifdef DEBUG
    Serial.print("validUTACCmd invalid UTAC cmd :"); Serial.println(ntohs(cmd->cmd));
    #endif
    return(false);
  }

  #ifdef DEBUG
  Serial.println("validUTACCmd valid UTAC cmd recieved");
  #endif
  return(true);
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


/*******************************************************************************
*
* until a set clock cmd is added, set the time to:
*
* Sunday Jan 1, 2023 00:00:00
*
*******************************************************************************/
void initializeRTC(void)
{
  // initialize the RTC
  Controllino_RTC_init();

  Controllino_SetTimeDate(
      1,    // Sunday Jan 1st - this is the date day, i.e. 1st
      0,    // day of the week, Sunday = 0
      0,    // month Jan = 0
      123,  // yead - 1900; 2023 - 1900 = 123
      0,    // hour
      0,    // minute
      0     // seconds
  ); 
}


/*******************************************************************************
*
* format of time stamp not defined yet
*
* going to sum the return parameters from 
*
* char Controllino_ReadTimeDate(
*     unsigned char *aDay, unsigned char *aWeekDay, unsigned char *aMonth,
*     unsigned char *aYear, unsigned char *aHour, unsigned char *aMinute,
*     unsigned char *aSecond)
*
*******************************************************************************/
#define NUM_RTC_PARAMS  7
uint16_t getTimeStamp(void)
{
  uint16_t  timestamp = 0;
  unsigned char data[NUM_RTC_PARAMS];
  

/*
  // get the current time from the RTC
  Controllino_ReadTimeDate(&data[0], &data[1], &data[2], &data[3],
                          &data[4], &data[5], &data[6]);

  // sum the return parameters to get a number, timestamp
  for(int i = 0; i < NUM_RTC_PARAMS; i++)
  {
    timestamp += (uint8_t)data[i];
  }
*/
  timestamp = millis() / 1000;

  
  #ifdef DEBUG_TS
  Serial.print("getTimeStamp returning: "); Serial.println((uint16_t)timestamp);
  #endif

  return((uint16_t)timestamp);
}


/*******************************************************************************
*
*
*
*******************************************************************************/
void logDataPoint(uint16_t sv1, uint16_t pv1, uint16_t sv2, uint16_t pv2,
                  uint16_t sv3, uint16_t pv3, uint16_t sv4, uint16_t pv4,
                  int r1, int r2, int r3, int r4)
{
  // update at data_points_index, then increment it
  data_points[data_points_index]  =
  {
    getTimeStamp(), // time stamp
    sv1, pv1,       // Sv and Pv for accu 1
    sv2, pv2,       // Sv and Pv for accu 2
    sv3, pv3,       // Sv and Pv for accu 3
    sv4, pv4,       // Sv and Pv for accu 4
    (bool)r1, (bool)r2, (bool)r3, (bool)r4  // realay status
  };

  #ifdef DEBUG_TS
  Serial.println("logDataPoint added ts, 4 pair Sv, Pv, last 4 are relay status:");
  Serial.print(data_points[data_points_index].ts); Serial.print(", ");
  Serial.print(data_points[data_points_index].sv1); Serial.print(", ");
  Serial.print(data_points[data_points_index].pv1); Serial.print(", ");
  Serial.print(data_points[data_points_index].sv2); Serial.print(", ");
  Serial.print(data_points[data_points_index].pv2); Serial.print(", ");
  Serial.print(data_points[data_points_index].sv3); Serial.print(", ");
  Serial.print(data_points[data_points_index].pv3); Serial.print(", ");
  Serial.print(data_points[data_points_index].sv4); Serial.print(", ");
  Serial.print(data_points[data_points_index].pv4); Serial.print(", ");
  Serial.print(data_points[data_points_index].r1); Serial.print(", ");
  Serial.print(data_points[data_points_index].r2); Serial.print(", ");
  Serial.print(data_points[data_points_index].r3); Serial.print(", ");
  Serial.print(data_points[data_points_index].r4); Serial.println("");
  #endif

  data_points_index = (data_points_index + 1) % MAX_NUM_DATA_POINTS;
  
  #ifdef DEBUG
  Serial.print("data_points_index : "); Serial.println(data_points_index);
  #endif
}


/*******************************************************************************
*
* read the RS232 connection, handle the input command
*
*******************************************************************************/
void handleRS232Cmd(void)
{
  int16_t buff_len        = 0;
  utac_cmd_t* p_utac_cmd  = 0;
  uint8_t target_id;
  bool  outcome = true; // overall operation outcome


  //
  // have the rx_buff loaded with bytes from the i2c - set buff_len
  //
  buff_len = MAX_CMD_BUFF_LENGTH;


  //
  // for now, we're only handling the 8 byte long cmd from email transaction
  //
  // - pkt looks like this
  // | Address | Message | Command | Value   | CRC-16  |
  // |  8 bits | 8 bits  | 16 bits | 16 bits | 16 bits |
  // |  01H  | 06H   | 01H.2CH | 03H.84H | xxxx  |
  // 
  // there is no length member, so until further notice, always read 8 bytes
  // 
  // rx_bytes is the count of bytes expected to be read
  // 
  p_utac_cmd  = (utac_cmd_t*)rx_buff; // cast into the buffer

  if( (false == validUTACCmd(p_utac_cmd)) )
  {
    #ifdef DEBUG
    Serial.print("\n\n\nhandleRS232Cmd invalid UTAC cmd received, flush Wire\n\n");
    #endif
    
    flushWire();
    p_utac_cmd->addr = 0xff;
    
    #ifdef DEBUG
    Serial.println("sending back modified pkt to have 0xffff for cmd and val to indicate fail");
    #endif

    // reply with FF's replacing the cmd and val .. ?
    p_utac_cmd->cmd = 0xFFFF;
    p_utac_cmd->val = 0xFFFF;
    p_utac_cmd->crc = htons(calcCRC16((uint8_t*)(p_utac_cmd), sizeof(utac_cmd_t) - 2));

    //
    // the reply will be sent when reply ISR is invoked - only setting up the 
    // buffer's contents here
    //
    
    return;
  }

  //
  // handle the cmd
  //
  switch(ntohs(p_utac_cmd->cmd))
  {
    case UTAC_SET_TEMP_CMD: // this is the only command !
    {
      #ifdef DEBUG
      Serial.print("handleRS232Cmd found UTAC_SET_TEMP_CMD: ");
      Serial.println(ntohs(p_utac_cmd->cmd));
      #endif

      //
      // put the new value for the affected id in the set_values array
      //  - if the input id is 0, update all the set_values with the new value
      //
      if( (0 == p_utac_cmd->addr) )
      {
        #ifdef DEBUG
        Serial.print("handleRS232Cmd input id is 0, updating all Svs to :");
        Serial.println(ntohs(p_utac_cmd->val), 16);
        #endif

        for(int i = 0; i < MAX_SLAVE_IDS; i++)
          set_values[i] = ntohs(p_utac_cmd->val);

      } else
      {
        target_id = p_utac_cmd->addr - 1;
        
        #ifdef DEBUG
        Serial.print("handleRS232Cmd input id is "); Serial.print(p_utac_cmd->addr);
        Serial.print(" which is zero-based id: "); Serial.print(target_id);
        Serial.print(", updating it to :"); Serial.println(ntohs(p_utac_cmd->val), 16);
        #endif

        // update the Sv array for 'this' set Sv command
        set_values[target_id] = ntohs(p_utac_cmd->val);
      }

      //
      // put the stored set_values into the arrays that are used by the Modbus lib
      // for the ModbusSet structures
      //
      for(int i = 0; i < MAX_SLAVE_IDS; i++)
      {
        // update the 1st 16 bit array address - the Modbus library only uses
        // the 0th array location of au16reg
        ModbusSet[i].au16reg[0] = set_values[i];

        #ifdef DEBUG
        Serial.print("updated Sv for write for zero-based id: "); Serial.print(i);
        Serial.print(" to: "); Serial.println(ModbusSet[i].au16reg[0], 16);
        #endif
      }

      if( (0 == p_utac_cmd->addr) )
      {
        #ifdef DEBUG
        Serial.println("writing to all Ids");
        #endif

        //
        // run the write state machine for all Ids - not using the broadcast
        // address - never tested it, so call Sv for each accuthermo individually
        //
        for(int i= 0; i < MAX_SLAVE_IDS; i++)
        {
          if( (false == runWriteStateMachine(i)))
          {
            // best effort, write to all Ids and keep track of failure
            outcome = false;

            #ifdef DEBUG
            Serial.print("failure writing to zero-based id: "); Serial.println(i);
            #endif
          } else
          {
            #ifdef DEBUG
            Serial.print("success writing to zero-based id: "); Serial.println(i);
            #endif
          }
        }
      } else
      {
        if( (false == runWriteStateMachine(target_id)) )
        {
          #ifdef DEBUG
          Serial.print("writing to Id: "); Serial.print(target_id);
          Serial.println("...");
          #endif
          
          // best effort, write to all Ids and keep track of failure
          outcome = false;

          #ifdef DEBUG
          Serial.print("failure writing to id: "); Serial.println(target_id);
          #endif
        } else
        {
          #ifdef DEBUG
          Serial.print("success writing to id: "); Serial.println(target_id);
          #endif
        }
      }

      if( (false == outcome))
      {
        #ifdef DEBUG
        Serial.println("sending back modified pkt to have 0xffff for cmd and val to indicate fail");
        #endif

        // reply with FF's replacing the cmd and val .. ?
        p_utac_cmd->cmd = 0xFFFF;
        p_utac_cmd->val = 0xFFFF;
        p_utac_cmd->crc = htons(calcCRC16((uint8_t*)(p_utac_cmd), sizeof(utac_cmd_t) - 2));

      } else // else send back the original pkt - no need to recalculate the CRC16
      {
        #ifdef DEBUG
        Serial.println("sending back original pkt to indicate success");
        #endif
      }

      // the ISR for Wire communication will write back this buff
      break;
    }

    default:
    {
      #ifdef DEBUG
      Serial.print("handleRS232Cmd unhandled cmd: ");
      Serial.println(p_utac_cmd->cmd);
      #endif

      // and the ISR invoked for Wire communication will write back this buff
      break;
    }
  }
}


/*******************************************************************************
*
*
*
*******************************************************************************/
void initialize(void)
{
  #ifdef DEBUG
  Serial.println("initialize(void) entered");
  #endif

  //
  // startup the RS_232 connection
  //
  Wire.begin(I2C_ADDR);
  Wire.setClock(100000);
  Wire.onReceive(receiveEvent); //register event
  Wire.onRequest(requestEvent);

  //
  // initialize the real time clock (RTC)
  // set the time to Jan 1 2023 00:00:00 until we add the set RTC command
  //
  initializeRTC();

  //
  // initialize the data_points array
  //
  memset(data_points, '\0', sizeof(data_points));

  //
  // initialize the set_values array
  //
  memset(set_values, '\0', sizeof(set_values));
  have_initial_sv = false;

  //
  // initialize the relay?Array(s)
  //
  memset(relay1Array, '\0', sizeof(relay1Array));
  memset(relay2Array, '\0', sizeof(relay2Array));
  memset(relay3Array, '\0', sizeof(relay3Array));
  memset(relay4Array, '\0', sizeof(relay4Array));

  //
  // initialize last_data_point_log_time and log flag
  //
  last_data_point_log_time = millis();
  log_data_point = false;

  //
  // create the get structures to get SV and PV
  //
  ModbusQuery[0].u8id = 1; // slave address
  ModbusQuery[0].u8fct = 3; // function code (this one is registers read)
  ModbusQuery[0].u16RegAdd = 0x1000; // start address in slave
  ModbusQuery[0].u16CoilsNo = 2; // number of elements (coils or registers) to read
  ModbusQuery[0].au16reg = ModbusSlaveRegisters; // pointer to a memory array

  ModbusQuery[1].u8id = 2;
  ModbusQuery[1].u8fct = 3;
  ModbusQuery[1].u16RegAdd = 0x1000;
  ModbusQuery[1].u16CoilsNo = 2;
  ModbusQuery[1].au16reg = ModbusSlaveRegisters + 2;

  ModbusQuery[2].u8id = 3;
  ModbusQuery[2].u8fct = 3;
  ModbusQuery[2].u16RegAdd = 0x1000;
  ModbusQuery[2].u16CoilsNo = 2;
  ModbusQuery[2].au16reg = ModbusSlaveRegisters + 4;

  ModbusQuery[3].u8id = 4;
  ModbusQuery[3].u8fct = 3;
  ModbusQuery[3].u16RegAdd = 0x1000;
  ModbusQuery[3].u16CoilsNo = 2;
  ModbusQuery[3].au16reg = ModbusSlaveRegisters + 6;

  //
  // create the set structures
  // 
  ModbusSet[0].u8id = 1;
  ModbusSet[0].u8fct = 6;
  ModbusSet[0].u16RegAdd = 0x0000;
  ModbusSet[0].u16CoilsNo = 2;
  ModbusSet[0].au16reg = ModbusSlaveRegisters + 8;

  ModbusSet[1].u8id = 2;
  ModbusSet[1].u8fct = 6;
  ModbusSet[1].u16RegAdd = 0x0000;
  ModbusSet[1].u16CoilsNo = 2;
  ModbusSet[1].au16reg = ModbusSlaveRegisters + 10;

  ModbusSet[2].u8id = 3;
  ModbusSet[2].u8fct = 6;
  ModbusSet[2].u16RegAdd = 0x0000;
  ModbusSet[2].u16CoilsNo = 2;
  ModbusSet[2].au16reg = ModbusSlaveRegisters + 12;

  ModbusSet[3].u8id = 4;
  ModbusSet[3].u8fct = 6;
  ModbusSet[3].u16RegAdd = 0x0000;
  ModbusSet[3].u16CoilsNo = 2;
  ModbusSet[3].au16reg = ModbusSlaveRegisters + 14;

  ControllinoModbusMaster.begin( 19200 ); // baud-rate at 19200
  ControllinoModbusMaster.setTimeOut( 2000 ); // roll over if no answer in 2000 ms

  pinMode(CONTROLLINO_R1, OUTPUT);
  pinMode(CONTROLLINO_R2, OUTPUT);
  pinMode(CONTROLLINO_R3, OUTPUT);
  pinMode(CONTROLLINO_R4, OUTPUT);
  pinMode(CONTROLLINO_D0, OUTPUT);
  digitalWrite(CONTROLLINO_R1, LOW);
  digitalWrite(CONTROLLINO_R2, LOW);
  digitalWrite(CONTROLLINO_R3, LOW);
  digitalWrite(CONTROLLINO_R4, LOW);

  // start Serial1 for the data output
  Serial1.begin(9600);

  #ifdef DEBUG
  Serial.println("initialize(void) exiting");
  #endif
}


/*******************************************************************************
*
*
*******************************************************************************/
bool runWriteStateMachine(int currentQuery)
{
  bool retVal = true;
  WRITE_STATES writeState = TX_WRITE_REQ_STATE;
  bool done = false;
  uint8_t byte_count  = 0;
  uint8_t last_error  = 0;


  #ifdef DEBUG
  Serial.println("runWriteStateMachine entered, writting:");
  Serial.print("id: "); Serial.print(currentQuery); Serial.print(" Sv: ");
  Serial.println(ModbusSet[currentQuery].au16reg[0], 16);
  #endif


  // enter the state machine
  

  do
  {
    switch ( writeState )
    {
      case TX_WRITE_REQ_STATE:
      {

        #ifdef DEBUG
        Serial.print("runWriteStateMachine sending Modbus query: ");
        Serial.println(currentQuery);
        #endif

        ControllinoModbusMaster.query( ModbusSet[currentQuery] ); // send query (only once)
        writeState = RX_WRITE_RESP_STATE;
    
        break;
      }
    
      case RX_WRITE_RESP_STATE:
      {
        byte_count = ControllinoModbusMaster.poll(); // check incoming messages
        /*
        #ifdef DEBUG
        Serial.print("runWriteStateMachine got byte count: ");
        Serial.println(byte_count);
        #endif
        */
        
        if (ControllinoModbusMaster.getState() == COM_IDLE)
        {
          #ifdef DEBUG
          Serial.print("runWriteStateMachine completed Modbus query: ");
          Serial.println(currentQuery);
          #endif

          last_error = ControllinoModbusMaster.getLastError();

          // response from the slave was received
          digitalWrite(CONTROLLINO_D0, HIGH);

          if( (0 != last_error) )
          {
            #ifdef DEBUG
            Serial.print("runWriteStateMachine got error: ");
            Serial.println(last_error);
            #endif

            retVal  = false;
          } else
          {
            #ifdef DEBUG
            Serial.println("runWriteStateMachine success writing cmd rx response");
            #endif

            retVal  = true;
          }

          done = true;

          digitalWrite(CONTROLLINO_D0, LOW);
        }

        break;
      } // end case RX_WRITE_RESP_STATE:
    } // end switch ( writeState )
  } while( (false == done) );

  return(retVal);
}

// BS function that executes whenever data received from master
// this function is registered as an event, see initialize()
void receiveEvent(int howMany)
{
  msgs_received += 1;
  
  // read the data from the I2c
  Serial.print("receiveEvent called howMany: "); Serial.println(howMany);

  memset(rx_buff, '\0', MAX_CMD_BUFF_LENGTH + 1);
 
  for(int i = 0; i < howMany && i < MAX_CMD_BUFF_LENGTH; i++)
  {
    rx_buff[i] = Wire.read();
  }

  Serial.print("receiveEvent read: ");
  for(int i = 0; i < howMany && i < MAX_CMD_BUFF_LENGTH; i++)
  {
    Serial.print("0x"); Serial.print(rx_buff[i], 16); Serial.print(" ");
  }

  Serial.println(" ");
  Serial.println("receiveEvent done");

  #ifdef DEBUG_I2C
  Serial.print("msgs_received: "); Serial.println(msgs_received);
  Serial.print("bad_msgs: "); Serial.println(bad_msgs);
  #endif

  process_cmd_buff  = true;
}

// BS function that executes whenver data is requested by master
// this function is registered as an event, see initialize()
void requestEvent() {
  Serial.print("\trequestEvent called, writing: "); Serial.println(rx_buff[0], 16);
//  Wire.write(rx_buff[0]);
  Wire.write(rx_buff, 8);
  Serial.println("\trequestEvent done");
}

void restart_wire()
{
  Wire.end();
  delay(5000);
  Wire.begin(I2C_ADDR);
  Wire.setClock(100000);
  Wire.onReceive(receiveEvent); //register event
  Wire.onRequest(requestEvent);
}


void dumpTempData(void)
{
  char buff[92];
  char fbuffs[64];
  
  
  if( (0 == Serial1.available()) )
  {
    #ifdef DEBUG
    Serial.println("no request for dump temperature data");
    #endif

    return;
  }

  #ifdef DEBUG
  Serial.println("dumping temperature data");
  #endif

  // clear input data from the Serial1 UART - this is char that let us know they want the data
  while( (0 < Serial1.available()) )
  {
    Serial1.read();
  }

/* 
data_points[data_points_index]
data_points_index = (data_points_index + 1) % MAX_NUM_DATA_POINTS;
*/

  // dump out the data in comma separated format
  // time stamp, sv1, pv1, sv2, pv2, sv3, pv3, sv4, pv4

  Serial1.println("");
  Serial1.println("");
  Serial1.println("begin ----------------------");

  // handle wrap-around in the stats data struct
  // start at data_points_index
  // end at data_points_index -1

  // probably a defect here ..
  // will write out my thinking
  // because there is wrap around in this structure, need to find where to start dumping and where to stop dumping
  //
  // so if time stamp is 0 at data_points_index - have not wrapped around yet
  int start = 0;
  int end = 0;
  if( (0 == data_points[data_points_index].ts) )
  {
    start = 0;
    end = MAX_NUM_DATA_POINTS;
  } else
  {
    // have wrapped around data_points_index is pointiong at the youngest/earliest entry
    start = data_points_index;
    end = (data_points_index = 0 ? (MAX_NUM_DATA_POINTS - 1) : (data_points_index - 1) % MAX_NUM_DATA_POINTS);
  }


  Serial1.println("ts,pv1,sv1,pv2,sv2,pv3,sv3,pv4,sv4");

  for(int i = 0, index = start; ((i < MAX_NUM_DATA_POINTS) && (index != end)); i++)
  {
    // if no data at this line . . split !
    if( (0 == data_points[index].ts) )
      break;
      
    // build one line of output
    memset(buff, '\0', sizeof(buff));
    memset(fbuffs, '\0', sizeof(fbuffs));

    sprintf(buff, "%hu, %s, %s, %s, %s, %s, %s, %s, %s",
      data_points[index].ts,
      dtostrf(convertToFloat(data_points[index].pv1), 6, 2, &fbuffs[0]),
      dtostrf(convertToFloat(data_points[index].sv1), 6, 2, &fbuffs[8]),
      dtostrf(convertToFloat(data_points[index].pv2), 6, 2, &fbuffs[16]),
      dtostrf(convertToFloat(data_points[index].sv2), 6, 2, &fbuffs[24]),
      dtostrf(convertToFloat(data_points[index].pv3), 6, 2, &fbuffs[32]),
      dtostrf(convertToFloat(data_points[index].sv3), 6, 2, &fbuffs[40]),
      dtostrf(convertToFloat(data_points[index].pv4), 6, 2, &fbuffs[48]),
      dtostrf(convertToFloat(data_points[index].sv4), 6, 2, &fbuffs[56])
    );

    Serial1.println(buff);
    Serial1.flush();

    index = (index + 1) % MAX_NUM_DATA_POINTS;
  }

  Serial1.println("");
  Serial1.println("");
  Serial1.println("end ----------------------");
}


float convertToFloat(uint16_t val)
{
  float f = 0.0;

  
  if( (0x8000 & val) )
  {
    #ifdef DEBUG_CONV
    Serial.println("handling negative number conversion");
    #endif

    f = (float)((val+512)&1023) - 512.0 ;
    f = f / (float)10;

  } else
  {
    #ifdef DEBUG_CONV
    Serial.println("handling positive number conversion");
    #endif

    f = (float)val / (float)10;
  }

  #ifdef DEBUG
  Serial.print("returning : "); Serial.println(f);
  #endif
  return(f);
}


/* End of the example. Visit us at https://controllino.biz/ or contact us at info@controllino.biz if you have any questions or troubles. */

/* 2017-03-31: The sketch was successfully tested with Arduino 1.8.1, Controllino Library 1.1.0 and CONTROLLINO MAXI and MEGA. */
