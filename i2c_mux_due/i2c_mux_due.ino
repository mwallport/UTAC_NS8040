#include <FreeRTOS.h>
#include <queue.h>
#include <Wire.h>

#define TX_PERIOD_MS          1000

// struct to hold the cmd contents
typedef struct utac_cmd_s
{
  uint8_t   addr; // RS485 id
  uint8_t   msg;  // read or write
  uint16_t  cmd;  // the command
  uint16_t  val;  // the value for the command
  uint16_t  crc;  // the crc-16 for the pkt
} utac_cmd_t;

/* 
 * Declaring a global variable of type QueueHandle_t 
 */
QueueHandle_t h2c_structQueue;    // Receives command structs from the 4 Host ports tasks to single send task.
QueueHandle_t c2h1_structQueue;   // ACK back to host port 1
QueueHandle_t c2h2_structQueue;   // ACK back to host port 2
QueueHandle_t c2h3_structQueue;   // ACK back to host port 3
QueueHandle_t c2h4_structQueue;   // ACK back to host port 4

#define MAX_CMD_BUFF_LENGTH   64

#define x4_SERIAL //- uncomment when serial console not needed; switches to include Serial

void setup() {

  h2c_structQueue = xQueueCreate(10, // Queue length
                              sizeof(struct utac_cmd_s) // Queue item size
                              );
  
  c2h1_structQueue = xQueueCreate(2, // Queue length
                              sizeof(struct utac_cmd_s) // Queue item size
                              );
  
  c2h2_structQueue = xQueueCreate(2, // Queue length
                              sizeof(struct utac_cmd_s) // Queue item size
                              );
  
  c2h3_structQueue = xQueueCreate(2, // Queue length
                              sizeof(struct utac_cmd_s) // Queue item size
                              );
  
  c2h4_structQueue = xQueueCreate(2, // Queue length
                              sizeof(struct utac_cmd_s) // Queue item size
                              );

  if(h2c_structQueue != NULL && \
    c2h1_structQueue != NULL && \
    c2h2_structQueue != NULL && \
    c2h3_structQueue != NULL && \
    c2h4_structQueue != NULL)
  {
    // Create task that consumes the queue if it was created.
    xTaskCreate(TaskH2CTx, // Task function
                "H2CTx", // A name just for humans
                128,  // This stack size can be checked & adjusted by reading the Stack Highwater
                NULL, 
                1, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                NULL);

    // Create task that consumes the queue if it was created.
    
/* MA - don't need this task - needed to move the requestFrom into the cth task
    xTaskCreate(TaskC2HRx, // Task function
                "C2HRx", // A name just for humans
                128,  // This stack size can be checked & adjusted by reading the Stack Highwater
                NULL, 
                1, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                NULL);
*/

#ifdef x4_SERIAL  //Only use 3 serial ports if "Serial" needs to be used for debug
    // Create task that publish data in the queue if it was created.
    xTaskCreate(TaskH2CRx1, // Task function
                "H2CRx1", // Task name
                128,  // Stack size
                NULL, 
                1, // Priority
                NULL);
                
    // Create task that publish data in the queue if it was created.
    xTaskCreate(TaskC2HTx1, // Task function
                "C2HTx1", // Task name
                128,  // Stack size
                NULL, 
                1, // Priority
                NULL);
#else
//Serial.begin(115200);                
#endif
    // Create task that publish data in the queue if it was created.
    xTaskCreate(TaskH2CRx2, // Task function
                "H2CRx2", // Task name
                128,  // Stack size
                NULL, 
                1, // Priority
                NULL);

    // Create task that publish data in the queue if it was created.
    xTaskCreate(TaskC2HTx2, // Task function
                "C2HTx2", // Task name
                128,  // Stack size
                NULL, 
                1, // Priority
                NULL);

    // Create task that publish data in the queue if it was created.
    xTaskCreate(TaskH2CRx3, // Task function
                "H2CRx3", // Task name
                128,  // Stack size
                NULL, 
                1, // Priority
                NULL);

    // Create task that publish data in the queue if it was created.
    xTaskCreate(TaskC2HTx3, // Task function
                "C2HTx3", // Task name
                128,  // Stack size
                NULL, 
                1, // Priority
                NULL);

    // Create task that publish data in the queue if it was created.
    xTaskCreate(TaskH2CRx4, // Task function
                "H2CRx4", // Task name
                128,  // Stack size
                NULL, 
                1, // Priority
                NULL);

    // Create task that publish data in the queue if it was created.
    xTaskCreate(TaskC2HTx4, // Task function
                "C2HTx4", // Task name
                128,  // Stack size
                NULL, 
                1, // Priority
                NULL);
  }

  xTaskCreate(TaskBlink, // Task function
              "Blink", // Task name
              128, // Stack size 
              NULL, 
              0, // Priority
              NULL );

  vTaskStartScheduler();
}

void loop() {}


/**
 * TaskH2CRx[1..4] receive Host data from each serial port.  Once a buffer holds a whole command the utac_cmd_s struct gets packed
 * and pushed onto the h2c_structQueue.
 */
#ifdef x4_SERIAL
void TaskH2CRx1(void *pvParameters)
{
  (void) pvParameters;

    // Init Arduino serial
  Serial.begin(9600);
  
  for (;;)
  {
    struct utac_cmd_s* currentCmd;
    uint32_t bytesRead = 0;
    uint8_t buff[MAX_CMD_BUFF_LENGTH + 1];


    memset(buff, '\0', MAX_CMD_BUFF_LENGTH + 1);
    
    if (Serial.available() >= sizeof(utac_cmd_s)) { //wait until FIFO holds a whole command
      for (bytesRead = 0; bytesRead < sizeof(utac_cmd_s); bytesRead++)  {
        buff[bytesRead] = Serial.read();
      }

      //Serial.print("H2CRx1 got pkt: ");
      for(int i = 0; i < 8; i++)
      {
        //Serial.print("0x"); //Serial.print(buff[i], 16); //Serial.print(" ");
      }
      //Serial.println("");
    
      currentCmd = (utac_cmd_s*)buff;
      
      if( (errQUEUE_FULL == xQueueSend(h2c_structQueue, currentCmd, 0)) )
      {
        // flush - shed traffic
        while( (Serial.available()) )
        {
          uint8_t b = Serial.read();
        }
      }
    }

    // One tick delay (15ms) in between reads for stability
    // TODO: longer delay may be desireable.  Investigate.
    vTaskDelay(1);
  }
}

/**
 * TaskC2HTx[1..4] Once an ACK from the card shows up in the appropriate queue, send the ACK to the corresponding serial port.
 */
void TaskC2HTx1(void * pvParameters) {
  (void) pvParameters;
  uint8_t buff[MAX_CMD_BUFF_LENGTH + 1];


  for (;;) 
  {
    memset(buff, '\0', MAX_CMD_BUFF_LENGTH + 1);

    if (xQueueReceive(c2h1_structQueue, (utac_cmd_s*)buff, portMAX_DELAY) == pdPASS) {
      Serial.write(buff, 8);      
    }
  }
}
#endif

void TaskH2CRx2(void *pvParameters)
{
  (void) pvParameters;

    // Init Arduino serial
  Serial1.begin(9600);
  
  for (;;)
  {
    struct utac_cmd_s* currentCmd;
    uint32_t bytesRead = 0;
    uint8_t buff[MAX_CMD_BUFF_LENGTH + 1];

    memset(buff, '\0', MAX_CMD_BUFF_LENGTH + 1);

    //Serial.println("H2CRx2 reading Serial1");
    
    if (Serial1.available() >= sizeof(utac_cmd_s)) {
      for (bytesRead = 0; bytesRead < sizeof(utac_cmd_s); bytesRead++)  {
        buff[bytesRead] = Serial1.read();
      }

      //Serial.print("H2CRx2 got pkt: ");
      for(int i = 0; i < 8; i++)
      {
        //Serial.print("0x"); //Serial.print(buff[i], 16); //Serial.print(" ");
      }
      //Serial.println("");

          currentCmd = (utac_cmd_s*)buff;
      
      if( (errQUEUE_FULL == xQueueSend(h2c_structQueue, currentCmd, 0)) )
      {
        // flush - shed traffic
        while( (Serial1.available()) )
        {
          uint8_t b = Serial1.read();
        }
      }
    }

    // One tick delay (15ms) in between reads for stability
    vTaskDelay(1);
  }
}

void TaskC2HTx2(void * pvParameters) {
  (void) pvParameters;
  uint8_t buff[MAX_CMD_BUFF_LENGTH + 1];


  for (;;) 
  {
    memset(buff, '\0', MAX_CMD_BUFF_LENGTH + 1);

    if (xQueueReceive(c2h2_structQueue, (utac_cmd_s*)buff, portMAX_DELAY) == pdPASS) {
      Serial1.write(buff, 8);      
    }
  }
}

void TaskH2CRx3(void *pvParameters)
{
  (void) pvParameters;

    // Init Arduino serial
  Serial2.begin(9600);

  for (;;)
  {
    struct utac_cmd_s* currentCmd;
    uint32_t bytesRead = 0;
    uint8_t buff[MAX_CMD_BUFF_LENGTH + 1];

    memset(buff, '\0', MAX_CMD_BUFF_LENGTH + 1);

    //Serial.println("H2CRx3 reading Serial2");
    
    if (Serial2.available() >= sizeof(utac_cmd_s)) {
      for (bytesRead = 0; bytesRead < sizeof(utac_cmd_s); bytesRead++)  {
        buff[bytesRead] = Serial2.read();
      }

      //Serial.print("H2CRx3 got pkt: ");
      for(int i = 0; i < 8; i++)
      {
        //Serial.print("0x"); //Serial.print(buff[i], 16); //Serial.print(" ");
      }
      //Serial.println("");

      currentCmd = (utac_cmd_s*)buff;
      
      if( (errQUEUE_FULL == xQueueSend(h2c_structQueue, currentCmd, 0)) )
      {
        // flush - shed traffic
        while( (Serial2.available()) )
        {
          uint8_t b = Serial2.read();
        }
      }
    }

    // One tick delay (15ms) in between reads for stability
    vTaskDelay(1);
  }
}

void TaskC2HTx3(void * pvParameters) {
  (void) pvParameters;
  uint8_t buff[MAX_CMD_BUFF_LENGTH + 1];


  for (;;) 
  {
    memset(buff, '\0', MAX_CMD_BUFF_LENGTH + 1);

    if (xQueueReceive(c2h3_structQueue, (utac_cmd_s*)buff, portMAX_DELAY) == pdPASS) {
      Serial2.write(buff, 8);      
    }
  }
}

void TaskH2CRx4(void *pvParameters)
{
  (void) pvParameters;

  Serial3.begin(9600);

  for (;;)
  {
    struct utac_cmd_s* currentCmd;
    uint32_t bytesRead = 0;
    uint8_t buff[MAX_CMD_BUFF_LENGTH + 1];

    memset(buff, '\0', MAX_CMD_BUFF_LENGTH + 1);

    //Serial.println("H2CRx4 reading Serial3");

    if (Serial3.available() >= sizeof(utac_cmd_s)) {
      for (bytesRead = 0; bytesRead < sizeof(utac_cmd_s); bytesRead++)  {
        buff[bytesRead] = Serial3.read();
      }

      //Serial.print("H2CRx4 got pkt: ");
      for(int i = 0; i < 8; i++)
      {
        //Serial.print("0x"); //Serial.print(buff[i], 16); //Serial.print(" ");
      }
      //Serial.println("");

      currentCmd = (utac_cmd_s*)buff;
      
      if( (errQUEUE_FULL == xQueueSend(h2c_structQueue, currentCmd, 0)) )
      {
        // flush - shed traffic
        while( (Serial3.available()) )
        {
          uint8_t b = Serial3.read();
        }
      }
    }

    // One tick delay (15ms) in between reads for stability
    vTaskDelay(1);
  }
}

void TaskC2HTx4(void * pvParameters) {
  (void) pvParameters;
  uint8_t buff[MAX_CMD_BUFF_LENGTH + 1];


  for (;;) 
  {
    memset(buff, '\0', MAX_CMD_BUFF_LENGTH + 1);

    if (xQueueReceive(c2h4_structQueue, (utac_cmd_s*)buff, portMAX_DELAY) == pdPASS) {
      Serial3.write(buff, 8);      
    }
  }
}

/**
 * TaskH2CTx Once a command is in the queue the command struct gets unpacked into the buff
 * and send over I2C to the card.
 */
void TaskH2CTx(void * pvParameters) {
  (void) pvParameters;

  struct utac_cmd_s* currentCmd;
  uint32_t bytesSent, count;
  uint8_t buff[MAX_CMD_BUFF_LENGTH + 1];
  int bytes_written;
  int bytes_requested;
  int ret_val;
  uint8_t byte_read;
  boolean restart_wire = false;
  volatile int count_down;
  uint32_t total_msg_send_attempts = 0;
  uint32_t msg_send_fail_count = 0;  


  Wire.begin(); // join i2c bus (address optional for master)
  Wire.setClock(100000);

  for (;;) 
  {
    memset(buff, '\0', MAX_CMD_BUFF_LENGTH + 1);

    if (xQueueReceive(h2c_structQueue, (utac_cmd_s*)buff, portMAX_DELAY) == pdPASS)
    {
      //Serial.println("total_msg_send_attempts: "); //Serial.println(total_msg_send_attempts);
      //Serial.println("msg_send_fail_count: "); //Serial.println(msg_send_fail_count);
      
      // send the packet
      //Serial.print("controller sending pkt data: ");
      for(int i = 0; i < 8; i++)
      {
        //Serial.print("0x"); //Serial.print(buff[i], 16); //Serial.print(" ");
      }
      //Serial.println(" ");
      
      Wire.beginTransmission(4);
      bytes_written = 0;
      bytes_written = Wire.write(buff, 8);
      //Serial.print("controller sent bytes_written: "); //Serial.println(bytes_written);
    
      ret_val = 0;
      ret_val = Wire.endTransmission();
    
      total_msg_send_attempts += 1;
      restart_wire = false;
    
      switch(ret_val)
      {
        case 0: 
          //Serial.println("\tendTransmission SCCEESS 0, requesting ACK ...");

          /* need to wait for the time-out period on the Controllino for it
           *  to handle the packet in the worst case scenario
           * 
           * Time out on Controllino is 2 seconds
           */
          //Serial.println("controller sleeping 2 seconds...");
          count_down = 8;
          while(count_down--)
            delay(250);
          //Serial.println("controller done sleeping 2 seconds, requesting ack...");
          
          bytes_requested = 0;
          bytes_requested = Wire.requestFrom(4, 8);    // request 8 byte ACK from peripheral device #4
          //Serial.print("\tbytes_requested: "); //Serial.println(bytes_requested);

          // max 0.5 seconds waiting for byte to come back
          // if byte not come back . .. restart the Wire ?? ? 
          //Serial.println("controller waiting for 1 seconds for response...");
          count_down = 4;
          while(count_down--)
          {
            if( (0 < Wire.available()) )
              break;

            delay(250); // yeah .. the delay on the Due is dodgy lower than 250
          }
          //Serial.println("controller done waiting for 1 seconds for response...");

          memset(buff, '\0', MAX_CMD_BUFF_LENGTH + 1);
          //Serial.print("\tread ack bytes: ");
          for(int i = 0; (i < 8) && Wire.available(); i++)
          {
            buff[i] = Wire.read();
            //Serial.print("0x"); //Serial.print(buff[i], 16); //Serial.print(" ");
          }
          //Serial.println(" ");
          
 
          currentCmd = (utac_cmd_s*)buff;
          switch(buff[0])
          {
            case 1 :
              xQueueSend(c2h1_structQueue, currentCmd, portMAX_DELAY);
              break;
            case 2 :
              xQueueSend(c2h2_structQueue, currentCmd, portMAX_DELAY);
              break;
            case 3 :
              xQueueSend(c2h3_structQueue, currentCmd, portMAX_DELAY);
              break;
            case 4 :
              xQueueSend(c2h4_structQueue, currentCmd, portMAX_DELAY);
              break;
            default :
              // restart Wire here ??
              restart_wire = true;
              break;
          }

          break;
    
        case 1:
          //Serial.println("\tendTransmission FAIL 1 - data too long to fit in Tx buff");
          break;
    
        case 2:
          ////Serial.println("\tendTransmission FAIL 2 - received NACK on transmit of address");
          restart_wire = true;
          break;
    
        case 3:
          //Serial.println("\tendTransmission FAIL 3 - received NACK on transmit of data");
          restart_wire = true;
          break;
    
        case 4:
          //Serial.println("\tendTransmission FAIL 4 - other error");
          restart_wire = true;
          break;
    
        case 5:
          //Serial.println("\tendTransmission FAIL 5 - timeout");
          restart_wire = true;
          break;
    
        default:
          //Serial.print("\tendTransmission FAIL default, got: "); //Serial.println(ret_val);
          restart_wire = true;
          break;
      }
    
      if(true == restart_wire)
      {
        restart_wire_comm();
        msg_send_fail_count += 1;
      }
    }
  }
}


void restart_wire_comm()
{
  Wire.end();
  delay(5000);
  Wire.begin();
  Wire.setClock(100000);
}


/**  MA
 * TaskC2HRx When ACK is received for a particular ID it is pushed on the queue corresponding to that ID.
 *
void TaskC2HRx(void * pvParameters) {
  (void) pvParameters;

  //Serial.begin(9600);

  for(;;)
  {	
    while (Wire.available()) { // peripheral may send less than requested
      uint8_t c = Wire.read(); // receive a byte as character
      switch(c) {
        case 1 :
          xQueueSend(c2h1_structQueue, &c, portMAX_DELAY);
          break;
        case 2 :
          xQueueSend(c2h2_structQueue, &c, portMAX_DELAY);
          break;
        case 3 :
          xQueueSend(c2h3_structQueue, &c, portMAX_DELAY);
          break;
        case 4 :
          xQueueSend(c2h4_structQueue, &c, portMAX_DELAY);
          break;
      }
      //Serial.println(c);         // print the character
    }
    vTaskDelay(1);
  }
}
*/


/* 
 * Blink task. 
 * Heart beat sign of life. 
 */
void TaskBlink(void *pvParameters)
{
  (void) pvParameters;

  pinMode(LED_BUILTIN, OUTPUT);

  for (;;)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay( pdMS_TO_TICKS( 250 ) );
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay( pdMS_TO_TICKS( 250 ) );
  }
}
