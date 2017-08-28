#include <bluefruit.h>

#define LAMP_ARRAY_SIZE   32
#define GROUP_COUNT       4
#define GROUP_SIZE        8
bool    DEBUG             = true;

uint8_t lamp_num          = 0;
uint8_t group_matrix[GROUP_COUNT][GROUP_SIZE];
unsigned long conn_ref_time;
bool sent_convar = false;
bool disconnecting = false;
uint16_t c_conn_handle;

typedef struct {
  uint8_t id;
  uint8_t addr[3];
  uint8_t c_status;
  uint8_t c_level;
  uint8_t min_level;
  uint8_t max_level;
  uint8_t tc_cap;
  uint8_t coolest;
  uint8_t warmest;
  uint8_t c_temp;
  uint8_t sensor0_int = 0;
  uint8_t sensor0_frac = 0;
  uint8_t sensor1_int = 0;
  uint8_t sensor1_frac = 0;
} Lamp;

Lamp LampArray[LAMP_ARRAY_SIZE];

const char* name_of_lamp = "BLEL";

// BLE Client Service (Connection to Peripheral aka next device)
BLEClientUart clientUart;

// BLE Peripheral Service (Connection to Phone)
BLEUart bleUart;
BLEDis  bleDis;

void setup()
{
  Serial.begin(115200);
  if (DEBUG) Serial.println("Bluefruit Lamp COMM");
  
  //Initialize the group matrix
  for (unsigned int x = 0; x < GROUP_COUNT; x++) {
    for (unsigned int y = 0; y < GROUP_SIZE; y++) {
      group_matrix[x][y] = 255;
    }
  }
  if (DEBUG) Serial.println("Group matrix initialized");

  //----- BLUETOOTH SETUP -----
  
  // Enable both peripheral and central
  Bluefruit.begin(true, true);
  
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("BLE Lighting Control");

  // Configure and Start Device Information Service
  bleDis.setManufacturer("Adafruit Industries");
  bleDis.setModel("Bluefruit Feather52");
  bleDis.begin();
  
  //----- PERIPHERAL CONNECTION SETUP -----
  
  // Callbacks for Peripheral
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start BLE Uart Service
  bleUart.begin();
  //bleUart.setRxCallback(handleDataFromPhone); //Didn't work, has to be called separately

  // Set up Advertising Packet
  setupAdv();

  //----- CENTRAL CONNECTION SETUP -----

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback_c);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback_c);

  // Init BLE Central Uart Service
  clientUart.begin();
  clientUart.setRxCallback(handleDataFromPeripheral);

  /* Setup Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Filter only accept bleuart service
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback_c);
  Bluefruit.Scanner.restartOnDisconnect(false);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.filterUuid(bleUart.uuid);
  Bluefruit.Scanner.useActiveScan(false);

}

void setupAdv(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleUart 128-bit uuid
  Bluefruit.Advertising.addService(bleUart);

  //Include Appearance for phone identification
  Bluefruit.Advertising.addAppearance(54321);

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.ScanResponse.addName();

  /* Setup Advertising
   * - Don't start auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
}

void loop()
{  
  checkConnectionStatuses();
  
  //handleDataFromSerial();
  
  handleDataFromPhone();
  
  //handleDataFromPeripheral() works automatically and there is no need to call it separately
}

void checkConnectionStatuses(void)
{
  if ( !Bluefruit.connected() ) disconnecting = false;

  //If a device has connected as master but didn't send init command, disconnect
  if (Bluefruit.connected() && !disconnecting && !sent_convar && millis()-conn_ref_time > 6000) {
    Serial.println("Didn't receive init from phone, disconnecting");
    Bluefruit.disconnect();
    disconnecting = true;
  }
  //Connection to phone lost, start advertising
  if ( millis()-conn_ref_time > 10000 && !Bluefruit.Advertising.isRunning() && !Bluefruit.connected() && Bluefruit.Central.connected() ) {
    Bluefruit.Advertising.start(0);
    Serial.println("Advertising");
  }
  //No connection to a lamp, start scanning
  if ( !Bluefruit.Scanner.isRunning() && !Bluefruit.Central.connected() ) {
    Bluefruit.Scanner.start(0);
    Serial.println("Scanning");
  }

  //If connection to peripheral is ready for transmission, RED LED of the nRF52 Feather is turned on
  if ( Bluefruit.Central.connected() && clientUart.discovered() ) {
    digitalWrite(LED_RED, HIGH);
  } else {
    digitalWrite(LED_RED, LOW);
  }
}

void handleDataFromSerial(void)
{
  // Forward data from HW Serial to Phone (BLEUART) and Peripheral (CLIENTUART)
  while (Serial.available())
  {
    // Delay to wait for enough input, since we have a limited transmission buffer
    delay(2);

    uint8_t buf[64];
    int count = Serial.readBytes(buf, sizeof(buf));
    
    //Send data to phone
    bleUart.write( buf, count );

    if ( Bluefruit.Central.connected() && clientUart.discovered() )
    {
      // Discovered means peripheral connection is in working state
      // Send data to peripheral
      clientUart.write(buf, count);
    }
  }
}

void handleDataFromPhone(void)
{
  // Forward from Phone (BLEUART) to HW Serial and possible Peripheral (CLIENTUART)

  uint8_t buf2[64];
  uint16_t idx = 0;
  uint8_t ch;
  memset(buf2, 0, 63);
  while ( bleUart.available() )
  {
    ch = (uint8_t) bleUart.read();
    if (ch == '!') {
      buf2[idx] = ch;
      idx++;
      break;
    }
    else {
      buf2[idx] = ch;
      idx++;
    }
  }
  
  if (idx > 0 && !sent_convar && buf2[0] == 'I') {
    Serial.println("Sending constants & variables");
    sendConVarInfoToPhone();
  }
  //If group command, process it
  else if (idx > 0 && buf2[0] == 'G') {
    if (buf2[2] == '+') {
      addLampToGroup(buf2[1], buf2[3]);
    }
    else {
      removeLampFromGroup(buf2[1], buf2[3]);
    }
  }
  //Otherwise send the command forward
  else if ( idx > 0 && Bluefruit.Central.connected() && clientUart.discovered() )
  {
    // Peripheral connection is in working state
    // Forward data to Peripheral
    clientUart.write(buf2, idx);
    Serial.println("Sent a command to a lamp");
  }
  /*
  //Forward data to HW Serial
  if (DEBUG && idx > 0) {
    Serial.println((char*)buf2);
  }*/
}

void handleDataFromPeripheral(BLEClientUart& cent_uart)
{
  uint8_t buf3[64];
  uint16_t idx = 0;
  uint8_t ch;
  memset(buf3, 0, 63);


  //Read data from Peripheral
  while ( clientUart.available() )
  {
    ch = (uint8_t) clientUart.read();
    if (ch == '!') {
      buf3[idx] = ch;
      idx++;
      break;
    }
    else {
      buf3[idx] = ch;
      idx++;
    }
  }

  uint8_t resp[64];
  memset(resp, 0, 63);

  //QUERY: Message is from a lamp that joined the chain and wants an id
  if (buf3[0] == 'Q') {
    
    bool found = false;

    //Check first if lamp has already been assigned an id
    for (uint8_t i = 0; i < lamp_num && !found; i++) {
      
      // If found in array, send the id in response
      if (   LampArray[i].addr[0] == buf3[1] 
          && LampArray[i].addr[1] == buf3[2] 
          && LampArray[i].addr[2] == buf3[3]   ) {
        
        resp[0] = 'R';              //Response type
        resp[1] = buf3[1];          //Address byte 1
        resp[2] = buf3[2];          //Address byte 2
        resp[3] = buf3[3];          //Address byte 3
        resp[4] = LampArray[i].id;  //Id of the lamp
        resp[5] = '!';              //End message
        
        clientUart.write(resp, 6);
        found = true;
        if (DEBUG) {
          Serial.print("Sent id to a KNOWN lamp, ID: ");
          Serial.println(LampArray[i].id);
        }
      }
    }
    
    //Lamp not found in array, assign a new id
    if (!found) {
      
      resp[0] = 'R';              //Response type
      resp[1] = buf3[1];          //Address byte 1
      resp[2] = buf3[2];          //Address byte 2
      resp[3] = buf3[3];          //Address byte 3
      resp[4] = lamp_num;         //Id of the new lamp
      resp[5] = '!';              //End message
      
      //Update the LampArray
      LampArray[lamp_num].id = lamp_num;
      LampArray[lamp_num].addr[0] = buf3[1];
      LampArray[lamp_num].addr[1] = buf3[2];
      LampArray[lamp_num].addr[2] = buf3[3];

      //Send the reply to the lamp
      clientUart.write(resp, 6);
      if (DEBUG) {
        Serial.print("Sent id to a NEW lamp, ID: ");
        Serial.println(lamp_num);
      }
      
      lamp_num++;
    }
  }
  
  else if (buf3[0] == 'A' && Bluefruit.connected()) {
    bleUart.write( buf3, idx );
  }

  else if (buf3[0] == 'C') {
    if (DEBUG) {
      Serial.print("Received constants from ");
      Serial.println(buf3[1]);
    }
    LampArray[ buf3[1] ].min_level = buf3[2];
    LampArray[ buf3[1] ].max_level = buf3[3];
    LampArray[ buf3[1] ].tc_cap = buf3[4];
    LampArray[ buf3[1] ].coolest = buf3[5];
    LampArray[ buf3[1] ].warmest = buf3[6];

    if ( sent_convar && Bluefruit.connected() ) bleUart.write( buf3, idx );
  }

  else if (buf3[0] == 'U') {
    if (DEBUG) {
      Serial.print("Received update from ");
      Serial.println(buf3[1]);
    }
    LampArray[ buf3[1] ].c_status = buf3[2];
    LampArray[ buf3[1] ].c_level = buf3[3];
    LampArray[ buf3[1] ].c_temp = buf3[4];
    LampArray[ buf3[1] ].sensor0_int = buf3[5];
    LampArray[ buf3[1] ].sensor0_frac = buf3[6];
    LampArray[ buf3[1] ].sensor1_int = buf3[7];
    LampArray[ buf3[1] ].sensor1_frac = buf3[8];
    
    if ( sent_convar && Bluefruit.connected() ) bleUart.write( buf3, idx );
  }
}

void scan_callback_c(ble_gap_evt_adv_report_t* report)
{
  uint8_t len = 0;
  uint8_t const* data = extractScanData(report->data, report->dlen, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &len);
  bool is_lamp = false;

  //Check if the scan report is from a BLE lamp
  while( len ) {
    // found matched
    if ( !memcmp(data, (const uint8_t*)name_of_lamp, sizeof(name_of_lamp)) ) {
      is_lamp = true;
      break;
    }
    else {
      data += sizeof(name_of_lamp);
      len  -= sizeof(name_of_lamp);
    }
  }
  // Check if the lamp advertising contains BleUart service
  if ( is_lamp )
  {
    if (DEBUG) {
      Serial.println("BLE Lamp detected");
      Serial.println("Attempt to connect to peripheral ... ");
    }

    // Connect to device with bleUart service in advertising
    // Use Min & Max Connection Interval default value
    Bluefruit.Central.connect(report);
  }
}

void connect_callback_c(uint16_t conn_handle)
{
  if (DEBUG) {Serial.println("Connected to peripheral");}

  if (DEBUG) {Serial.print("Discovering BLE Uart Service ... ");}

  if ( clientUart.discover(conn_handle) )
  {
    if (DEBUG) {
      Serial.println("Found it");
      Serial.println("Enable TXD's notify");
    }
    clientUart.enableTXD();
    c_conn_handle = conn_handle;
    if (DEBUG) Serial.println("Ready to receive from peripheral");
  }
  else
  {
    if (DEBUG) {Serial.println("Found NONE, disconnecting");}
    Bluefruit.Central.disconnect(conn_handle);
  } 
}

void disconnect_callback_c(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  if (DEBUG) Serial.println("Disconnected from peripheral");
}

void connect_callback(uint16_t conn_handle)
{
  if (DEBUG) Serial.println("Connected to a phone");  
  conn_ref_time = millis();
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) reason;
  
  if (DEBUG) {
  Serial.println();
  Serial.println("Disconnected from the phone");
  sent_convar = false;
  }
}

void addLampToGroup(uint8_t group_num, uint8_t lamp_id) {
  for (int i = 0; i < GROUP_SIZE; i++) {
    if (group_matrix[group_num][i] == 255) {
      group_matrix[group_num][i] = lamp_id;
      if (DEBUG) { 
        Serial.print("Added lamp ");
        Serial.print(lamp_id);
        Serial.print(" to group ");
        Serial.println(group_num);
      }
      break;
    }
  }
}

void removeLampFromGroup(uint8_t group_num, uint8_t lamp_id) {
  for (int i = 0; i < GROUP_SIZE; i++) {
    if (group_matrix[group_num][i] == lamp_id) {
      group_matrix[group_num][i] = 255;
      if (DEBUG) { 
        Serial.print("Removed lamp ");
        Serial.print(lamp_id);
        Serial.print(" from group ");
        Serial.println(group_num);
      }
      break;
    }
  }
}

void sendConVarInfoToPhone() {
  uint8_t buf[32];
  memset(buf, 0, 31);
  
  //Send the information of all known lamps if there are any
  if (lamp_num > 0) {
    
    //First, send constants of the lamps
    for (int i = 0; i < lamp_num; i++) {
      buf[0] = 'C';                       //Response type
      buf[1] = LampArray[i].id;           //Id of a lamp in the list of known lamps
      buf[2] = LampArray[i].min_level;    //Current status of a lamp in the list of known lamps
      buf[3] = LampArray[i].max_level;    //Current level of a lamp in the list of known lamps
      buf[4] = LampArray[i].tc_cap;       //Current level of a lamp in the list of known lamps
      buf[5] = LampArray[i].coolest;      //Current level of a lamp in the list of known lamps
      buf[6] = LampArray[i].warmest;      //Current level of a lamp in the list of known lamps
      buf[7] = '!';                       //End message
  
      bleUart.write( buf, 8 );
    }
    if (DEBUG) Serial.println("Sent constants to phone");
    memset(buf, 0, 31);
    
    //Second, send latest known variables of the lamps
    for (int i = 0; i < lamp_num; i++) {
      buf[0] = 'U';                         //Response type
      buf[1] = LampArray[i].id;             //Id of a lamp in the list of known lamps
      buf[2] = LampArray[i].c_status;       //Current status of a lamp in the list of known lamps
      buf[3] = LampArray[i].c_level;        //Current level of a lamp in the list of known lamps
      buf[4] = LampArray[i].c_temp;         //Current color temp of a lamp in the list of known lamps
      buf[5] = LampArray[i].sensor0_int;    //Integer part of sensor 0 reading
      buf[6] = LampArray[i].sensor0_frac;   //Fraction part of sensor 0 reading
      buf[7] = LampArray[i].sensor1_int;    //Integer part of sensor 1 reading
      buf[8] = LampArray[i].sensor1_frac;   //Fraction part of sensor 1 reading
      buf[9] = '!';                         //End message
  
      bleUart.write( buf, 6 );
    }
    if (DEBUG) Serial.println("Sent variables to phone");

    sendGroupInfoToPhone();
  }
  //No known lamps, send error
  else {
    buf[0] = 'U';                 //Update type
    buf[1] = 255;                 //Error code for "No known lamps in array"
    buf[2] = '!';                 //End message

    bleUart.write( buf, 3 );
  }
  sent_convar = true;
}

void sendGroupInfoToPhone() {
  uint8_t buf[8];
  memset(buf, 0, 7);
  
  for (int g = 0; g < GROUP_COUNT; g++) {
    for (int i = 0; i < GROUP_SIZE; i++) {
      if (group_matrix[g][i] != 255) {
        buf[0] = 'G';                   //Group type
        buf[1] = g;                     //Group number
        buf[2] = '+';                   //Add definer
        buf[3] = group_matrix[g][i];    //Id of the lamp
        buf[4] = '!';                   //End message

        bleUart.write( buf, 5 );
        if (DEBUG) {
          Serial.print("Sent G");
          Serial.print(g);
          Serial.print("+");
          Serial.print(group_matrix[g][i]);
          Serial.println(" to phone");
        }
      }
    }
  }
}

uint8_t* extractScanData(uint8_t const* scandata, uint8_t scanlen, uint8_t type, uint8_t* result_len)
{
  *result_len = 0;

  // len (1+data), type, data
  while ( scanlen )
  {
    if ( scandata[1] == type )
    {
      *result_len = scandata[0]-1;
      return (uint8_t*) (scandata + 2);
    }
    else
    {
      scanlen  -= (scandata[0] + 1);
      scandata += (scandata[0] + 1);
    }
  }

  return NULL;
}

void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here

  // Request CPU to enter low-power mode until an event/interrupt occurs
  //waitForEvent();
}
