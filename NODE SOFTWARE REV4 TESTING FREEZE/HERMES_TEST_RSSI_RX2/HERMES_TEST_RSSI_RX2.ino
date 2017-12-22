//HERMES NODE CODE REV4 - CORTEX ARM4/WIZFI220 - RSSI TESTING
//This code recieves packets and simply reveals an RSSI reading for that packet.
//By Jeffrey Wu

#include <SPI.h>
#include <string.h>

//COMPILER DIRECTIVES

//IP ADDRESS
//254: HADES
//1: BASE STATION
//All Other Numbers: Node

//SET IP ADDR: CHANGE THIS FOR EACH NODE
#define NODE_NUMBER 2
//Define which version of WizFi: 210 FOR TESTING ONLY.
#define WIZFI220
//#define WIZFI210

//output debug digests on Serial.
//- true for LAPTOP
//- false for NODE MODE
#define TESTING true
//#define DIGEST
#define TEST_UP1
//#define TEST_DOWN1
//delay on the transmit buffers
#define tx_delay 15

//command delay
#define inter_command_delay 300

//LIS3DSH Accelerometer control

//THRESHOLDS
//Wait for readings to be stable: Out target: 57000 TILT MAX 54000 TOTALLY FLAT
#define ACCEL_FLAT 50000
#define ACCEL_TILT_MAX 57000
#define ACCEL_DEV 1000
#define ACCEL_STABLE_THRESH 75
#define ACCEL_DELAY 200

//axis address modes
#define X_LOW 0xA8
#define X_HIGH 0xA9
#define Y_LOW 0xAA
#define Y_HIGH 0xAB
#define Z_LOW 0xAC
#define Z_HIGH 0xAD

//control register address modes
#define CTRL_REG4 0x20
//#define CTRL_REG1 0x21
//#define CTRL_REG2 0x22
//#define CTRL_REG3 0x23
//#define CTRL_REG5 0x24

//WIZFI2x0 PIN LAYOUTS
#define UART_CTS 3
#define UART_RTS 2
#define RESET 4
#define HOST_SIG 6
#define ACCEL_CS 9
#define SPI_CS 10
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_CLK 13
#define CONNECTED_FB 14
#define COMMAND_REG 13

//LED ILLUMINATION
#define LED_CONNECTED 21
#define LED_POWER 20

//VOLTAGE DIVIDER MONITORS
#define TEMP_MONITOR A9
#define BATTERY_MONITOR A8

//temp high variables
#define m_high (-1.1)
#define c_high (78.833)
//temp low variables
#define m_low (-0.4)
#define c_low (53.377)

#define volt_multiplier 0.017
#define volt_bias 0.404

//PACKET TYPES
#define PKT_UPDATE 77
#define PKT_URGENT 78
//update 'M'
//urgent 'N'

//Important constants
const unsigned char N_L = '\n'; //newline/Linefeed
const unsigned char C_R = '\r'; //return
const unsigned char esc = 27; //escape character for data transfer
const unsigned char delimiter = '.';

//AT commands
//Startup
const char COMMAND_AT[3] = "AT"; //AT: No operation.
const char COMMAND_DISCO[6] = "AT+WD"; //disconnect
#ifdef WIZFI220
  const char COMMAND_MAX_POWER[9] = "AT+WP=15"; //Set max power for WIZFI220
#else
  const char COMMAND_MAX_POWER[8] = "AT+WP=7"; //Set max power for WIZFI210
#endif
#ifdef WIZFI220
  const char COMMAND_EXT_POWERAMP_ON[11] = "AT+EXTPA=1"; //Set ext power amplifier: Only for WIZFI220
#endif
const char COMMAND_DHCP[11] = "AT+NDHCP=1"; //Set DHCP on.
const char COMMAND_PASSWORD[17] = "AT+WWPA=HADESNET"; //Set HADESNET as password
const char COMMAND_AD_HOC_ENABLE[8] = "AT+WM=1"; //Set Ad-Hoc mode on
const char COMMAND_BULK_MODE[11] = "AT+BDATA=1"; //turn on bulk data mode.
const char COMMAND_UART_MAX[] = "ATB=921600,8,n,1"; //full speed UART.
const char COMMAND_HW_FC[] = "AT&R1"; //HARDWARE CTS/RTS

//Connections and Wifi
const char COMMAND_CONNECT_HADESNET[15] = "AT+WA=HADESNET"; //connect to HADESNET
const char COMMAND_START_UDP_SERVER[14] = "AT+NSUDP=5000"; //start server on local
const char COMMAND_NETSTAT[11] = "AT+NSTAT=?"; //get network statistics
const char COMMAND_GET_RSSI[11] = "AT+WRSSI=?"; //get RSSI.

//Variable AT commands: USES IP.
char COMMAND_SET_SELF_IP[46] = "AT+NSET=10.42.0.000,255.255.255.0,10.42.0.000"; //set a string to be the IP address.
char COMMAND_UPLINK_UDP_CLIENT[26] = "AT+NCUDP=10.42.0.000,5000"; //set port to uplink
char COMMAND_DOWNLINK_UDP_CLIENT[26] = "AT+NCUDP=10.42.0.000,5000"; //Set port to uplink

//Wait for Deployment
void WaitForDeployment();
char SPITransferData(char addr, char din);

//WizFiCommands
void WizFiSetup(); //setup the WizFi module
void WizFiReboot(); //reboot the WizFi module's Networking functions.

void UART_Baud_UP(); //UART BAUD UP.

//Send a command down to WizFi220. NO CHECKS: OPEN LOOP
void send_command(const char*); //send data down on ser1 to wizfi220
void send_command(char*); //The same command but with char* instead of const char*
void command_delay(const char*); //output the command on Serial1 and delay some.

void SetHostIPAddr(); //Set self IP address.
void NodeServerSetup(); //Setup the node host UDP server.
void NodeDownstreamSetup(); //Setup data for downstream Base stn to HADES
void NodeUpstreamSetup(); //Setup data for upstream HADES to Base stn
void Ready_To_Rx(); //go into data mode
void freeCommandMode(); //free command mode.

//Buffer/char output functions
char char_in_Ser1(); //Get a character and bounce it to the testing
void clear_Ser1_Buffers(); //clear the buffers

//ADC monitoring functions
unsigned int readTemp();
unsigned int readBatt();
void readNodeDiagnostic();

//Monitoring Output: Send packets out.
void SendUpdate(); //if Things are fine.
void SendUrgentUpdate(); //if Something is wrong: 1Hz Rate. Sends if Temp/Batt/No Data (node error)

//UTILITY FUNCTIONS
void Int2AsciiExt(unsigned int,char*);//Assumes ptr points to 4 bytes of memory. Output limits are 0 to 999

//System runs on static IP addresses.
const char HADES_IP[13] = "10.42.0.254"; //HADES' IP
const char BASE_STN_IP[11] = "10.42.0.1"; //BASE STATION IP

//Testing Variables
boolean testing = TESTING;
//if true, sends data back down on Serial.

//=====OPERATION VARIABLES=====

//Accelerometer Variables
long z_accel = 0;

//rolling average filter.
int z_buff[10] = {0,0,0,0,0,0,0,0,0,0};

// variable position
unsigned int v_pos = 0;

char high_byte = 0;
char low_byte = 0;

//End Accelerometer Variables.

//Nodes are denoted from 1 to 41+, with reserved last addr and Base station being zero.
unsigned char node_Number = NODE_NUMBER;
unsigned char hostIP[13]; //My IP address in ASCII form.

char length_buffer[4] = "000";
unsigned char IP_HOST = 0;
char target_CID = 0;

//ADC
float temperature = 0.0;
float batteryVoltage = 0.0;
float thermistor = 0.0;

unsigned char lowTraffic = 'F';
unsigned char errorTraffic = 'F';

//=====TEST VARIABLES=====

//Temporary Data buffer
volatile char dataBuff[1024]; //get a char pointer

//PACKET REPEATER
unsigned char TxBuff[1024]; //set up transmit buffer.
volatile int TxPos = 0; //transmit buffer position

//This code determines whether the node has been deployed correctly.
void WaitForDeployment(){

  char stable = 0; //Stable flag
  char valid = 0; //valid flag

  //poll the node until its orientation changes to z >0.75g for a long period.
  while (true){

    z_accel= 0; //reset the acceleration on this axis.
  
    //get data z
    low_byte = SPITransferData(Z_LOW,0x00);
    high_byte = SPITransferData(Z_HIGH,0x00);

    //add it to the array
    z_buff[v_pos] = (high_byte << 8) + low_byte;

    //if the readingsa are within standard deviation of the array reading.
    if ((z_buff[v_pos] > z_accel - ACCEL_DEV) && (z_buff[v_pos] < z_accel + ACCEL_DEV)) stable = 1;
    else stable = 0; //we are no longer stable.

    ++v_pos; //increment the array position
    if (v_pos == 10) v_pos = 0; //reset v position
  
    for (int k = 0; k < 10; ++k){ //add the elements of the acceleration
      z_accel += z_buff[k];
    }

    //divide through to get acceleration
    z_accel /= 10;

    //if the accelerometer is at a valid angle, we increment the number of valid reading points.
    if (z_accel > ACCEL_FLAT && z_accel < ACCEL_TILT_MAX) ++valid;
    else valid = 0;
    
    //break if we are both valid and stable at the same time
    if (valid == ACCEL_STABLE_THRESH && stable == 1) break;

    delay(ACCEL_DELAY); //do readings every 200ms
  }
}

//SETUP THE SPI data LINK
void SPISetup(){
  
  //Begin the transaction
  SPI.begin();
  
  //CHIP SELECT
  pinMode(ACCEL_CS, OUTPUT); //set pin to be chip select.
  
  //set the chip select high.
  digitalWrite(ACCEL_CS, HIGH);
  
  //control register 4
  SPITransferData(CTRL_REG4,0x97);
}

//Setup the various pins and system
void TeenseyPinSetup(){

  //Setup WizFi Reset line.
  pinMode(RESET,INPUT);

  //Setup Illumination LEDs
  pinMode(LED_CONNECTED, OUTPUT); //This determines the LED that blinks indicating data transfer
  pinMode(LED_POWER, OUTPUT); //This illuminates the LED that lights after setup.
  analogWrite(LED_CONNECTED,LOW);
  analogWrite(LED_POWER,LOW);

  //Setup WizFI UART connection inputs/outputs
  pinMode(UART_CTS, OUTPUT);
  pinMode(UART_RTS, INPUT);
  digitalWrite(UART_CTS, HIGH); //clear the Wizfi to send.

  //SPI CONNECTIONS TO ACCELEROMETER.
  //Begin the transaction
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4); //16/4 = 4MHz
  //begin the SPI.
  SPI.begin();
  
  //Setup WizFi Reset line: Switch to output to drive.
  pinMode(RESET,OUTPUT);

  //reset the WizFi
  digitalWrite(RESET,LOW);
  delay(2000);
  digitalWrite(RESET,HIGH);
  delay(1000);
  
  //Set WIZFI return signals to be input.
  pinMode(COMMAND_REG,INPUT);
  pinMode(CONNECTED_FB,INPUT);

  SPISetup(); //setup the SPI data transfer
}

//=====SETUP
void setup() {

  //=====BEGIN=====

  //Setup Teensey system on boot.
  TeenseyPinSetup();

  #ifdef TESTING
    Serial.begin(1000000); //Setup serial connection to Arduino IDE
    Serial.println("Press Any Key to Continue.");
    
    //wait until the signal.
    while (Serial.available() == 0);
    
    Serial.read(); //clear the byte.
  #elif
    delay(500); //delay 0.5 seconds.
  #endif

  Serial1.begin(115200); //start Serial to WIZFI

  #ifdef TESTING
    Serial.println("===HERMES NODE: DEBUG R4.0===");
  #endif
  
  //IMPLEMENTATION: Wait for deployment of node.
  //This function blocks the execution until ready.
  //WaitForDeployment();
  
  WizFiSetup();
}

//EMPTY LOOP: HERE FOR COMPLETENESS.
void loop() {
  //Do NOTHING: Use callbacks to negotiate incoming data in Ready to Rx.

  //=====TESTING=====
  //testing
  //freeCommandMode();
  
  Ready_To_Rx(); //enter Rx mode.
  //=====END TESTING=====
}

//Setup the WizFi220 Module
void WizFiSetup(){

  //=====Start SETUP=====
  clear_Ser1_Buffers();
  
#ifdef TESTING
  Serial.println("Setup Starting...");
#endif

  send_command(COMMAND_AT);
  
  send_command(COMMAND_DISCO);
  send_command(COMMAND_AD_HOC_ENABLE);
  send_command(COMMAND_PASSWORD);
  send_command(COMMAND_DHCP);
  //send_command(COMMAND_HW_FC);
  //send_command(COMMAND_MAX_POWER);
  send_command(COMMAND_BULK_MODE);
  
  #ifdef WIZFI220
  send_command(COMMAND_EXT_POWERAMP_ON);
  #endif
  
  //END CLAUSE CLEAR ALL BUFFERS AND WAIT A WHILE.
  delay(1000);
  #ifdef TESTING
  Serial.println("Setup Complete");
  #endif
  delay(2000);
  
  analogWrite(LED_POWER,128); //set Green LED ON

  //Catch clause for strange [ERROR] that pops up. We just clear it out.
  //Known WizFi preculiarity.
  while (1){
    if (Serial1.available() >= 10){
       clear_Ser1_Buffers(); //clear the buffers
       break;
    }
  }
  
  HADESNET_Scan_Connect(); //connect to HADESNET
  SetHostIPAddr(); //Set self IP address
  
  NodeServerSetup(); //Setup our UDP server: GIVES CID 0
  NodeDownstreamSetup(); //Setup our link to home base station: GIVES CID 1
  NodeUpstreamSetup(); //Setup our link to HADES: GIVES CID 2

  delay(1000);
  
  UART_Baud_UP(); //raise baud rate to max

  analogWrite(LED_CONNECTED,128); //set Orange LED ON

  //allow the power to be maximum
  send_command(COMMAND_MAX_POWER);

  //=====End SETUP =====
}

//Waits for an OK_Message to arrive to continue the program.
boolean OK_Check(){
  
  unsigned char data = 0; //data from serial
  unsigned char counter = 0; //count the number of cycles.
  unsigned char last_count = 0;
  boolean result = false;
  
  //loop forever
  while (1){

    last_count = Serial1.available();
    
    delay(300);//delay first to wait for input from UART from WizFi.

    //poll the serial until we get at least 60.
    if (last_count > 60){
      while (Serial1.available()){
        data = Serial1.read();
        if (data == 'I') result = true;

        if (testing)  Serial.write(data);
      }
      return result;
    }

    if (last_count == Serial1.available()) counter++;
    else counter = 0;
    if (counter == 10) return false;
  }
  return false;
}

void HADESNET_Scan_Connect(){

  Serial.println("Connecting to HADESNET...");
  
  //loop until we return (after getting the OK.)
  while (1){
    
    clear_Ser1_Buffers(); //first clear the buffers.
    //put the request in.
    send_command(COMMAND_CONNECT_HADESNET); //AT: CONNECT TO HADESNET

    //TODO: USE WIZFI AP FEEDBACK.
    if (OK_Check()){ //If we get an active OK
      if(testing) Serial.println("HADESNET connection obtained.");
      break;//If we get a satisfactory response, break.  
    }
    
    if(testing) Serial.println("No HADESNET connection...Continue Scanning");
  }
  
  //clear all buffers, we are connected
  clear_Ser1_Buffers();
  
  delay(500);

}

//gets one byte from the buffer and sends it to the serial. Otherwise, just reads.
//Clears just one char from the buffer and sends it to the serial output.
char char_in_Ser1(){
  char inByte = Serial1.read(); //read the data

  #ifdef TESTING
  Serial.write(inByte); //mirror it to the screen if just testing.
  #endif
  
  return inByte; //give the data back
}

//clears buffer in serial1.
void clear_Ser1_Buffers(){
  while (Serial1.available()){ //flush all bytes in Serial1 Rx buffer.

    #ifdef TESTING
      Serial.write(Serial1.read());
    #else
      Serial1.read();
    #endif
  }
}

//Sends a command to the WizFi220 and then delays,
void command_delay(const char* command){

  //Get the string length.
  int string_len = strlen(command);
  
  //loop and write characters
  for (int i = 0; i < string_len; ++i){
    Serial1.write(command[i]);
  }

  //Print \r to serial.
  Serial1.write(C_R);
  
  //print the newline character
  Serial1.write(N_L);
  
  //delay
  delay(inter_command_delay);
}

//Send command down on serial 1.
void send_command(const char* command){
  clear_Ser1_Buffers(); //clear the buffers anytime before we wish to do anything.
  command_delay(command); //output the command
  delay(50);
}

//Send command down on serial 1 using char ptr
void send_command(char* command){
  clear_Ser1_Buffers(); //clear the buffers anytime before we wish to do anything.
  command_delay(command); //output the command
  delay(50);
}

//Set self IP address.
void SetHostIPAddr() {

  //first calculate our IP address.
  char node_number_ASCII[4] = "000"; 
  char gateway_ASCII[4] = "000";
  
  Int2AsciiExt(node_Number, node_number_ASCII); //to current node
  Int2AsciiExt(node_Number - 1, gateway_ASCII); //to last node

  //set our current node
  COMMAND_SET_SELF_IP[16] = node_number_ASCII[2];
  COMMAND_SET_SELF_IP[17] = node_number_ASCII[1];
  COMMAND_SET_SELF_IP[18] = node_number_ASCII[0];

  //set the next node
  COMMAND_SET_SELF_IP[42] = gateway_ASCII[2];
  COMMAND_SET_SELF_IP[43] = gateway_ASCII[1];
  COMMAND_SET_SELF_IP[44] = gateway_ASCII[0];

  //AT command: IP address set.
  send_command(COMMAND_SET_SELF_IP);

  delay(inter_command_delay);// delay 300ms
}

void NodeServerSetup(){ //Setup our UDP server
  
  //AT command: Start UDP local server on port 5000: This port is the same on all nodes.
  send_command(COMMAND_START_UDP_SERVER);

  delay(inter_command_delay); //delay 300ms.
}

void NodeDownstreamSetup(){ //Setup our link to home base station
  
  //AT command: UDP client to downstream node to port 5000
  //add -1 to our IP address and forward
  //Then get handle and store in variable
  
  char last_node_ASCII[4] = "000";
  
  Int2AsciiExt(node_Number - 1, last_node_ASCII);

  COMMAND_DOWNLINK_UDP_CLIENT[17] = last_node_ASCII[2];
  COMMAND_DOWNLINK_UDP_CLIENT[18] = last_node_ASCII[1];
  COMMAND_DOWNLINK_UDP_CLIENT[19] = last_node_ASCII[0];
  
  send_command(COMMAND_DOWNLINK_UDP_CLIENT); //send the command
  
  delay(inter_command_delay); //delay 300ms.
}

void NodeUpstreamSetup(){ //Setup our link to HADES.
  
  //AT command: UDP client to upstream node to port 5000
  //add +1 to our IP address and forward
  //Then get handle and store in variable
  
  char next_node_ASCII[4] = "000";
  
  Int2AsciiExt(node_Number + 1,next_node_ASCII);

  COMMAND_UPLINK_UDP_CLIENT[17] = next_node_ASCII[2];
  COMMAND_UPLINK_UDP_CLIENT[18] = next_node_ASCII[1];
  COMMAND_UPLINK_UDP_CLIENT[19] = next_node_ASCII[0];
  
  send_command(COMMAND_UPLINK_UDP_CLIENT); //send the command
  
  delay(inter_command_delay); //delay 300ms.
}

//=====UTILITY FUNCTIONS=====

//Int number to ascii string
//assumes char* return is 3 bytes.
void Int2AsciiExt(unsigned int input, char * output){  

  //Set numbers
  unsigned char hundreds = 0, tens = 0, units = 0; //hold val for each area
  tens = input / 10; //set tens
  units = input % 10; //set units

  //Get hundreds
  //basically modulo the tens.
  while (tens > 9)
  {
    hundreds++;
    tens -= 10;
  }

  //Set the array.
  output[0] = units + '0';
  output[1] = tens + '0';
  output[2] = hundreds + '0';

}

//Allows the user to directly make AT commands
void freeCommandMode(){

  //loop indefinitely.
  while (1){

    // read from port 1, send to port 0: FROM COMPUTER
    if (Serial1.available()) { //Mirror the output from Arduino to PC
      char inByte = Serial1.read(); //read the byte from the WIZFI
      Serial.write(inByte); //send it down to UART
    }
  
    // read from port 0, send to port 1: TO ARDUINO
    if (Serial.available()) { //Send PC byte to unit OR change modes
      char inByte = Serial.read(); //read the byte from the controller

      //get out.
      if (inByte == 'x') break;
      else Serial1.write(inByte); //send it to WIZFI

    }// end if 
  }// end while
}

//poll the serial buffers and wait for data.
//Prerequisits: We are already setup for data transmission.
//Needs:
//- Assosiation to HADESNET
//- IP setup correctly
//- CID setup
void Ready_To_Rx(){

  //=====PACKET REPEATER FUNCTION=====

  //Static packet parameters.
  TxBuff[0] = esc; //escape sequence
  TxBuff[1] = 'Z'; //Z for packet indicator
  TxBuff[3] = '0'; // zero in the thousandths position. Max packet size 990 bytes

  //Statistical analysis VARIABLES
  
  //for counting the time
  //Utilization = time/(time + dead_time)
  unsigned long Time = 0; //Time for processing
  unsigned long Dead_Time = 0; //time in-between processing

  //Error flags
  unsigned char ERR = 0; //byte error
  unsigned long DROP = 0; //dropped packets
  unsigned char ID = '0'; //incorrect packet framing
  char LAST_LETTER = '.'; //check for char
  unsigned char test = '0'; //check the last testing byte
  
  //packet length (from the incoming packet.)
  unsigned int pack_len = 0;
  
  //The input byte from Serial.
  unsigned char inByte = 0;

  //loop variable.
  int loop_v = 0;

  //bytes encountered
  unsigned int bytes_encountered = 0;

  //flags variable.
  char set_flags = 0x00;
  //OLD
  //bit 0 = esc sequence found
  //bit 1 = u found
  //bit 2 = addr found
  //bit 3 = SOP found
  //bit 4 = len found = max(235);

  #ifdef TESTING
    Serial.println();
    Serial.println("READY TO READ PACKETS..."); //send packet read start
  #endif
  
  //now clear the buffers
  clear_Ser1_Buffers();

  //LOOP FOREVER
  while (1){
    
    //we recieve some data over the WizFi serial, so we get it into the Serial buffers.
    if (Serial1.available()){
      while (Serial1.available()){
        
        //get the next packet.
        inByte = Serial1.read();

        //CHECK FLAGS and packet
        if ((set_flags == 0x00) && (inByte == delimiter)){

          analogWrite(LED_CONNECTED,0); //set Orange LED OFF
          
          set_flags = 0x01; //set the flag

          while (Serial1.available() < 5); // do nothing until we reach 5 bytes

          #ifdef DIGEST
          //TESTING
            Serial.println();
            //Print Packet start indicator
            Serial.println("PKT START");
  
            //Clear out Opening 4 bytes
            Serial.write(inByte);
            Serial.write(Serial1.read());
            Serial.write(Serial1.read());
            Serial.write(Serial1.read());
            Serial.write(Serial1.read());
          #else
            //IMPLEMENTATION
            Serial1.read();
            Serial1.read();
            Serial1.read();
            Serial1.read();
          #endif

          //==Do Processing==
          //Start processing time.
          Time = millis();
          //Stop the Dead time.
          Dead_Time = millis() - Dead_Time;
          //==End do processing==
        }
        //Check delimiter
        else if (set_flags == 0x01 && inByte == delimiter){
          set_flags = 0x07; //set second flag.
          #ifdef DIGEST
            Serial.write(inByte);
          #endif
        }
        else if (set_flags == 0x01 && inByte != delimiter){
          set_flags = 0x00;//reset state machine.
          #ifdef DIGEST
            Serial.write(inByte);
            Serial.println("DELIMIT BAD");
          #endif
        }
        else if (set_flags == 0x07){
          
          while (Serial1.available() < 2); //wait until we get at least two.

          //1 digit
          //..0.1
          //2 digit
          //..0.11
          //3 digit
          //..0.111

          IP_HOST = 0; //rezero HOST.
          
          //do 1st digit
          IP_HOST += (inByte - '0'); //ones position

          //do 2nd digit
          inByte = Serial1.read(); //try for tens.
          
          if (inByte >= '0' && inByte <= '9'){ // if tens
            
            IP_HOST *= 10; //new tens
            IP_HOST += (inByte - '0'); //new ones
          }

          //do 3rd digit
          inByte = Serial1.read(); //try for hundreds.
          
          if (inByte >= '0' && inByte <= '9'){ // if valid number.
            
            IP_HOST *= 10; //new hundreds and ones
            IP_HOST += (inByte - '0'); //ones
          }
          
          //decide target CID.
          if (IP_HOST < node_Number) target_CID = 2; //send north to HADES
          else target_CID = 1; //send south to base station

          //Serial1.write(esc);
          //Serial1.write('Z');
          //Serial1.write(target_CID + '0');
          //Serial1.write('0');
  
          TxBuff[2] = target_CID + '0'; //Set the Target CID.

          #ifdef DIGEST
            Serial.print(IP_HOST,DEC);
            Serial.println(" WAITING FOR DELIMITER");
          #endif

          set_flags = 0x0F; //set the flag
        }
        else if (set_flags == 0x0F){ //wait until delimiter

          #ifdef DIGEST
            Serial.write(inByte);
          #endif
          
          //Delimit Character is 9.(ASCII for tab.)
          if (inByte == 9){
            set_flags = 0x1F; //Set the flag
            break;
          }
        }
        else if (set_flags == 0x1F){ //Start getting the data. We have reached the end of the header

          while (inByte == C_R || inByte == N_L) inByte = Serial1.read();
          
          //The first byte is the length byte divided down by 10, so we can grab it and put out the length.
          pack_len = inByte * 10; //multiply by 10.

          #ifdef DIGEST
          //print the packet length.
            Serial.println();

            Serial.print(pack_len,DEC);
            Serial.println();
          #endif

          if (pack_len > 990){
            set_flags = 0;
            #ifdef TESTING
              Serial.print("PACK_LEN_ERR");
            #endif
            break;
          }

          #ifdef DIGEST
            Serial.println("=====DATA START=====");
          #endif
          
          //Send the data to array form
          Int2AsciiExt(pack_len,length_buffer);

          TxBuff[4] = length_buffer[2];
          TxBuff[5] = length_buffer[1];
          TxBuff[6] = length_buffer[0];

          //Serial1.write(length_buffer[2]);
          //Serial1.write(length_buffer[1]);
          //Serial1.write(length_buffer[0]);

          TxPos = 7; //reset the TxPosition

          TxBuff[TxPos++] = inByte; //Mirror: put this byte into the system. THIS IS PACKET LENGTH

          //Serial1.write(inByte);

          while (Serial1.available() < 3);
          
          inByte = Serial1.read();

          while (inByte == C_R || inByte == N_L) inByte = Serial1.read(); //drop the new data if it is an illegal character

          //only for testing
          //check the packet continuity
          if (inByte == 'A' && LAST_LETTER == 'Z'){
            //DROP = 0; //correct packet
            DROP = inByte;
          }
          else{
            //DROP = (char)inByte - LAST_LETTER - 1; //correct packet will read 0, incorrect packet will read 1.
            DROP = inByte;
          }
          LAST_LETTER = inByte; //reassign the last packet

          TxBuff[TxPos++] = inByte; //put the next byte in.

          //Serial1.write(inByte);

          //get the next packet.
          inByte = Serial1.read();
          
          while (inByte == C_R || inByte == N_L) inByte = Serial1.read(); //drop the new data if it is an illegal character

          ID = inByte; //should be ID. Means something different in practice vs. in implementaiton.

          bytes_encountered = 3; //3 bytes encountered thus far. Can reduce to 2 by eliminating the drop testing.
          
          while (true){ //new while
            
            if (bytes_encountered == pack_len){ //if we reach the end of the packet.
              set_flags = 0x00; //clear the flag

              //put the last packet into the stack
              TxBuff[TxPos++] = inByte;
              //Serial1.write(inByte);
              /*
              //Send the completed packet to Wizfi.
              for (loop_v = 0; loop_v < TxPos; loop_v++){
                
                #ifdef DIGEST
                  //Mirror to the serial.
                  Serial.write(TxBuff[loop_v]);
                #endif

                //multihop problem here
                Serial1.write(TxBuff[loop_v]); //send the packet out to serial.
              }
              
              #ifdef DIGEST
               //Mirror to the serial.
                Serial.write(N_L);
              #endif

              //multihop problem here.
              Serial1.write(N_L); //send the newline to serial
              */

              //Process the RSSI down to a single character string
              //ask the WizFi for the RSSI.
              send_command(COMMAND_GET_RSSI);
              Serial.print("RSSI: ");

              delay(50);
              
              while (Serial1.available()) {
                inByte = Serial1.read();
                if (inByte == '-' || (inByte >= '0' && inByte <= '9')){
                  Serial.write(inByte);
                }
              }
              Serial.println();
              
              analogWrite(LED_CONNECTED,128); //set Orange LED ON
              
              break; //break while true
              
            }
            else{ //push the data into the array.
              if (inByte != N_L || inByte != C_R){
                TxBuff[TxPos++] = inByte;
              }
              else
                bytes_encountered--;
              //Serial1.write(inByte);
            }
            
            while (!Serial1.available()); //wait until we get a valid byte

            ++bytes_encountered;

            //get the next byte
            inByte = Serial1.read();

            //FOR TESTING ONLY. CHECK WHETHER PACKET IS ERRORED.
            if (test != inByte && inByte != esc && inByte != 'E' && inByte != N_L) ERR = 1;

            //FOR TESTING ONLY. CHECK THE TEST ID and CHANGE IF MAX.
            if (test == '9') test = '0';
            else test++;

          } //end new while

          #ifdef TESTING
          //print the output log
          /*
          Serial.print("DELAY: ");
          Serial.print(millis()-Time);
          Serial.print(" DEAD time: ");
          Serial.print(Dead_Time);
          Serial.print(" ERROR: ");
          Serial.print(ERR,DEC);
          Serial.print(" DRP/DUP: ");
          Serial.write(DROP);
          Serial.print(" Packets. ID: ");
          Serial.write(ID);
          Serial.print(" PACK_SIZE: ");
          Serial.print(bytes_encountered,DEC);
          Serial.print(" FWD_TO: ");
          Serial.print(target_CID,DEC);
          Serial.println();
          */
          #endif

          //RESET VAR
          ERR = 0;
          DROP = 0;
          test = '0';
          //reset the byte counters.
          pack_len = 0;
          bytes_encountered = 0;
          
          //start the end of the dead time.
          Dead_Time = millis();
        }
        //bad packets.
      #ifdef DIGEST
        else{
          //dud bytes print.
          //Serial.write(inByte);
        }
      #endif
      }
      
    } //end if data
    
    #ifdef TESTING
    if (Serial.read() == 'x'){

      clear_Ser1_Buffers();
      break;
    }
    #endif
    
  } //end while 1

} //end function

void UART_Baud_UP(){

  //Send the command to increase baud
  send_command(COMMAND_UART_MAX);

  //delay some time to allow 
  delay(3000);
  clear_Ser1_Buffers(); //get rid of potentially errored bytes

  //increase speed of our own serial
  Serial1.begin(921600);

  #ifdef TESTING
  Serial.println("Baud rate increased...");
  #endif
}

//check whether UART is currently 115k or 921k.
//returns false if 115.2k
//returns true if 921k.
boolean CheckUARTAt921(){

  //First Check for 115k
  Serial1.begin(115200);
  //Send the dummy command
  send_command(COMMAND_AT);
  //Delay a bit
  delay(1000);

  //If we do not get errored bytes, we are actually txing on 921600.
  if (Serial1.available() > 5) {
    
    //Change to 921200.
    UART_Baud_UP();
    //Change ourselves.
    Serial1.begin(921600);
    clear_Ser1_Buffers();
    
    return false;
  }

  #ifdef TESTING
  Serial.println("No change in Baud rate required. Already 921600.");
  #endif
  
  Serial1.begin(921600); //Change ourselves.
  clear_Ser1_Buffers(); //clear the buffers
  
  return true; //return true
}

//=====ADC FUNCTIONS=====

//read Temperature from ADC
unsigned int readTemp(){

  //sample
  unsigned int result = analogRead(TEMP_MONITOR);

  result = 100/((1024/result)-1);

  //map linearly
  if (result >= 36.0){ //high range
    result = (m_low * result) + c_low;
  }
  else {//low range
    result = (m_high * result) + c_high;
  }

  return result;
}

//read Battery Level from ADC
unsigned int readBatt(){

  //sample data
  unsigned int result = analogRead(BATTERY_MONITOR);

  result *= volt_multiplier; //map to voltage.
  result -= volt_bias; //drop the voltage by 0.4v.

  return result;
}

//=====MONITORING FUNCTIONS=====

//Send update to home base station
void SendUpdate(){

    //Write the sequence to the Wizfi
    Serial1.write(esc);
    Serial1.write('Z');
    Serial1.write('1'); //send home
    //Write Packet length
    Serial1.write('0');
    Serial1.write('0');
    Serial1.write('1');
    Serial1.write('0');
    //write data. 10 bytes
    Serial1.write(1); //LENGTH 1x10 bytes
    Serial1.write(PKT_UPDATE); //send the packet index.
    Serial1.write(NODE_NUMBER);//source
    Serial1.write((char)temperature);
    Serial1.write((char)batteryVoltage);
    Serial1.write(errorTraffic); //signal loss
    Serial1.write(lowTraffic); //low traffic: Link loss
    Serial1.write('0');
    Serial1.write('0');
    Serial1.write('0');
    
    //finish with newline
    Serial1.write(N_L);
}

//send urgent update to home base station
void SendUrgentUpdate(){
  
    //Write the sequence to the Wizfi
    Serial1.write(esc);
    Serial1.write('Z');
    Serial1.write('1'); //send home
    //Write Packet length
    Serial1.write('0');
    Serial1.write('0');
    Serial1.write('1');
    Serial1.write('0');
    //write data. 10 bytes
    Serial1.write(1); //LENGTH 1x10 bytes
    Serial1.write(PKT_URGENT); //send the packet index.
    Serial1.write(NODE_NUMBER);
    Serial1.write((char)temperature);
    Serial1.write((char)batteryVoltage);
    Serial1.write(errorTraffic); //signal loss
    Serial1.write(lowTraffic); //low traffic: Link loss
    Serial1.write('0');
    Serial1.write('0');
    Serial1.write('0');
    
    //finish with newline
    Serial1.write(N_L);
}

//=====End MONITORING FUNCTIONS=====

//TESTING - SEND CONSTANT BITRATE

//Send data uplink.
void send_test_data(){

  Serial.println();
  
  //CBR TRANSMIT VARIABLES.
  unsigned int test_data_len = 400; //200 bytes
  unsigned char test_data_len_char = 198; //198 40 bit blocks.
  char TxString[19] = "Sending 000 bytes.";
  unsigned char pkt_count = 'A';

  unsigned long timing = 0;

  //clear all buffers
  clear_Ser1_Buffers();

  char inByte = 0;

  char asdf = '1';

  while (1){
    if (Serial.available()){      
      inByte = Serial.read(); //check the next byte
      if (inByte != 10){
        asdf = inByte;
      }
    }

    Int2AsciiExt(test_data_len,length_buffer);
  
    TxString[8] = length_buffer[2];
    TxString[9] = length_buffer[1];
    TxString[10] = length_buffer[0];
    
    timing = millis();
    
    //Write the escape sequence to the WizFi
    Serial1.write(esc);
    Serial1.write('Z');
    //Write to CID: write to 2. +1
    #ifdef TEST_UP1
      Serial1.write('2');
    #else
      Serial1.write('1');
    #endif
    
    //Data length
    Serial1.write('0');
    Serial1.write(length_buffer[2]);
    Serial1.write(length_buffer[1]);
    Serial1.write(length_buffer[0]);

    test_data_len_char = test_data_len/10;
    Serial1.write(test_data_len_char); //send the packet char
    
    Serial1.write(pkt_count); //send the packet index.
    Serial1.write(asdf); //send the ID
    
    //loop and shoot data out.
    for (unsigned int i = 0; i < test_data_len-3; ++i){
      Serial1.write(dataBuff[i]);
    }
    
    //finish with newline
    Serial1.write(N_L);

    pkt_count++;

    if (pkt_count >'Z') pkt_count = 'A';

    delay(tx_delay);
    
    timing = millis() - timing;

    Serial.print("The tx time is: ");
    Serial.print(timing);
    Serial.print(" ms. ID: ");
    Serial.print(asdf,DEC);
    Serial.print(" ");
    Serial.print(TxString);
    Serial.println();

    clear_Ser1_Buffers();
  }
}

//rerun the setup of network functions.
void WizFiReboot(){
  
#ifdef TESTING
  Serial.println("REBOOTING...");
#endif

  HADESNET_Scan_Connect(); //connect to HADESNET
  SetHostIPAddr(); //Set self IP address
  
  NodeServerSetup(); //Setup our UDP server: GIVES CID 0
  NodeDownstreamSetup(); //Setup our link to home base station: GIVES CID 1
  NodeUpstreamSetup(); //Setup our link to HADES: GIVES CID 2
}

void readNodeDiagnostic(){
  
}

//=====SPI FUNCTION FOR ACCEL

//Send SPI Byte
char SPITransferData(char addr, char din){
  
  SPI.beginTransaction(SPISettings(2000000,MSBFIRST,SPI_MODE0));
  //CS low to select chip
  digitalWrite(ACCEL_CS,LOW);
  
  //Transfer the data over
  SPI.transfer(addr);
  char temp = SPI.transfer(din);
  
  //CS high to deselect chip
  digitalWrite(ACCEL_CS,HIGH);

  SPI.endTransaction();

  return temp;
}


