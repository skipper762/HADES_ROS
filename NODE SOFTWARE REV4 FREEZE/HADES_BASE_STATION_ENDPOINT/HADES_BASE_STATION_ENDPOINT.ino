//HERMES NODE CODE REV4 - CORTEX ARM4/WIZFI220
//This is the Base Station Code
//By Jeffrey Wu

//WORKING COPY
//FULL IMPLEMENTATION

//=====ROS ENABLED =====

//BASE STATION TODO
//The base station needs to just send the packet upstream to CID1. As such, all Tx will carry CID1 and can be pre-processed on PC.
//The base station takes all incoming packets and sends them over serial. We do not need to Decompress so we will just let the PC do it.

//Include statements for ROS.
#include <ros.h>
#include <std_msgs/String.h>
#include <hades_base_stn/data_packet.h>
#include <hades_base_stn/control_packet.h>
#include <hades_base_stn/sensor_packet.h>
#include <string.h>
#include <stdlib.h>

//IP ADDRESS
//254: HADES
//1: BASE STATION
//All Other Numbers: Node

//Define which version of WizFi: 210 FOR TESTING ONLY.
#define WIZFI220
//#define WIZFI210
#define BUFF_SIZE_TO_PC 1024 //2kb

#define LED_POWER_ON 80

//#define DIGEST
//#define TESTING
#define IMPL

//output debug digests on Serial.
//delay on the transmit buffers
#define tx_delay 15

//command delay
#define inter_command_delay 300

//WIZFI2x0 PIN LAYOUTS
#define UART_CTS 3
#define UART_RTS 2
#define RESET 4
//#define HOST_SIG 6
//#define ACCEL_CS 9
//#define SPI_CS 10
//#define SPI_MOSI 11
//#define SPI_MISO 12
//#define SPI_CLK 13
#define CONNECTED_FB 14
#define COMMAND_REG 13

//LED ILLUMINATION
#define LED_CONNECTED 21
#define LED_POWER 20

//=====CODE BEGIN=====

//build the ROS node
ros::NodeHandle nh; //create handler of type ros::HodeHandle

//packets for publishers
hades_base_stn::data_packet incomingVideoData;
hades_base_stn::sensor_packet incomingSensorData;
hades_base_stn::control_packet incomingControlPacket;

ros::Publisher fromHADES("upstream_video",&incomingVideoData);
ros::Publisher fromHADESSensor("upstream_sensor",&incomingSensorData);
ros::Publisher fromHADESControl("upstream_control",&incomingControlPacket);

volatile int LED_STATE = 0;
volatile char setup_rdy = 0;

//CID CONSTANTS:
//- 0:Our UDP server: Recieved to Base Station
//- 1:Downstream to HADES

//Important constants
const unsigned char N_L = '\n'; //newline/Linefeed
const unsigned char C_R = '\r'; //return
const unsigned char esc = 27; //escape character for data transfer mode 0x33
const unsigned char delimiter = '.';

//AT commands
//Startup
const char COMMAND_AT[3] = "AT"; //AT: No operation
const char COMMAND_DISCO[6] = "AT+WD"; //disconnect //new
const char COMMAND_MAX_POWER[9] = "AT+WP=15"; //Set max power for WIZFI220
const char COMMAND_EXT_POWERAMP_ON[11] = "AT+EXTPA=1"; //Set ext power amplifier: Only for WIZFI220
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
char COMMAND_SET_SELF_IP[46] = "AT+NSET=10.42.0.001,255.255.255.0,10.42.0.001"; //set a string to be the IP address.
char COMMAND_UPLINK_UDP_CLIENT[26] = "AT+NCUDP=10.42.0.254,5000"; //set port to uplink

//NOT USED IN IMPLEMENTATION
//Serial Commands from host for testing
const unsigned char com_set = 's'; //setup
const unsigned char com_data_mode = 'd'; //prepare to get data out.
const unsigned char com_data_mode_exit = 'e'; //prepare to change settings.
const unsigned char com_netstat = 'n'; //get connection statistics
const unsigned char free_com_enter = 'y'; //free conneciton mode enter.
const unsigned char free_com_exit = 'x'; //free connection mode exit.

//===== START Function prototypes =====

//HOME STATION FUNCTIONS ONLY
void SendPackToPC(); //Send the Rx buffer to the PC.
void SendPackToHADES(); //send the Tx buffer to HADES

void SerialSetup(); //start the serial.

//OK output checks
boolean OK_Check(); //wait for OK signal

//Buffer/char output functions
char char_in_Ser1(); //Get a character and bounce it to the testing
void clear_Ser1_Buffers(); //clear the buffers
void clear_Ser_Buffers(); //clear the input buffers.

//WizFiCommands
void WizFiSetup(); //setup the WizFi module

void UART_Baud_UP(); //UART BAUD UP.
boolean CheckUART(int); //check UART BAUD RATE.

//Send a command down to WizFi220. NO CHECKS: OPEN LOOP
void send_command(const char*); //send data down on ser1 to wizfi220
void send_command(char*); //The same command but with char* instead of const char*
void command_delay(const char*); //output the command on Serial1 and delay some.

//WIFI/IP COMMANDS
void freeCommandMode(); //free command mode for free AT command output
void HADESNET_Scan_Connect(); //Continually scan for HADESNET and return once connection is achieved.

void SetHostIPAddr(); //Set self IP address.
void NodeServerSetup(); //Setup the node host UDP server.
void NodeDownstreamSetup(); //Setup data for downstream Base stn to HADES
void Ready_To_Rx(); //go into data mode

//UTILITY FUNCTIONS
void Int2AsciiExt(unsigned int,char*);//Assumes ptr points to 4 bytes of memory. Output limits are 0 to 999

//System runs on static IP addresses.
const char HADES_IP[13] = "10.42.0.254";
const char BASE_STN_IP[11] = "10.42.0.1"; //HOME IP ADDRESS

//Testing Variables
//if true, sends data back down on Serial.

//node num = 4/98
boolean auto_tx = false; //for auto tx: SET TRUE FOR TESTING. node num = 4/98
boolean auto_rx = true; //node num = 5/99

//=====OPERATION VARIABLES=====

char length_buffer[4] = "000";
unsigned char IP_HOST = 0;
char target_CID = 0;

//UART flag
boolean high_UART = false;

//=====BUFFERS=====
//holds buffered data to send to PC.
volatile unsigned char TO_PC[BUFF_SIZE_TO_PC];
//holds buffered data to send to HADES.
volatile unsigned char TO_HADES[BUFF_SIZE_TO_PC];

//=====TEST VARIABLES=====
unsigned char test_data_len = 200; //200 bytes
//Temporary Data buffer
volatile char dataBuff[990]; //get a char pointer

uint8_t rxBuff[BUFF_SIZE_TO_PC]; //2048 buffer.
volatile int rxPos = 0; // the reciever's position

//#####PROGRAM START#####

//we have a callback from base station.
void fromBase_callback(const hades_base_stn::control_packet& ctrl_pack){

  //we send a control packet over radio to HADES.
  if (setup_rdy == 1){ 
    
    if (LED_STATE == 0){
      LED_STATE = 1;
      analogWrite(LED_POWER,LED_POWER_ON);
    }
    else{
      LED_STATE = 0;
      analogWrite(LED_POWER,0);
    }
      
    //A packet called data_pack comes in, so we send the data down to the WizFi220.
    Serial1.write(esc);
    Serial1.write('Z'); //delimiters
    Serial1.write('1'); //to CID 1: HADES
  
    //10 bytes
    Serial1.write('0');
    Serial1.write('0');
    Serial1.write('1');
    Serial1.write('0');
    
    Serial1.write(ctrl_pack.data[0]);
    Serial1.write(ctrl_pack.data[1]);
    Serial1.write(ctrl_pack.data[2]);
    Serial1.write(ctrl_pack.data[3]);
    Serial1.write(ctrl_pack.data[4]);
    Serial1.write(ctrl_pack.data[5]);
    Serial1.write(ctrl_pack.data[6]);
    Serial1.write(ctrl_pack.data[7]);
    Serial1.write(ctrl_pack.data[8]);
    Serial1.write(ctrl_pack.data[9]);
    
    //send the newline
    Serial1.write(N_L);
  
  }
}
//this is the associated callback.
ros::Subscriber<hades_base_stn::control_packet> fromBase("downstream", fromBase_callback);

//Setup the various pins and system
void TeenseyPinSetup(){

  //Setup WizFi Reset line.
  pinMode(RESET,INPUT);

  //Setup WizFI UART connection inputs/outputs
  pinMode(UART_CTS, OUTPUT);
  pinMode(UART_RTS, INPUT);
  digitalWrite(UART_CTS, LOW); //clear to send WizFI220
  
  //Set WIZFI return signals to be input.
  pinMode(COMMAND_REG,INPUT);
  pinMode(CONNECTED_FB,INPUT);
  
  //Setup WizFi Reset line: Switch to output to drive.
  pinMode(RESET,OUTPUT);
  
  //reset the WizFi
  digitalWrite(RESET,LOW);
  delay(2000);
  digitalWrite(RESET,HIGH);
  delay(1000);
}

void setup() {

  //begin the Serial to WizFi
  Serial1.begin(115200);
  
  //Setup Teensey system on boot.
  TeenseyPinSetup();

  analogWrite(LED_POWER,LED_POWER_ON);
  
  #ifdef TESTING
    Serial.begin(1000000); //Setup to Arduino IDE: Output to COM port
    Serial.println("Press Any Key to Continue.");
    
    //wait until the signal.
    while (Serial.available() == 0);
    
    while (Serial.available() != 0) Serial.read(); //clear the byte.

    Serial.println("===HADES BASE STATION R4.0===");
  #else
    delay(200); //delay 3 seconds.
  #endif

  //=====WIZFI MODULE SETUP=====
  WizFiSetup(); //setup the WizFi Module through setting up commands.
  //=====END WIZFI MODULE SETUP=====
  
  #ifdef TESTING
  //=====TESTING SETUP=====
  //Setup the data buffers.
  for (int k = 0; k < test_data_len; ++k){
    dataBuff[k] = (k % 10) + '0'; //turn to char.
  }
  //=====END TESTING SETUP===== 
  #endif
  
  //=====IMPLEMENTATION=====
  #ifdef IMPL
    nh.initNode();

    //Subscribe and advertise to the ROS connections
    nh.subscribe(fromBase); //the controller commands from the base station.
    nh.advertise(fromHADES); //the video stream to be sent to base station
    nh.advertise(fromHADESSensor); //sensor message from HADES
    nh.advertise(fromHADESControl); //control message from HADES
  #endif
  //=====END IMPLEMENTATION=====

  #ifdef IMPL
  //go to main.
  Ready_To_Rx();
  #endif
}

//do nothing: everything is handled by callback functions.
void loop() {
  /*
  #ifdef IMPL
    nh.spinOnce();
  #endif

  #ifdef TESTING
  //=====TESTING=====
  Ready_To_Rx();

  freeCommandMode();
  //====END TESTING=====
  #endif
  */
} //end loop

//Setup the WizFi220 Module
//WORKING
void WizFiSetup(){

  //=====Start SETUP=====
  
  #ifdef TESTING
    Serial.println("Setup Start...");
  #endif
  
  send_command(COMMAND_AT);
  send_command(COMMAND_AT);
  
  send_command(COMMAND_DISCO);
  send_command(COMMAND_AD_HOC_ENABLE); 
  send_command(COMMAND_PASSWORD);
  send_command(COMMAND_MAX_POWER);
  send_command(COMMAND_BULK_MODE);
  send_command(COMMAND_EXT_POWERAMP_ON);
  
  //END CLAUSE CLEAR ALL BUFFERS AND WAIT A WHILE.
  delay(1000);
  #ifdef TESTING
    Serial.println("Setup Complete");
  #endif
  delay(2000);

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

  //Rx Server
  NodeServerSetup(); //Setup our UDP server: GIVES CID 0
  //Tx Client
  NodeUpstreamSetup(); //Setup our link to HADES: GIVES CID 1

  send_command(COMMAND_MAX_POWER);
  
  delay(3000);
  analogWrite(LED_CONNECTED,LED_POWER_ON);
  
  UART_Baud_UP(); //raise baud rate to max
  Serial1.begin(921600); //raise serial rate to match new baud.
  
  delay(500);

  setup_rdy = 1;

  //=====End SETUP =====
}

//CHECK THIS
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

        #ifdef TESTING
          Serial.write(data);
        #endif
      }
      return result;
    }

    //if we have a stable string, our command has finished.
    if (last_count == Serial1.available()) counter++;
    else counter = 0; //otherwise, we reset and wait until we are stable
    if (counter == 10) return false; //we timeout.
  }
  return false;
}

void HADESNET_Scan_Connect(){

  Serial1.begin(115200);
  
  #ifdef TESTING
    Serial.println("Connecting to HADESNET...");
  #endif
  
  //loop until we return (after getting the OK.)
  while (1){
    
    clear_Ser1_Buffers(); //first clear the buffers.
    //put the request in.
    send_command(COMMAND_CONNECT_HADESNET); //AT: CONNECT TO HADESNET
    delay(1500);
    
    if (OK_Check()){ //If we get an active OK
      
      #ifdef TESTING
        Serial.println();
        Serial.println("HADESNET connection obtained.");
      #endif
      
      break;//If we get a satisfactory response, break.  
    }
    #ifdef TESTING
      Serial.println("No HADESNET connection...Continue Scanning");
    #endif
  }

  //clear all buffers, we are connected
  clear_Ser1_Buffers();
  //delay a bit.
  delay(inter_command_delay);
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
      Serial.write(Serial1.read());//Mirror the data back to the Arduino if just testing.
    #else 
      Serial1.read();
    #endif
  }
}

//Clear the input buffers
void clear_Ser_Buffers(){
  while (Serial.available()){
    Serial.read(); //flush the buffers.
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
  //print the newline character
  Serial1.write(N_L);
  
  //delay
  delay(inter_command_delay);
}

//Send command down on serial 1.
void send_command(const char* command){

///===CURRENTLY OPEN LOOP: ONLY USE FOR GUARANTEED OUTPUTS: Works 100% of the time.

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
      
      if (inByte == free_com_exit) { //x
        Serial.println("Free Command Mode Exit.");
        break; //break if we recieve an escape sequence
      }
      else{
        Serial1.write(inByte); //send it to WIZFI
      }
    }// end if 
  }// end while
}

//Set self IP address.
void SetHostIPAddr() {
  #ifdef TESTING
    Serial.println("Setting up IP Address...");
  #endif

  //AT command: IP address set.
  send_command(COMMAND_SET_SELF_IP);

  delay(inter_command_delay);// delay 300ms
}

void NodeServerSetup(){ //Setup our UDP server
  #ifdef TESTING
    Serial.println("Setting up UDP server...");
  #endif
  //AT command: Start UDP local server on port 5000: This port is the same on all nodes.
  send_command(COMMAND_START_UDP_SERVER);

  delay(inter_command_delay); //delay 300ms.
}

void NodeUpstreamSetup(){ //Setup our link to HADES.
  #ifdef TESTING
    Serial.println("Setting up upstream line(TO HADES)...");
  #endif
  //AT command: UDP client to upstream node to port 5000
  //add +1 to our IP address and forward
  //Then get handle and store in variable
  
  //char next_node_ASCII[4] = "000";
  
  //Int2AsciiExt(1,next_node_ASCII);

  //COMMAND_UPLINK_UDP_CLIENT[17] = next_node_ASCII[2];
  //COMMAND_UPLINK_UDP_CLIENT[18] = next_node_ASCII[1];
  //COMMAND_UPLINK_UDP_CLIENT[19] = next_node_ASCII[0];
  
  send_command(COMMAND_UPLINK_UDP_CLIENT); //send the command
  
  delay(inter_command_delay); //delay 300ms.
}

//poll the serial buffers and wait for data.
//Prerequisits: We are already setup for data transmission.
//Needs:
//- Assosiation to HADESNET
//- IP setup correctly
//- CID setup
void Ready_To_Rx(){

  //=====PACKET RECIEVER FUNCTION=====

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
  unsigned char pack_type = 0; //the type of packet incoming

  //The input byte from Serial.
  unsigned char inByte = 0;

  //loop variable.
  int loop_v = 0;

  //bytes encountered
  int bytes_encountered = 0;

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

    //spin once.
    nh.spinOnce();
    
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

          //do 2nd digit
          inByte = Serial1.read(); //try for tens.
          
          //do 3rd digit
          inByte = Serial1.read(); //try for hundreds.
          
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

          //first inByte from while loop.
  
          //error check
          while (inByte == C_R || inByte == N_L) inByte = Serial1.read();
          
          //The first byte is the length byte divided down by 10, so we can grab it and put out the length.
          pack_len = inByte * 10; //multiply by 10.

          //store the length
          rxBuff[0] = inByte;
          incomingVideoData.data[0] = inByte;

          #ifdef DIGEST
          //print the packet length.
            Serial.println();

            Serial.print(pack_len,DEC);
            Serial.println();
          #endif

          //packet length error check
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
          
          while (Serial1.available() < 3);
          
          inByte = Serial1.read();

          while (inByte == C_R || inByte == N_L) inByte = Serial1.read(); //drop the new data if it is an illegal character

          //THIS IS PACKET TYPE
          pack_type = inByte; //the type of packet determines what message to send.

           //store the packet type
          rxBuff[1] = pack_type;
          incomingVideoData.data[1] = pack_type;

          DROP = pack_type;
            
          //get the next packet.
          inByte = Serial1.read();
          
          while (inByte == C_R || inByte == N_L) inByte = Serial1.read(); //drop the new data if it is an illegal character

          //THIS IS ID TYPE: not used in implementation

          //store the ID.
          rxBuff[2] = inByte; //UDP type
          incomingVideoData.data[2] = inByte;

          rxPos = 2; //set the rx position to the start of the packet.
          
          bytes_encountered = 3; //3 bytes encountered thus far. Can reduce to 2 by eliminating the drop testing.
          
          while (true){ //new while
            
            if (bytes_encountered == pack_len){ //if we reach the end of the packet.
              set_flags = 0x00; //clear the flag

              //put the last packet into the stack
              rxBuff[rxPos++] = inByte;

              //NOW WE DECIDE WHAT TO DO WITH THE PACKET.
              //VIDEO PACKET
              if (pack_type == 'X' || pack_type == 'V' || pack_type == 'K'){
                //video packet
                //set video size
                incomingVideoData.pack_size = 50;

                //plug the data in.     
                //Send the completed packet to HADES.
                for (loop_v = 3; loop_v < rxPos; ++loop_v){
                  
                    incomingVideoData.data[loop_v] = rxBuff[loop_v];
                }
                
                //incomingVideoData.data = (uint8_t*)rxBuff;

                //send the data to the base station endpoint.
                #ifdef IMPL
                  fromHADES.publish(&incomingVideoData);
                #endif
  
                rxPos = 0; //reset the rx position 
              }
              else if (pack_type == 'S'){
              //=====Sensor reading packet=====
                incomingSensorData.type = 6;
                
                //Transmit reserved field
                for (int i = 0; i < 9; ++i){
                  incomingSensorData.reserved[i] = rxBuff[i];
                }
            
                //Transmit sensor type
                for (int i = 0; i < 10; ++i){
                  incomingSensorData.sensor_type[i] = rxBuff[i];
                }
            
                int temp = 0; //temp var
                //Transmit sensor data.
                for (int i = 0; i < 40; i = i + 4){
                  temp = 0;
                  temp += rxBuff[i];
                  temp = temp << 8;
                  
                  temp += rxBuff[i + 1];
                  temp = temp << 8;
                  
                  temp += rxBuff[i + 2];
                  temp = temp << 8;
                  
                  temp += rxBuff[i + 3];
                  temp = temp << 8;

                  
                  incomingSensorData.data[i] = temp;
                }

                #ifdef IMPL
                fromHADESSensor.publish(&incomingSensorData);
                #endif
              }
              /*
              else if (pack_type =='A'){
                //=====audio packet=====
                //audio packet not yet implemented
              }
              */
              else if (pack_type == 'C'){
              //=====Control Packet=====

                //just plug the data in. Always 10 bytes
                for (int i = 0; i < 10; ++i){
                 incomingControlPacket.data[i] = rxBuff[i];
                }

                //now publish the data.
                #ifdef IMPL
                fromHADESControl.publish(&incomingControlPacket);
                #endif
              }
              
              analogWrite(LED_CONNECTED,LED_POWER_ON); //set Orange LED ON
              
              break; //break while true
              
            }
            else{ //push the data into the array.
                rxBuff[rxPos++] = inByte;
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
          #endif

          //RESET VAR
          ERR = 0;
          DROP = 0;
          test = '0';
          //reset the byte counters.
          pack_len = 0;
          bytes_encountered = 0;
          
          rxPos = 0;
          
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

//sends a packet back down on serial.
void SendPackToPC(){

  //Send Packet to PC over Serial.
  
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

  Serial.println("No change in Baud rate required. Already 921600.");
  
  //Change ourselves.
  Serial1.begin(921600);
  
  clear_Ser1_Buffers();
    
  return true;
}


//Send test data somewhere.
void send_test_data(){

  Serial.println();

  unsigned long timing = 0;

  //clear all buffers
  clear_Ser1_Buffers();
  clear_Ser_Buffers();

  char inByte = 0;

  //CBR TRANSMIT VARIABLES.
  unsigned int test_data_len = 300; //200 bytes
  unsigned char test_data_len_char = 198; //198 40 bit blocks.
  //char TxString[19] = "Sending 000 bytes.";
  unsigned char pkt_count = 'A';
  char asdf = '1';

  //char throttle_counter = 0;

  while (1){
    if (Serial.available()){      
      inByte = Serial.read(); //check the next byte
      if (inByte != 10){
        asdf = inByte;
      }
    }

    Int2AsciiExt(test_data_len,length_buffer);
    
    timing = millis();
    
    //Write the escape sequence to the WizFi
    Serial1.write(esc);
    Serial1.write('Z');
    //Write to CID: write to 2. +1
    Serial1.write('2');

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
    Serial.println();

    clear_Ser1_Buffers();
  }
}

//rerun the setup of network functions.
void WizFiReboot(){
#ifdef TESTING
  Serial.println("REBOOTING...");
#endif

  send_command(COMMAND_DISCO);
  HADESNET_Scan_Connect(); //connect to HADESNET
  SetHostIPAddr(); //Set self IP address
  
  NodeServerSetup(); //Setup our UDP server: GIVES CID 0
  NodeDownstreamSetup(); //Setup our link to home base station: GIVES CID 1
  NodeUpstreamSetup(); //Setup our link to HADES: GIVES CID 2
}
