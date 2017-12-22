//HERMES NODE CODE REV5 - CORTEX M4/WIZFI220
//NOT FROZEN
//This is the HADES code
//By Jeffrey Wu

//WORKING COPY
//====ROS ENABLED=====

//=====COMPILER DIRECTIVES=====
//Include statements for ROS.

#include <ros.h>
#include <std_msgs/String.h>
#include <hades_com_sys/data_packet.h>
#include <hades_com_sys/sensor_packet.h>
#include <hades_com_sys/control_packet.h>

#include <string.h>
#include <stdio.h>

#define WIZFI220
//#define WIZFI210

#define LED_POWER_ON 60

#define BUFF_SIZE_TO_PC 2048 //2kb
#define BUFF_SIZE_TO_HADES 2048 //2kb

//IP ADDRESS
//254: HADES
//1: BASE STATION
//All Other Numbers: Node

//output debug digests on Serial.
//#define TESTING
//#define TEST_FREE
//#define TESTRSSI
//#define DIGEST
#define IMPL
#define HADES_TEST_HOST_IP 8
#define NEXT_TEST_HOST_IP 3
//delay on the transmit buffers
//#define tx_delay 12
//#define tx_delay 400
//#define tx_pkt_size 450


#ifdef TESTRSSI
  #define tx_delay 500
  #define tx_pkt_size 400
#else
  #define tx_delay 14
  #define tx_pkt_size 450
#endif

//settings old 300byte at 7-5ms delay
//30/10/17 450byte at 12ms delay
//

//command delay 
#define inter_command_delay 300

//WIZFI2x0 PIN LAYOUTS
#define UART_CTS 8
#define UART_RTS 3
#define RESET 4
#define HOST_SIG 6
#define ACCEL_CS 9
#define SPI_CS 10
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_CLK 13
#define CONNECTED_FB 14
#define COMMAND_REG 13

//LED INDICATORS
#define LED_CONNECTED 21
#define LED_POWER 20

//=====CODE BEGIN=====

// The circuit:
// * RX is digital pin 19 (connect to TX of other device)
// * TX is digital pin 18 (connect to RX of other device)

//CID CONSTANTS:
//- 0:Our UDP server to HADES
//- 1:Downstream to base station

//Important constants
const unsigned char N_L = '\n'; //newline/Linefeed
const unsigned char C_R = '\r'; //return
const unsigned char esc = 27; //escape character for data transfer mode 0x33
const unsigned char delimiter = '.';

//AT commands
//Startup
const char COMMAND_AT[3] = "AT"; //AT: Prep to send.
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

//Variable AT commands: USES IP.
#ifdef TESTING
  //TESTING
  char COMMAND_SET_SELF_IP[46]; //set a string to be the IP address.
  char COMMAND_DOWNLINK_UDP_CLIENT[26]; //Set port to uplink
#else
  //implementation
  char COMMAND_SET_SELF_IP[46] = "AT+NSET=10.42.0.254,255.255.255.0,10.42.0.001"; //set a string to be the IP address.
  char COMMAND_DOWNLINK_UDP_CLIENT[26] = "AT+NCUDP=10.42.0.001,5000"; //Set port to uplink
#endif

//NOT USED IN IMPLEMENTATION
//Serial Commands from host for testing
const unsigned char com_set = 's'; //setup
const unsigned char com_data_mode = 'd'; //prepare to get data out.
const unsigned char com_data_mode_exit = 'e'; //prepare to change settings.
const unsigned char com_netstat = 'n'; //get connection statistics
const unsigned char free_com_enter = 'y'; //free conneciton mode enter.
const unsigned char free_com_exit = 'x'; //free connection mode exit.

//====ROS VARIABLES=====

//build the ROS node
ros::NodeHandle nh;

hades_com_sys::data_packet incomingData; //A packet to go to HADES
hades_com_sys::control_packet outgoingControlData; //A control packet to go to Base Station

//build the publisher to send a packet to HADES over USB.
ros::Publisher toHADES("downstream",&outgoingControlData);

int transmit_length = 0; //transmit size
volatile char setup_rdy = 0; //flag for the ready setup
int LED_STATE = 0;
char LED_STATE_2 = 0;


//====END ROS VARIABLES=====

//===== START Function prototypes 

void SerialSetup(); //start the serial.

//OK output checks
boolean OK_Check(); //wait for OK signal

//Buffer/char output functions
char char_in_Ser1(); //Get a character and bounce it to the testing
void clear_Ser1_Buffers(); //clear the buffers

//WizFiCommands
void WizFiSetup(); //setup the WizFi module

//Send a command down to WizFi220.
void send_command(const char*); //send data down on ser1 to wizfi220
void send_command(char*); //The same command but with char* instead of const char*
void command_delay(const char*); //output the command on Serial1 and delay some.

//WIFI/IP COMMANDS
void freeCommandMode(); //free command mode for free AT command output
void HADESNET_Scan_Connect(); //Continually scan for HADESNET and return once connection is achieved.

void SetHostIPAddr(); //Set self IP address.
void NodeServerSetup(); //Setup the node host UDP server.
void NodeDownstreamSetup(); //Setup data for upstream HADES to Base stn

void Ready_To_Rx(); //go into data mode

void send_test_data();
void UART_Baud_UP();

//UTILITY FUNCTIONS
void Int2AsciiExt(unsigned int,char*);//Assumes ptr points to 4 bytes of memory. Output limits are 0 to 990

//System runs on static IP addresses.
const char HADES_IP[13] = "10.42.0.254"; //The IP Address
const char BASE_STN_IP[11] = "10.42.0.2";

//=====OPERATION VARIABLES=====

char length_buffer[4] = "000";
unsigned char IP_HOST = 0;
char target_CID = 0;

//UART flag
boolean high_UART = false;

//=====BUFFERS=====
//holds buffered data to send to PC.
volatile unsigned char TO_BASE[BUFF_SIZE_TO_PC];
//holds buffered data to send to HADES.
volatile unsigned char TO_HADES[BUFF_SIZE_TO_HADES];

volatile unsigned char TxBuff[1024]; //set up transmit buffer.
volatile unsigned int TxPos = 0;

volatile char dataBuff[990]; //get a char pointer

uint8_t rxBuff[BUFF_SIZE_TO_PC]; //2048 buffer.
volatile int rxPos = 0; // the reciever's position

//#####PROGRAM START#####

//Setup the various pins and system
void TeenseyPinSetup(){
  //Setup WizFi Reset line.
  pinMode(RESET,INPUT);
  
  //Setup Illumination LEDs
  pinMode(LED_CONNECTED, OUTPUT); //This determines the LED that blinks indicating data transfer
  pinMode(LED_POWER, OUTPUT); //This illuminates the LED that lights after setup.
  analogWrite(LED_CONNECTED,0);
  analogWrite(LED_POWER,100);
  
  //Setup WizFI UART connection inputs/outputs
  pinMode(UART_CTS, OUTPUT);
  pinMode(UART_RTS, INPUT);
  digitalWrite(UART_CTS, LOW);
  
  //Set WIZFI return signals to be input.
  pinMode(COMMAND_REG,INPUT);
  pinMode(CONNECTED_FB,INPUT);

  //Setup WizFi Reset line: Switch to output to drive.
  pinMode(RESET,OUTPUT);
  digitalWrite(RESET,LOW);
  delay(2000);
  digitalWrite(RESET,HIGH);
  delay(1000);
}

//CALLBACK TO send a packet out immidiately.
void toBase_callback(const hades_com_sys::data_packet& data_pack){

  if (setup_rdy == 1){

    analogWrite(LED_POWER,0);
    
    transmit_length = data_pack.pack_size * 10; //get the packet
    transmit_length += 4; //4 bytes added for the length field.
    
    //We send the data down to the WizFi220.
    Serial1.write(esc);
    Serial1.write('Z');
    Serial1.write('1');

    //loop over all of the packets and transmit the data.
    for (int i = 0; i < transmit_length; ++i){
      Serial1.write(data_pack.data[i]);
    }
  
    //send the newline character
    Serial1.write(N_L);

    //delay a bit to prevent overload on WizFi.
    delay(12);

    analogWrite(LED_POWER, LED_POWER_ON);

  }
  
}
//this is the associated callback.
ros::Subscriber<hades_com_sys::data_packet> fromBase("upstream", toBase_callback);

//CALLBACK TO send a packet out immidiately.
void toBaseSensor_callback(const hades_com_sys::sensor_packet& sensor_pack){
  
    if (setup_rdy == 1){

    if (LED_STATE == 1){
      LED_STATE = 0;
      analogWrite(LED_POWER, LED_POWER_ON);
    }
    else{
      LED_STATE = 1;
      analogWrite(LED_POWER,0);
    }
    
    //We send the data down to the WizFi220.
    Serial1.write(esc);
    Serial1.write('Z');
    Serial1.write('1');

    //send 60 bytes
    Serial1.write('0');
    Serial1.write('0');
    Serial1.write('6');
    Serial1.write('0');

    //1
    Serial1.write(6); //character 6 for sensor 60 bytes

    //9
    //Transmit reserved field
    for (int i = 0; i < 9; ++i){
      Serial1.write(sensor_pack.reserved[i]);
    }

    //10
    for (int i = 0; i < 10; ++i){
      Serial1.write(sensor_pack.sensor_type[i]);
    }

    int temp = 0; //temp var
    //40
    for (int i = 0; i < 10; ++i){
      temp = sensor_pack.data[i];
      Serial1.write((char)temp);
      Serial1.write((char)(temp >> 8));
      Serial1.write((char)(temp >> 16));
      Serial1.write((char)(temp >> 24));
    }
  
    //send the newline character
    Serial1.write(N_L);

    //send a mirror signal
    //toHADES.publish(&outgoingControlData);

  }
}
//this is the associated callback.
ros::Subscriber<hades_com_sys::sensor_packet> fromBaseSensor("upstream_sensor", toBaseSensor_callback);


//CALLBACK TO send a packet out immidiately.
void toBaseControl_callback(const hades_com_sys::control_packet& ctrl_pack){
  //currently do nothing
}
//this is the associated callback.
ros::Subscriber<hades_com_sys::control_packet> fromBaseControl("upstream_control", toBaseControl_callback);

void setup() {
  
  //Setup Teensey system on boot.
  TeenseyPinSetup();

  //change to RosNode.
  #ifdef TESTING
      
    Serial.begin(1000000); //Setup serial connection to Arduino IDE
    
    //set the IP addresses straight
    sprintf(COMMAND_SET_SELF_IP,"AT+NSET=10.42.0.%i,255.255.255.0,10.42.0.001",HADES_TEST_HOST_IP);
    sprintf(COMMAND_DOWNLINK_UDP_CLIENT, "AT+NCUDP=10.42.0.%i,5000",NEXT_TEST_HOST_IP);
    
    Serial.println(COMMAND_SET_SELF_IP);
    Serial.println(COMMAND_DOWNLINK_UDP_CLIENT);
    
    Serial.println("Press Any Key to Continue.");

    //wait until the signal.
    while (Serial.available() == 0);
    
    Serial.read(); //clear the byte.

    Serial.println("===HADES' ENDPOINT R4.0===");
  #else
    delay(500);
  #endif

  //START THE SERIAL TO THE WIZFI
  Serial1.begin(115200);


  //=====WIZFI MODULE SETUP=====
  WizFiSetup(); //setup the WizFi Module through setting up commands.
  //=====END WIZFI MODULE SETUP=====
  
  //If working as a ROS node
  #ifdef IMPL
    //initialize the node
    nh.initNode();

    //Subscribe and advertise the ROS connections
    nh.subscribe(fromBase); //upstream link
    nh.advertise(toHADES); //downstream link
    nh.subscribe(fromBaseControl); //upstream link control signals
    nh.subscribe(fromBaseSensor); //upstream link sensor readings
  #endif
  //END ROS NODE SETUP
  
  //FOR IMPLEMENTATION
  #ifdef IMPL
    //TODO: SEE IF WE CAN GENERATE INSTRUCTIONS HERE.
    Ready_To_Rx();
    //just SpinOnce and do nothing else.
    //nh.spinOnce();
  #endif
}

//LOOP: We have properly set up the WizFi220 Unit.
//Now we just listen to the commands.
void loop() {

  //FOR TESTING
  #ifdef TESTING
    //=====TESTING SETUP=====
    #ifdef TEST_FREE
      freeCommandMode();
    #endif
    send_test_data();
    //=====END TESTING SETUP=====
  #endif
  
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
  send_command(COMMAND_DHCP);
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
  NodeDownstreamSetup(); //Setup our link to home base station: GIVES CID 1
  
  UART_Baud_UP(); //raise baud rate to max
  Serial1.begin(921600); //raise serial rate to match new baud.

  //delay for the state to settle.
  delay(1000);

  clear_Ser1_Buffers(); //buffers = 0; Clear out dead bytes to prevent propagation into system.

  send_command(COMMAND_MAX_POWER);

  //delay again for the state to settle
  delay(1000);

  analogWrite(LED_CONNECTED,LED_POWER_ON); //enable the connected LED

  setup_rdy = 1; //set the register flag.
  
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

    if (last_count == Serial1.available()) counter++;
    else counter = 0;
    if (counter == 10) return false;
  }
  return false;
}


void HADESNET_Scan_Connect(){

  
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
      Serial.println();
      #endif
  
      break;//If we get a satisfactory response, break.  
    }
    #ifdef TESTING
      Serial.println("No HADESNET connection...Continue Scanning");
    #endif
  //}
  

  //clear all buffers, we are connected
  clear_Ser1_Buffers();
  
  delay(500);
 }
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

void NodeDownstreamSetup(){ //Setup our link to home base station
  #ifdef TESTING
    Serial.println("Setting up downstream line(TO BASE STN)...");
  #endif

  send_command(COMMAND_DOWNLINK_UDP_CLIENT); //send the command
  
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

          //analogWrite(LED_CONNECTED,0); //set Orange LED OFF
          
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
          //outgoingControlData.data[0] = inByte;

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
          //outgoingControlData.data[1] = pack_type;

          DROP = pack_type;
            
          //get the next packet.
          inByte = Serial1.read();
          
          while (inByte == C_R || inByte == N_L) inByte = Serial1.read(); //drop the new data if it is an illegal character

          //THIS IS ID TYPE: not used in implementation

          //store the ID.
          rxBuff[2] = inByte; //UDP type
          //outgoingControlData.data[2] = inByte;

          rxPos = 2; //set the rx position to the start of the packet.
          
          bytes_encountered = 3; //3 bytes encountered thus far. Can reduce to 2 by eliminating the drop testing.
          
          while (true){ //new while
            
            if (bytes_encountered == pack_len){ //if we reach the end of the packet.
              set_flags = 0x00; //clear the flag

              //put the last packet into the stack
              rxBuff[rxPos++] = inByte;

              //NOW WE DECIDE WHAT TO DO WITH THE PACKET.
              if (pack_type == 'C'){
              //=====Control Packet=====

                //just plug the data in. Always 10 bytes
                for (int i = 0; i < 10; ++i){
                 outgoingControlData.data[i] = rxBuff[i];
                }

                //now publish the data.
                toHADES.publish(&outgoingControlData);
                
              }

              if (LED_STATE_2 == 0){
                analogWrite(LED_CONNECTED,LED_POWER_ON); //set Orange LED ON
                LED_STATE_2 = 0;
              }
              else{
                analogWrite(LED_CONNECTED,0); //set Orange LED OFF
                LED_STATE_2 = 1;
              }
              
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
  
  //Change ourselves.
  Serial1.begin(921600);
  
  clear_Ser1_Buffers();
    
  return true;
}

//Raise the Baud rate of the device.
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


//Send data uplink.
void send_test_data(){

  Serial.println();

  char data_on = 1;
  char one_pack = 0;
  
  unsigned long timing = 0;

  //clear all buffers
  clear_Ser1_Buffers();
  
  char inByte = 0;
  
  //CBR TRANSMIT VARIABLES.
  int test_data_len = tx_pkt_size; //200 bytes
  unsigned char test_data_len_char = 0; //80 bit blocks.
  
  //Setup the data buffers.
  for (int k = 0; k < test_data_len; ++k){
    dataBuff[k] = (k % 10) + '0'; //turn to char.
  }

  //char TxString[19] = "Sending 000 bytes.";
  unsigned char pkt_count = 'A';
  
  char asdf = '1';

  while (1){
    
    if (Serial.available()){      
      inByte = Serial.read(); //check the next byte
      if (Serial.available() == 4){
        test_data_len = 0;
        
        test_data_len += (Serial.read() -48) *100;
        test_data_len += (Serial.read() -48) *10;
        test_data_len += (Serial.read() -48);

        while (Serial.available()) Serial.read();
        
      }
      if (inByte == 'x'){ //turn transmission off
        data_on = 0;
      }
      else if (inByte == 'y'){ //turn transmission on
        data_on = 1;
      }
      else if (inByte == 'j'){ //Send just one packet.
        one_pack = 1;
      }
      else if (inByte == 'z'){
        break;
      }
      else if (inByte != 10){
        asdf = inByte;
      }
    }

    if ((data_on == 0) && (one_pack == 0)){
      continue;
    }

    one_pack = 0;

    Int2AsciiExt(test_data_len,length_buffer);
  
    //TxString[8] = length_buffer[2];
    //TxString[9] = length_buffer[1];
    //TxString[10] = length_buffer[0];
    
    timing = millis();
    
    //Write the escape sequence to the WizFi
    Serial1.write(esc);
    Serial1.write('Z');
    //Write to CID: write to 2. +1
    //CHECK THIS.
    Serial1.write('1');

    //Data length
    Serial1.write('0');
    Serial1.write(length_buffer[2]);
    Serial1.write(length_buffer[1]);
    Serial1.write(length_buffer[0]);

    test_data_len_char = test_data_len/10; //divide by 10.
    Serial1.write(test_data_len_char); //send the packet char
    
    Serial1.write(pkt_count); //send the packet index.
    Serial1.write(asdf); //send the ID
    
    //loop and shoot data out.
    for (int i = 0; i < test_data_len-3; ++i){
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
    Serial.print(" PKT_SIZE: ");
    Serial.print(test_data_len, DEC);
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
}
