//HERMES accelerometer trigger test code R1.0

#include <SPI.h>

#define ACCEL_CS 9

//axis address modes
#define X_LOW 0xA8
#define X_HIGH 0xA9
#define Y_LOW 0xAA
#define Y_HIGH 0xAB
#define Z_LOW 0xAC
#define Z_HIGH 0xAD

//control register address modes
#define CTRL_REG4 0x20
#define CTRL_REG1 0x21
#define CTRL_REG2 0x22
#define CTRL_REG3 0x23
#define CTRL_REG5 0x24

//=====TEST VARIABLES=====
long x_accel = 0;
long y_accel = 0;
long z_accel = 0;

//rolling average filter.
int x_buff[10] = {0,0,0,0,0,0,0,0,0,0};
int y_buff[10] = {0,0,0,0,0,0,0,0,0,0};
int z_buff[10] = {0,0,0,0,0,0,0,0,0,0};

// variable position
char v_pos = 0;

char high_byte = 0;
char low_byte = 0;

//SETUP THE SPI data LINK
void SPISetup(){
  
  //Begin the transaction
  SPI.begin();
  
  //CHIP SELECT
  pinMode(ACCEL_CS, OUTPUT); //set pin to be chip select.
  
  //set the chip select high.
  digitalWrite(ACCEL_CS, HIGH);
  
  Serial.println("SPI online...");

  //control register 4
  SPITransferData(CTRL_REG4,0x97);
}

//=====SETUP
void setup() {

  Serial.begin(1000000); //Setup to Arduino IDE
  Serial.println("===HERMES ACCELEROMETER TESTING===");
  
  SPISetup(); //setup the SPI data transfer
}

//====LOOP
void loop() {

  //get data x
  low_byte = SPITransferData(X_LOW,0x00);
  high_byte = SPITransferData(X_HIGH,0x00);

  x_buff[v_pos] = (high_byte << 8) + low_byte;
  
  //get data y
  low_byte = SPITransferData(Y_LOW,0x00);
  high_byte = SPITransferData(Y_HIGH,0x00);

  y_buff[v_pos] = (high_byte << 8) + low_byte;

  //get data z
  low_byte = SPITransferData(Z_LOW,0x00);
  high_byte = SPITransferData(Z_HIGH,0x00);
  
  z_buff[v_pos] = (high_byte << 8) + low_byte;

  ++v_pos;

  if (v_pos == 10) v_pos = 0; //reset v position

  for (int k = 0; k < 10; ++k){
    x_accel += x_buff[k];
    y_accel += y_buff[k];
    z_accel += z_buff[k];
  }

  x_accel /= 10;
  y_accel /= 10;
  z_accel /= 10;

  Serial.print("X_ACCEL: ");
  Serial.print(x_accel);
  Serial.print(" Y_ACCEL: ");
  Serial.print(y_accel);
  Serial.print(" Z_ACCEL: ");
  Serial.print(z_accel);
  Serial.println();

  delay(100);
  
}

//=====SPI FUNCTIONS

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
