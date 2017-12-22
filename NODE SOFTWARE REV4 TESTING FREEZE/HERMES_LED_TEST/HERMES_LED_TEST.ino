//HERMES NODE CODE REV3 - TESTING LED
//By Jeffrey Wu

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
#define LED_CONNECTED 20 //ORANGE PWR_LED1
#define LED_POWER 21 //GREEN PWR_LED2

int PWM = 0;

//=====SETUP
void setup() {

  Serial.begin(1000000); //Setup serial connection to Arduino IDE
  
  pinMode(LED_CONNECTED, OUTPUT);
  pinMode(LED_POWER, OUTPUT);
  digitalWrite(LED_CONNECTED, LOW);
  digitalWrite(LED_POWER,LOW);
}

void loop() {

  //0-255.
  if (Serial.available() >= 4){

    PWM = 0;
    
    //get digits
    PWM += (Serial.read() - 48) * 100;
    PWM += (Serial.read() - 48) * 10;
    PWM += (Serial.read() - 48);

    //clear other serial
    while (Serial.available()) Serial.read();
  }
  
  Serial.println(PWM);
  
  analogWrite(LED_CONNECTED,PWM);
  analogWrite(LED_POWER,PWM);

  delay(100);
}

