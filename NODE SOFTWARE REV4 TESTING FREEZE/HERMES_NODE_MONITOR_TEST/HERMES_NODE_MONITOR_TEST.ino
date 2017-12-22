//BATTERY VOLTAGE/TEMP MONITOR TEST CODE

//VOLTAGE DIVIDER MONITORS
#define TEMP_MONITOR A9
#define BATTERY_MONITOR A8

#define m_high (-1.1)
#define c_high (78.833)

#define m_low (-0.4)
#define c_low (53.377)

//#define volt_Multiplier 0.0159
#define volt_multiplier 0.017
#define volt_bias 0.404

unsigned int BATT_VOLT = 0;
unsigned int TEMP_VOLT = 0;

float BATTERY_VOLTAGE = 0.0;
float TEMPERATURE = 0.0;

float THERMISTOR = 0.0;

 int BATT_VEC[10] = {0,0,0,0,0,0,0,0,0,0};
 int TEMP_VEC[10] = {0,0,0,0,0,0,0,0,0,0};

unsigned char v_pos = 0;

//=====SETUP=====
void setup() {

  Serial.begin(1000000); //start serial.

}

void loop() {

  BATTERY_VOLTAGE = 0.0;
  TEMPERATURE = 0.0;
  
  BATT_VEC[v_pos] = analogRead(BATTERY_MONITOR);
  TEMP_VEC[v_pos] = analogRead(TEMP_MONITOR); //get raw data.

  ++v_pos;

  if (v_pos == 10) v_pos = 0;

  for (int i = 0; i < 10; ++i){
    BATTERY_VOLTAGE += BATT_VEC[i];
    TEMPERATURE += TEMP_VEC[i];
  }

  //scale down
  BATTERY_VOLTAGE /= 10.0;
  TEMPERATURE /= 10.0;

  //BATTERY_VOLTAGE = (BATTERY_VOLTAGE*3.3*1.15)/(1024*0.233);
  BATTERY_VOLTAGE *= volt_multiplier;
  BATTERY_VOLTAGE -= volt_bias;
  THERMISTOR = 100/((1024/TEMPERATURE)-1);
  //THERMISTOR = (TEMPERATURE*100)/(1024-TEMPERATURE);

  if (THERMISTOR >= 36.0){ //high range
    TEMPERATURE = (m_low * THERMISTOR) + c_low;
  }
  else {//low range
    TEMPERATURE = (m_high * THERMISTOR) + c_high;
  }

  //Serial.print("BATT: ");
  Serial.print(BATTERY_VOLTAGE,DEC);
  //Serial.print("V TEMP: ");
  //Serial.print(TEMPERATURE,DEC);
  //Serial.print(" degrees THERMISTOR: ");
  //Serial.print(THERMISTOR,DEC);
  //Serial.println(" kohms");
  Serial.println();
 
}



