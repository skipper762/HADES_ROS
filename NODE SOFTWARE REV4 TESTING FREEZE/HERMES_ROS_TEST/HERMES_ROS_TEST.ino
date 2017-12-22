/*
 * rosserial TEST WizFi220 for HERMES on ARM Cortex
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <hades_com_sys/data_packet.h>
#include <hades_com_sys/control_packet.h>

//build the ROS node
ros::NodeHandle nh;
//hades_com_sys::data_packet incomingData;
hades_com_sys::data_packet outgoingData;

ros::Publisher toHADES("downstream",&outgoingData);

int LED_STATE = 0;

//we have a callback to base.
void toBase_callback(const hades_com_sys::data_packet& data_pack){

  //TEST: just flash an LED
  if (LED_STATE == 0){
    LED_STATE = 1;
    digitalWrite(13,HIGH);
  }
  else{
    LED_STATE = 0;
    digitalWrite(13,LOW);
  }

  //For reals: a packet called data_pack comes in, so we send the data down to the WizFi220.
  //Serial1.write('Z');
  //Serial1.write('1');
  //Serial1.write('0');

  //now write the data out.
  //send the data to the WizFi

  //mirror the data
  outgoingData.data[0] = data_pack.data[0];

  //publish the mirror
  toHADES.publish(&outgoingData);
  
}
//this is the associated callback.
ros::Subscriber<hades_com_sys::data_packet> toBase("upstream", toBase_callback);

void setup()
{

  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

  //=====make test packet
  outgoingData.pack_size = 1024;
  for (int i = 0; i < 1024; i++){
    outgoingData.data[i] = (i % 26) + 'A';
  }
  //=====end make test packet.
  
  //start the node
  nh.initNode();
  
  //subscribe to the incoming
  nh.subscribe(toBase);
  //advertise the mirror
  nh.advertise(toHADES);
}

void loop()
{
  nh.spinOnce();
  //delay(1); //check whether we still need this.
}
