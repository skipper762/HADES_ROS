#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <fstream>
#include <hades_com_sys/data_packet.h>
#include <hades_com_sys/sensor_packet.h>
#include <hades_com_sys/control_packet.h>

//======SYSTEM VARIABLES======
//TESTING
#define test_data_len 500
#define video_transmit_rate 10

//major buffer sizes
#define upstream_buff 2048
#define downstream_buff 2048

//rate of sensor transmission
#define sensor_rate_counter 100

//Camera variables
#define camera_fps 30

//transmit video packets on WizFi.
#define video_out

//======END SYSTEM VARIABLES======

//class that handles the communication between HADES system and
//the node hardware.
class COM_OPERATOR
{
	//external access
	public:
		COM_OPERATOR();
		~COM_OPERATOR();
	
	//internal access
	private:

		void mainLoop(); //main loop
		void incomingCallBack(const hades_com_sys::control_packet input_data); //data is coming in from WizFi.
		void outgoingSensorPacketTransmit(); //transmit a sensor packet.
		void incomingSensorData(const hades_com_sys::sensor_packet input_data); //sensor data coming from external source.
		void Int2AsciiExt(unsigned int input, char * output); //utility function
		void videoCallback(const hades_com_sys::data_packet data_pack);

		//ROS subscribers and publishers
		ros::NodeHandle nh; // the main process node object		

		//WIZFI INTERFACE
		ros::Subscriber command_stream_sub; //gets control signals from WizFi
		ros::Publisher data_stream_pub; //sends data to WizFi
		ros::Publisher sensor_stream_pub;//sends sensor data to WizFi
		ros::Publisher gen_control_pub; //sends control signals to WizFi
		
		//NOT IMPLEMENTED/REST OF ROS INTERFACE: This node to other nodes.
		ros::Publisher command_stream_pub; //sends control commands to HADES.
		//ros::Publisher node_deployment_pub; //sends node deployment command to HADES.
		ros::Subscriber sensor_data_sub; //gets sensor data from other sources
		
		ros::Subscriber vid_data_sub; //video data

		//MESSAGES FOR PUBLISHERS
		hades_com_sys::data_packet vid_pack; //the video packet message to send to base
		//hades_com_sys::sensor_packet int_sensor_pack; //the sensor packet message to send to base from internals.
		hades_com_sys::sensor_packet ext_sensor_pack; //the sensor packet message to send to base from externals.
		hades_com_sys::control_packet gen_control_pack; //general messages from HADES to base control.
		
		//private variables
		int out_pack_len; //set data length
		char dataBuff[1024]; //the array of characters.
		char test_char = 'A';
		char int_str[4] = {33,33,33,0}; //string to hold integers.
		char test_packet[10] = {0,0,0,0,0,0,0,0,0,0}; //test packet
		char sensor_counter;
		char buffer_set;
		int pack_size_bytes;

};

//constructor
COM_OPERATOR::COM_OPERATOR()
{
	//=====setup=====
	
	sensor_counter = 0; //for sensor transmissions
	out_pack_len = test_data_len; //set length of data
	pack_size_bytes = 0;

	buffer_set = 0; //cleared buffers.

	//setup the buffer
	int k = 0;
	//loop and fill the buffers
	for (; k < 1024; ++k){
		dataBuff[k] = (char)(k % 256);
		//vid_pack.data[k] = dataBuff[k];
	}
	//set packet size to the actual data length
	out_pack_len /= 10; //450 to 45
	vid_pack.pack_size = out_pack_len;

	//init the general packet data.
	for (int i = 0; i < 10; ++i){
		gen_control_pack.data[i] = 0;		
	}
	
	//NOT IMPLEMENTED: setup sensor variables 
	//sensor packets are setup in ext_sensor_pack and int_sensor_pack.
	//these sensor packets are used to store the values of the sensor elements. The sensor packets are 60 bytes.
	//whichever callback contains the sensor elements will write to ext_sensor_pack and int_sensor_pack.

	//get the data control packet from the Wizfi	
	command_stream_sub = nh.subscribe<hades_com_sys::control_packet>("downstream", upstream_buff, &COM_OPERATOR::incomingCallBack,this); //setup the subscriber for the downstream output

	//send the data packet to the Wizfi
	data_stream_pub = nh.advertise<hades_com_sys::data_packet>("upstream",downstream_buff); //setup the publisher for upstream input.
	sensor_stream_pub = nh.advertise<hades_com_sys::sensor_packet>("upstream_sensor", 80); //setup publisher for usptream sensor
	gen_control_pub = nh.advertise<hades_com_sys::control_packet>("upstream_control",40); //setup publisher for general control output to base station
	
	command_stream_pub = nh.advertise<hades_com_sys::control_packet>("controller_output",40); //setup publisher for controller output.
	
	//setup the video callback subscriber
	vid_data_sub = nh.subscribe<hades_com_sys::data_packet>("video_processed",1024,&COM_OPERATOR::videoCallback,this);
	//setup sensor
	sensor_data_sub = nh.subscribe<hades_com_sys::sensor_packet>("sensor_input",80,&COM_OPERATOR::incomingSensorData,this);

	//run the main.
	mainLoop();
}

//destructor
//Nothing at this stage
COM_OPERATOR::~COM_OPERATOR(){}

//Callback for sensor data
void COM_OPERATOR::incomingSensorData(const hades_com_sys::sensor_packet input_data){

	//TODO: FOR LATER IMPLEMENTATION: FILL OUT int_sensor_pack and ext_sensor_pack.
	//NOT IMPLEMENTED: get sensor data and fill out the sensor data.
}

//data is coming out from HADES to base station
void COM_OPERATOR::incomingCallBack(const hades_com_sys::control_packet input_data){

	//mirror this packet to the video processing and the control system. Publish to upstream control.
	//Any node that wishes to see the ocmmand controls will need to subscribe to upstream_control.
	command_stream_pub.publish(input_data);
}

//callback when image comes in.
void COM_OPERATOR::videoCallback(const hades_com_sys::data_packet data_pack){
	
	data_stream_pub.publish(data_pack);
}

void COM_OPERATOR::outgoingSensorPacketTransmit(){ //transmit a sensor packet.

	//NOT IMPLEMENTED: WE NEED TO USE THE SENSOR CALLBACK TO FILL THE SENSOR PACKET.

	//FOR EXAMPLE (FOR TESTING ONLY)
	//the contents of this packet are just dummy data.
	//In practice, fill this packet with actual data.

	//format:

	//type is length = dec 60.
	//reserved
	//field 1: sensor data type 
	//field 2: source of sensor data
	//field 3: urgent flag
	//rest: reserved.
	//Sensor type denotes which sensor is being considered: the sensor type enumerates each sensor with a unique character.
	//Sensor data denotes the sensor data that is acutally being sent.
	//fill in all fields with null if not used.

	ext_sensor_pack.type = 6; //TYPE is actually the length field. DEC 60
	ext_sensor_pack.reserved[0] = 'S'; //type for sensor data.
	
	for (int i = 1; i < 9; ++i){
		ext_sensor_pack.reserved[i] = i + 'a';
	}

	for (int i = 0; i < 10; ++i){
		ext_sensor_pack.sensor_type[i] = i + 'A';
		ext_sensor_pack.data[i] = i;
	}
	
	//publish the sensor packet
	sensor_stream_pub.publish(ext_sensor_pack);
}

//we are stuck in this main loop
void COM_OPERATOR::mainLoop(){

	ros::Rate k(video_transmit_rate);
	//while TRUE loop
	while (ros::ok()){
		
		/*
		//increment the sensor counter.
		++sensor_counter;
		//send the sensor data
		if (sensor_counter == sensor_rate_counter){ //every 5 seconds
			sensor_counter = 0; //reset the counter.
			//transmit the sensor packet.
			outgoingSensorPacketTransmit();
		}
		*/		

		//maintain the rate.
		ros::spinOnce(); //do events
		k.sleep(); //wait to maintain 20Hz
	}
}

//utility function
void COM_OPERATOR::Int2AsciiExt(unsigned int input, char * output){  

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

//NOW THIS IS THE MAIN FUNCTION
int main (int argc, char** argv)
{	
	//init	
	ros::init(argc,argv, "COMmunications OPerator Node");
	//ros Info
	ROS_INFO ("Init Complete: COMOP - HADES");
	//create the node. We hang in the main loop.
	COM_OPERATOR COMOP_Node;
	//we technically never get to this point.
	ros::spin();
}
