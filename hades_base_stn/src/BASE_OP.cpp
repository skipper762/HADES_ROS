#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <fstream>
#include <hades_base_stn/data_packet.h>
#include <hades_base_stn/control_packet.h>
#include <hades_base_stn/sensor_packet.h>
#include <stdio.h>

#define stream_buff_size 2048
#define transmit_rate 15

#define threshold_high 190
#define threshold_low 100

//class that handles the communication between HADES system and
//the node hardware.
class BASE_STN_OPERATOR
{
	//external access
	public:
		BASE_STN_OPERATOR();
		~BASE_STN_OPERATOR();
	
	//internal access
	private:

		void mainLoop(); //main loop

		void joyCallback(const sensor_msgs::Joy joy); // is called everytime a joy message is received 		
		void incomingVideoDataCallBack(const hades_base_stn::data_packet video_data); //data is coming in from WizFi.
		void incomingSensorDataCallBack(const hades_base_stn::sensor_packet sensor_data); //data is coming in from a sensor.
		void incomingControlDataCallBack(const hades_base_stn::control_packet control_data); //data is control data.
		void outgoingPacketTransmit(); //transmit a packet to the WizFi.

		void Int2AsciiExt(unsigned int input, char * output); //utility function to convert int to ascii

		//ROS subscribers and publishers
		ros::NodeHandle nh; // the main process node object
		
		//WIZFI INTERFACE
		ros::Publisher WizFi_controller_stream_pub; //Publish controller signals to WizFi
		ros::Subscriber WizFi_video_stream_sub; //subscribe to video data from WizFi
		ros::Subscriber WizFi_control_sig_stream_sub; //subscribe to the control signals coming from HADES.
		ros::Subscriber WizFi_sensor_stream_sub; //subscribe to the sensor signals coming from HADES.
		
		//REST OF ROS INTERFACE
		ros::Subscriber joy_sub; //subscribe to the joystick.
		
		//THIS IS FOR VIDEO
		ros::Publisher VIDEO_DATA_pub;
		
		//NOT IMPLEMENTED: USER INTERFACES.
		//ros::Subscriber command_data; //Set advanced commands from user interface to base station node.
		//ros::Publisher Sensor_data; //publisher to send sensor data to user interface.
		//ros::Publisher video_data; //publisher to send video data to user interface.
		
		//MESSAGES FOR PUBLISHERS
		hades_base_stn::control_packet ctrl_packet; //Controller commands: every 50ms
		//The video packet going to user.
		sensor_msgs::Image frame;

		//NOT IMPLEMENTED: ADVANCED CONTROL FEATURES
		//hades_base_stn::control_packet esc_control_packet; //low level control of ESCs: Whenever required
		//hades_base_stn::control_packet backup_control_packet; //bypass NUC and send a backup control packet to bkp MCU
		//hades_base_stn::control_packet other_command; //other command from base station to HADES: For scalability

		hades_base_stn::data_packet video_data; //video packet from HADES.

		hades_base_stn::sensor_packet sensor_data; //sensor packet from HADES.

		//Controller Command Global Variables
		char joy_active;

		char command_type; //command type
		char HADES_speed; //HADES speed
		char led_brightness; //LED brightness
		char camera_angle, camera_toggle; //camera angle select
		char prevLB,prevRB; //previous left and right buttons
		char drive,pivot; //motion controls
		char brake; //the brake command.
		
		//TEST VARIABLES
		//char test_str[2] = {0,0};
		//char int_str[4] = {33,33,33,0};
		//char test_char = 'A';
		char test_packet[100];
};

//constructor
BASE_STN_OPERATOR::BASE_STN_OPERATOR()
{
	//=====setup=====
	//subscribe to joy
	joy_sub = nh.subscribe<sensor_msgs::Joy>("joy",30, &BASE_STN_OPERATOR::joyCallback, this);

	//subscriptions and publishers to Wizfi
	//send data over.
	WizFi_controller_stream_pub = nh.advertise<hades_base_stn::control_packet>("downstream",40); //setup the publisher for upstream input.
	//subscribe to video from WIZFI
	WizFi_video_stream_sub = nh.subscribe<hades_base_stn::data_packet>("upstream_video",stream_buff_size, &BASE_STN_OPERATOR::incomingVideoDataCallBack, this);
	//subscribe to sensor
	WizFi_control_sig_stream_sub = nh.subscribe<hades_base_stn::sensor_packet>("upstream_sensor",100, &BASE_STN_OPERATOR::incomingSensorDataCallBack, this); 
	//subscribe to control	
	WizFi_sensor_stream_sub = nh.subscribe<hades_base_stn::control_packet>("upstream_control",20, &BASE_STN_OPERATOR::incomingControlDataCallBack, this);

	//publish the video stream to SCREEN
	VIDEO_DATA_pub = nh.advertise<hades_base_stn::data_packet>("compressed_vid",stream_buff_size);
	
	//init global variables

	//command variables
	joy_active = 0;
	
	command_type = 'C';
	HADES_speed = 5;
	led_brightness = 250;
	camera_angle = 0;
	camera_toggle = 0;

	prevLB = 0;
	prevRB = 0;

	drive = 0;
	brake = 0;
	pivot = 0;
	
	//for debug
	/*for (int i = 0; i < 100; ++i){
		test_packet[i] = 0;
	}*/

	//=====MAIN=====
	mainLoop();
}

//destructor: Nothing really goes in here.
BASE_STN_OPERATOR::~BASE_STN_OPERATOR()
{
	//Nothing at this stage
	ROS_INFO ("Terminating BASE_COM Node");
}

//data is coming out from HADES to base station
void BASE_STN_OPERATOR::incomingVideoDataCallBack(const hades_base_stn::data_packet video_data){

	
	/*sprintf(test_packet,"%i, %i, %i, %i, %i, %i, %i, %i, %i, %i",
		video_data.data[490],
		video_data.data[491],
		video_data.data[492],
		video_data.data[493],
		video_data.data[494],
		video_data.data[495],
		video_data.data[496],
		video_data.data[497],
		video_data.data[498],
		video_data.data[499]);

	ROS_INFO_STREAM("pk: " << test_packet);*/

	//Bounce the packet to the data processing
	VIDEO_DATA_pub.publish(video_data);
}

//data is coming in from a sensor on HADES
void BASE_STN_OPERATOR::incomingSensorDataCallBack(const hades_base_stn::sensor_packet sensor_data){
	//TODO:write sensor data to external node for processing and user interface
	//NOT IMPLEMENTED: SEND CONTENTS OF sensor_data to external node
}

//data is control data.
void BASE_STN_OPERATOR::incomingControlDataCallBack(const hades_base_stn::control_packet control_data){
	//TODO:write control data to external logger for processing and user interface
	//NOT IMPLEMENTED: SEND CONTENTS OF control_data to external node
}

//the controller singals are going to Base station
void BASE_STN_OPERATOR::outgoingPacketTransmit(){
	
	//our packet is already built, so all we do is shoot it out.
	//send controller data over the publisher
	WizFi_controller_stream_pub.publish(ctrl_packet);
}

//utility function
void BASE_STN_OPERATOR::Int2AsciiExt(unsigned int input, char * output){  

  //Set numbers
  unsigned char hundreds = 0, tens = 0, units = 0; //hold val for each area
  tens = input / 10; //set tens
  units = input % 10; //set units

  //Get hundreds
  //basically modulo the tens until we are done.
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

// is called whenever a joy message is received 
void BASE_STN_OPERATOR::joyCallback(const sensor_msgs::Joy joy_msg)
{
	if (joy_active == 0 && joy_msg.buttons[7]==1) joy_active = 1; //activate the controller.
	else if (joy_active == 1 && joy_msg.buttons[6]==1) joy_active = 0; //deactivate the controller.

	//ROS_INFO("CONTROLLER_COMMAND_RECCED");
	//Drive packet format
	//0: length (10 bytes) = const char 1
	//1: ID (Use control) = const char 'C'
	//2: TCP or UDP type = const char 0 (for UDP)
	//3: Drive strength: char 0 to +200 with 100 being neutral. Brake automatically applied when no drive strength.
	//4: Skew strength: char 0 to +200 with 0 being -%50 strength on left side. 100 for forward travel.
	//5: On-Spot turn: 0 if using byte 3/4: otherwise, 0 to +200 for spot turn.
	//6: Camera angle - 0: main; 1: left; 2: right; 3: rear... more can be added if neccessary.
	//7: LED brightness: 50-250 front LED
	//8: reserved
	//9: reserved

	//Deployment packet format
	//0: length (10 bytes) = const char 1
	//1: ID (Use deploy) = const char 'D'
	//2: TCP or UDP type = const char 1 (for TCP) or 0 for Advanced
	//3: LEFT OR RIGHT
	
	//NOT IMPLEMENTED: ADVANCED NODE DEPLOYMENT PROCEDURE: LOW LEVEL CONTROL 
	//4: ADV: ADVANCED Deployment enable: 1 for enable, 0 for disable.
	//5: ADV: CONTROL TYPE
	//6: ADV: deployment strength
	//7-9: RESERVED

	//TODO: CHECK whether this is a node deployment signal or a drive signal.
	//NOT YET IMPLEMENTED: NODE DEPLOYMENT SOFTWARE.
	if (true){ //drive signal
		command_type = 'C';
	}
	else { //deployment signal
		command_type = 'D';
	}
	
	//determine the speed
	//LB goes down a gear, RB goes up a gear
	if(joy_msg.buttons[5]==1&&prevRB==0){ // checks for a low to high transition (RB button being pressed) 
		if(HADES_speed<100){
			HADES_speed = HADES_speed + 5;// increases speed 
		}
	}
	if(joy_msg.buttons[4]&&prevLB==0){ // same as above for LB button which decreases top speed
		if(HADES_speed>5){
			HADES_speed = HADES_speed - 5; //decreases speed
		}
	}

	//set inputs from crosspad
	drive = joy_msg.axes[7];
	pivot = joy_msg.axes[6];

	prevRB = joy_msg.buttons[5];	// stores current RB in variable 
	prevLB = joy_msg.buttons[4];	// stores current LB in variable
	//one click HtoL transition HOLD
	
	//set settings
	ctrl_packet.data[0] = 1; //length 10/10bytes = 1 10 byte packet.
	ctrl_packet.data[1] = command_type; //C for control

	if (command_type == 'C'){ //control signal

		ctrl_packet.data[2] = 0; //0 for no ack
	
		//CROSS PAD:
		//UP: DRIVE FORWARD
		//DOWN: DRIVE BACKWARDS
		//LEFT: SPOT-TURN TO THE LEFT
		//RIGHT: SPOT-TURN TO THE RIGHT
	
		//set On-Spot Turn and drive strength

		if (pivot == -1){
			ctrl_packet.data[5] = 100 - HADES_speed; //left turn
		}
		else if (pivot == 1){
			ctrl_packet.data[5] = 100 + HADES_speed; //right turn
		}
		else{
			ctrl_packet.data[5] = 100; //no spot turn
			
			//set Drive Strength
			if (drive == -1){ //reverse
				ctrl_packet.data[3] = 100 - HADES_speed;
			}
			else if (drive == 1){ //drive
				ctrl_packet.data[3] = 100 + HADES_speed;
			}
			else { //go nowhere
				ctrl_packet.data[3] = 100;
			}
		}

		//set Skew Strength
		ctrl_packet.data[4] = 100 + (int)(50.0*(joy_msg.axes[5])) - (int)(50.0*(joy_msg.axes[2]));

		//Set Camera Angle
		//X toggles the camera feed: button prompt for camera change
		if (joy_msg.buttons[2] == 1 && camera_toggle == 0) {
			++camera_angle; //increment the camera angle
			camera_toggle = 1; //latch
		}
		else if (joy_msg.buttons[2] == 0){
			camera_toggle = 0; //delatch
		}

		if (camera_angle > 3) camera_angle = 0; //reset the camera angle if we have a valid trigger.
		ctrl_packet.data[6] = camera_angle; //set to the target camera

		//Set LED brightness
		//TODO:FIND LED brightness prompt (button or otherwise.) Not really that important
		ctrl_packet.data[7] = 250; //set to max
		
		//reserved field
		//can use for acutated rear joint
		//can use for front camera angle select on a servo
		//can select through IR/point map

		//CURRENT USE. data field 9 is now the threshold setting
		ctrl_packet.data[8] = 100; //reserved field
		ctrl_packet.data[9] = (char)(((joy_msg.axes[4]+1.0)/2.0)*255.0); //field for threshold setting.
		//threshold ceilings
		if (ctrl_packet.data[9] < threshold_low) ctrl_packet.data[9] = threshold_low;
		if (ctrl_packet.data[9] > threshold_high) ctrl_packet.data[9] = threshold_high;
		
		//=====braking function=====		
		
		brake = joy_msg.buttons[0];// boolean operator, if high overrides all commands and bring HADES to a halt.
		if (brake == 1){ //zero out all of the drive signals when the brake is initiated.
			ctrl_packet.data[3] = 100;
			ctrl_packet.data[4] = 100;
			ctrl_packet.data[5] = 100;
		}

	}
	else{ //deploy node signal
		
		//simple deployment
		ctrl_packet.data[2] = 1; //1 for ack
		ctrl_packet.data[3] = 0; //0 for LEFT, 1 for RIGHT
		
		//TODO:advanced deployment (Not currently used.)
		ctrl_packet.data[4] = 0; //advanced deployment
		ctrl_packet.data[5] = 0; //control type
		ctrl_packet.data[6] = 0; //turn strength

		ctrl_packet.data[7] = 100; //reserved field
		ctrl_packet.data[8] = 100; //reserved field
		ctrl_packet.data[9] = 100; //reserved field
	}
	
	//debug lines
	/*sprintf(test_packet,"PK: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i",
		ctrl_packet.data[0],
		ctrl_packet.data[1],
		ctrl_packet.data[2],
		ctrl_packet.data[3],
		ctrl_packet.data[4],
		ctrl_packet.data[5],
		ctrl_packet.data[6],
		ctrl_packet.data[7],
		ctrl_packet.data[8],
		ctrl_packet.data[9]);

	ROS_INFO_STREAM("CONTROL OUT: " << test_packet); */

//we transmit this packet on a timer to the WizFi on a regular basis (not everytime we recieve a command.)
}

//we are stuck in this main loop
void BASE_STN_OPERATOR::mainLoop(){
	
	ros::Rate r(transmit_rate); // 20Hz: send the data packet.
	while (ros::ok()){

		//do Things in While True
		//- send the controller message to the Wizfi.
		//NOT IMPLEMENTED
		//- send all advanced data and low level control features to the user interface
			
		/*sprintf(test_packet,"PK: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i",
		ctrl_packet.data[0],
		ctrl_packet.data[1],
		ctrl_packet.data[2],
		ctrl_packet.data[3],
		ctrl_packet.data[4],
		ctrl_packet.data[5],
		ctrl_packet.data[6],
		ctrl_packet.data[7],
		ctrl_packet.data[8],
		ctrl_packet.data[9]);

		ROS_INFO_STREAM("CONTROL OUT: " << test_packet);*/

		//only transmit the controller message if the controller message is active.
		if (joy_active == 1){
			outgoingPacketTransmit(); //send controller message.
			//ROS_INFO("SENDING_CONTROLLER_COMMAND_PKT");
		}
		
		//do the delay
		ros::spinOnce();
		r.sleep();
	}
}

//NOW THIS IS THE MAIN FUNCTION
int main (int argc, char** argv)
{	
	//init	
	ros::init(argc,argv, "BASE STATION COMmunications OPerator Node");
	//ros Info
	ROS_INFO ("Init Complete: BASE COMOP - HADES");
	//create the node. We hang in the main loop.
	BASE_STN_OPERATOR BASE_COMOP_Node;
	//we technically never get to this point.
	ros::spin();
}
