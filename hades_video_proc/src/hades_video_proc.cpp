#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <fstream>
#include <stdio.h>

#include <hades_com_sys/data_packet.h>
#include <hades_com_sys/control_packet.h>

//======PROCESSING SETTINGS=====

//grayscale conversion
#define B_coeff 29
#define G_coeff 150
#define R_coeff 77

#define frame_data_size 76800
#define frame_width 320
#define frame_height 240

//Sub/Pub buffer size
#define buff_size 4096

#define centroid_def 127

#define vid_size_max 500

#define ONE_BIT_COMPRESSION
//#define FOUR_BIT_COMPRESSION

//=====END PROCESSING SETTINGS=====

//the output video packet to HADES com
hades_com_sys::data_packet vid_data;

//the video publisher
ros::Publisher video_pub;

char int_str[4] = {33,33,33,0}; //string to hold integers for Int2AsciiExt
//utility function to turn integer to ASCII
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

//known issues
//- Do not send N_L or char dec 10, as this will crash the Wizfi.
//- For smaller packets, avoid using dec 10, for video or audio, we must use byte stuffing.

char test_packet[100] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //test packet

//For Threshold low bitrate outputs.
unsigned char centroid = centroid_def;

unsigned char comp_data[76800]; //the data to do processing on.
unsigned char test_data[7000]; //test data.
long int test_data_pos = 0; //test data position
long int data_ptr = 0; //the pointer to the data.

long int y_dash; //the grayscale hue during processing

//number of ones and zeros
long int ones = 0;
long int zeros = 0;

//current run length
unsigned char current_run = 0;

//run length
unsigned int run_length = 0;

//count variables
char count_subpacket = 0; //number of packet that has been cut out.
int arr_modifier = 0; //start position. This variable is 500*count_subpacket.

//For Threshold 3-bit colour outputs
unsigned char filtered_data[76800];

//We prepare to send the data out.
void imageCallBack(const sensor_msgs::Image msg){
	
	//reset the data pointer
	data_ptr = 0;
	
	//First send the packet to grayscale
	for (; data_ptr < frame_data_size; ++data_ptr){
		
		//do array access
		y_dash = B_coeff*msg.data[3*data_ptr] + G_coeff*msg.data[(3*data_ptr)+1] + R_coeff*msg.data[(3*data_ptr)+2];

		//fill the array.
		comp_data[data_ptr] = (unsigned char)((y_dash+128)>>8);

	}
	//comp data is 76800 array filled with grayscale data.

	//=====end grayscale=====
	
	#ifdef ONE_BIT_COMPRESSION
	//=====START COMPRESSION PROCESS=====

	//set the run length default to unity.
	//Zero is reserved for a 255 byte block.
	run_length = 1;

	//test_data[0] is the start colour reference
	if (comp_data[0] > centroid){
		current_run = 255; //currently white px
		test_data[0] = 255; //set to white: 255 or 1
	}
	else{
		current_run = 0; //currently black px
		test_data[0] = 0; //set to black: Zero or 0
	}

	//1st position is always the start bit colour.
	test_data_pos = 1; //set data pointer position to 1.

	//reset the data pointer
	//zeroth position already taken into account.
	data_ptr = 1;
	
	//do the transform using entropy run-length encoding
	for (; data_ptr < frame_data_size; ++data_ptr){
		
		if (current_run == 0){
		//our run is zero: currently black

			//encode 1
			if (comp_data[data_ptr] >= centroid){
				++ones;

				//stop the current run and swap.
				test_data[test_data_pos] = (unsigned char)run_length;
				++test_data_pos;

				//reset run length
				run_length = 1;
	
				//now change to encode 255.
				current_run = 255;
			}
			//encode 0
			else{
				++zeros;
				++run_length; //continue the current run
			}

		}
		else{
		//our run is one: currently white

			//encode 1
			if (comp_data[data_ptr] >= centroid){
				++ones;
				++run_length; //continue the current run
			}
			//encode 0
			else{
				++zeros;
				
				//stop the current run and swap.
				test_data[test_data_pos] = (unsigned char)run_length;
				++test_data_pos;
				
				//reset run length
				run_length = 1;

				//now change to encode 0.
				current_run = 0;
			}
		}
		
		//overflow check.
		//check we are not at the end so we do not over-access array.
		if ((run_length == 255) && (data_ptr != 76799)){
			//if we are currently checking for zero (black)
			if (current_run == 0){

				if (comp_data[data_ptr+1] >= centroid){ //if white

					//put a zero into the item.
					test_data[test_data_pos] = 255; //no delta
					++test_data_pos; //still raise the data pos

					run_length = 1; //reset the run length
					current_run = 255;
				}
				else{ //if black
					//put a zero into the item.
					test_data[test_data_pos] = 0; //delta
					++test_data_pos; //still raise the data pos

					run_length = 1;
					current_run = 0;
				}
			}
			//if we are currently checking for one (white)
			else{
				if (comp_data[data_ptr+1] >= centroid){//if white

					//put a zero into the item.
					test_data[test_data_pos] = 0; //no delta
					++test_data_pos; //still raise the data pos

					run_length = 1; //reset the run length
					current_run = 255;
				}
				else{//if black
					//put a zero into the item.
					test_data[test_data_pos] = 255; //delta
					++test_data_pos; //still raise the data pos

					run_length = 1;
					current_run = 0;
				}
			}
		}
	}
	#endif

	#ifdef FOUR_BIT_COMPRESSION
	//THIS IS THE CODE FOR 4 bit MONOCHROME COMPRESSION

	unsigned int buffer = 0;
	char count = 1;

	//pre-filter to smooth out thresholds
	for (long int k = 0; k < 76800; ++k){
		
		//set the temp buffer
		buffer = comp_data[k];
		//set the count.
		count = 1;

		if (k % 320 != 0) //not on left edge
		{
			buffer+= comp_data[k-1];
			++count;
		}
		if (k % 319 != 0) //not on right edge
		{
			buffer+= comp_data[k+1];
			++count;
		}
		if (k >= 320) //not on top edge
		{
			buffer+= comp_data[k-320];
			++count;
		}
		if (k <= 76479) //not on bottom edge
		{
			buffer+= comp_data[k+320];
			++count;
		}
		
		filtered_data[k] = buffer/count;//filter out the pixel

	}
	//filter is now applied to the data.
	//end pre-filter stage

	//build an accumulator.
	unsigned char accumulator = 0;

	//encode to 4 bit colour
	for (long int k = 0; k < 76800; ++k){

		unsigned char data = filtered_data[k];
		
		if (data < 16){

		}
		else if (data => 16 && data < 32){

		}
		else if (data => 32 && data < 48){

		}
		else if (data => 48 && data < 64){

		}
		else if (data => 64 && data < 80){

		}
		else if (data => 80 && data < 96){

		}
		else if (data => 96 && data < 112){

		}
		else if (data => 112 && data < 128){

		}
		else if (data => 128 && data < 144){

		}
		else if (data => 144 && data < 160){

		}
		else if (data => 160 && data < 176){

		}
		else if (data => 176 && data < 192){

		}
		else if (data => 192 && data < 208){

		}
		else if (data => 208 && data < 224){

		}
		else/*if (data => 224 && data < 256)*/{

		}
	}
	
	//end encode

	#endif
	
	/*sprintf(test_packet," %li 1s: %li 0s: %li Total_Proc: %li",
		test_data_pos,
		ones,
		zeros,
		ones + zeros);

	ROS_INFO_STREAM("FRAME: " << test_packet);*/
	//average data size = 2000 bytes

	//=====START PUTTING DATA INTO TRANSMISSION ARRAY=====

	while (test_data_pos >= 497){
		//add 500 byte offset.
		arr_modifier = count_subpacket*497;
		//set the packet size.
		vid_data.pack_size = 50; //50*10;

		//fill out the length field of the packet.
		vid_data.data[0] = '0';
		vid_data.data[1] = '5';
		vid_data.data[2] = '0';
		vid_data.data[3] = '0';

		vid_data.data[4] = 50; //500 bytes

		if (count_subpacket == 0){
			//the first packet
			vid_data.data[5] = 'K'; //for video
		}
		else{
			//not the first packet
			vid_data.data[5] = 'V'; //for video
		}
		
		//do not retransmit: UDP
		vid_data.data[6] = 0; //UDP
		
		//data starts at vid_data position 7.	
		
		//Set 500 bytes to send.
		for (data_ptr = 0; data_ptr < 497; ++data_ptr){
			//send data at appropriate place.
			vid_data.data[data_ptr + 7] = test_data[arr_modifier + data_ptr];
		}

		//publish the packet
		video_pub.publish(vid_data);

		test_data_pos -= 497; //take away 497 bytes.

		++count_subpacket; //increment the packet position at 497.
		
	        /*sprintf(test_packet," %i, %i, %i, %i, %i, %i, %i, %i, %i, %i",
		vid_data.data[4],
		vid_data.data[5],
		vid_data.data[6],
		vid_data.data[7],
		vid_data.data[8],
		vid_data.data[9],
		vid_data.data[10],
		vid_data.data[11],
		vid_data.data[12],
		vid_data.data[13]);

		ROS_INFO_STREAM("FULLPACK: " << test_packet);*/
	}

	//assign the final field.
	vid_data.pack_size = 50;
	
	//if the packet size is non-null.
	if (vid_data.pack_size != 0){
		
		//add 500 byte offset.
		arr_modifier = count_subpacket*497;

		//fill the header.
		vid_data.data[0] = '0';
		vid_data.data[1] = '5';
		vid_data.data[2] = '0';
		vid_data.data[3] = '0';

		vid_data.data[4] = 50;

		if (count_subpacket != 0){
			vid_data.data[5] = 'X'; //video type
		}
		else {
			vid_data.data[5] = 'K'; //First packet
		}
		vid_data.data[6] = 0; //UDP

		//send the rest of the frame
		for (data_ptr = 0; data_ptr < test_data_pos; ++data_ptr){
			//fill the array.
			vid_data.data[data_ptr + 7] = test_data[arr_modifier + data_ptr];
		}
		
		//pad zeroes to the end of the packet.
		for (; data_ptr < 497; ++data_ptr){
			//set the data bytes to zero.
			vid_data.data[data_ptr + 7] = 0;
		}

		//send the last packet (last bit of the frame.)
		video_pub.publish(vid_data);
		
	}

	//======END STATE 3: The whole frame is sent.

	//The whole frame is done. We are finished.

	//cleanup - 1bit compression
	current_run = 0; //reset the current run
	test_data_pos = 0; //reset the test data position.
	run_length = 0; //the run length
	ones = 0; //reset the ones
	zeros = 0; //reset the zeroes
	count_subpacket = 0; //reset the count subpacket count.
	arr_modifier = 0; //array modifier; this variable holds the count of sub-packet

	//=====END PUTTING DATA INTO TRANSMISSION ARRAY=====
	
}

//call back to modify video system parameters.
void controlCallBack(const hades_com_sys::control_packet control){

	//when a centroid packet is recieved, we simply set the centre threshold to that particular value.

	//TODO: MODIFY THE THRESHOLD and use this packet to do it.
	centroid = control.data[9];
}

//NOW THIS IS THE MAIN FUNCTION
int main (int argc, char** argv)
{

	//setup global variables and init.

	//init	
	ros::init(argc,argv, "VIDEO PROCESSING NODE");
	ROS_INFO("Initialized HADES video process");

	//get a node handle
	ros::NodeHandle nh;

	//null the array
	for (; data_ptr < frame_data_size; ++data_ptr){
		comp_data[data_ptr] = 0;
	}
	
	//set the global pointer to zero again.
	data_ptr = 0;

	//null the data array
	for (int i = 0; i < 1024; ++i){
		vid_data.data[i] = 0;
	}

	//fill the test array
	for (int i = 0; i < 4000; ++i){
		test_data[i] = 0;
	}

	//the output video packet advertiser to HADES COM: declared externally due to reference in imageCallBack
	video_pub = nh.advertise<hades_com_sys::data_packet>("video_processed",buff_size);

	//grab the raw video feed from the realsense camera
	ros::Subscriber video_sub = nh.subscribe<sensor_msgs::Image>("realsense/image_raw",buff_size,&imageCallBack);
	
	//subscribe to the controller output and get the control data out.
	ros::Subscriber control_sub = nh.subscribe<hades_com_sys::control_packet>("controller_output",40,&controlCallBack);
	
	//do nothing: let the callbacks do all the work.
	ros::spin();
}
