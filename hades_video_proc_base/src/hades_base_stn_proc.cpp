#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <hades_base_stn/data_packet.h>
#include <hades_base_stn/control_packet.h>

//Define maximum and minimum pixel sizes.
#define PIXEL_MAX 76800
#define FULL_SIZE 230400
#define FILTER_ON

//=====GLOBAL VARIABLES=====

//=====Filter variables
//afterimage frame filter.
sensor_msgs::Image filter_image[2];
sensor_msgs::Image final_out;

//=====subscribers and publishers
//the subscriber to the stream from Base Station Com Op Node
ros::Subscriber video_data_sub;
//the publisher to the video to the screen.
ros::Publisher video_image_pub;

//=====Packet processing variables.
sensor_msgs::Image processed_video; //the video that has been processed
//global variables for packet processing
char starting_char = 0; //starting character when decoding
char character_in = 0; //the input character.
char last_char = 1; //last character reference.
long int packet_length = 0; //packet length of the current packet worked on
char frame_finished = 0;
char pack_err = 0;

char state = 0; //state
//state = 0 waiting for new 1st frame.
//state = 1 waiting for subsequent sub-frames

long int data_ptr = 0; //the position of the pixel.

//TESTING STRING
char test_packet[100];

//number for storing the inter-display time.
double inter_arrival_time_d = 0.0;

//TODO:write image filter.
void filter_image_proc(){

	for (long j = 0; j < FULL_SIZE; ++j){
		
		final_out.data[j] = (filter_image[0].data[j] + filter_image[1].data[j] + processed_video.data[j])/3;
	
		filter_image[1].data[j] = filter_image[0].data[j];
		filter_image[0].data[j] = processed_video.data[j];

	}

}

//get statistics out.
void process_stats(){

	//set the inter-arrival-time
	inter_arrival_time_d = ros::Time::now().toSec() - inter_arrival_time_d; //set the time.
	
	//print the statistics.
	sprintf(test_packet,"IAT: %f DRP_ERR: %i UNIQUE: %i",
		inter_arrival_time_d,
		pack_err,
		frame_finished);

	inter_arrival_time_d = ros::Time::now().toSec(); //save the current time.

	ROS_INFO_STREAM("PKT_DONE. " << test_packet);
}

//=====PROGRAM START=====

//process the image
void PROCESS_IMAGE(const hades_base_stn::data_packet data_pack){

	//we are waiting for a first frame
	if (state == 0){
		
		//throw away UDP and packet length, and just pick out the type.
		if (data_pack.data[1] != 'K'){
			//Wait unti we get a start frame.
			//ROS_INFO("ERROR: NON-START PKT");
			return;
		}

		//set the start char. Either 1 or 0.
		starting_char = data_pack.data[3];
		
		//select the starting character
		if (starting_char == 0) character_in = 0; //black character = 0
		else character_in = 255; //white character = 1
		
		//get the packet length
		//packet_length = data_pack.data[0]*10;
		
		//set the last count to 1, reset the image data pointer.
		data_ptr = 0;
		last_char = 1;
		
		//loop through all of the packet.
		for (int i = 4; i < 500; ++i){
			
			//loop through the array at 3 times the size and fill in the pixels.
			if (data_pack.data[i] == 255){
	
				//if last-char is nonzero
				if ((i != 4) && (last_char != 0)){
					//deswap the character.
					if (character_in == 0) character_in = 255;
					else character_in = 0;
				}
				
				for (int k = 0; k < 762; ++k){
					
					//now put the data into the array.
					processed_video.data[data_ptr] = character_in;
					
					++data_ptr;

					//if we are finished.
					//send the frame out to the image viewer.
					if (data_ptr == FULL_SIZE){
				
						//do cleanup.
						state = 0;

						frame_finished = 1;
						
						return;
					}
				}

				last_char = 255;
			}
			else if (data_pack.data[i] != 0){
			//non-zero number - swap after x px.

				//as long as we are not at the first character.
				//check first that we are not starting.
				if ((i != 4) && (last_char != 0)){
					//deswap the character.
					if (character_in == 0) character_in = 255;
					else character_in = 0;
				}
				 
				//Loop over the count and fill iin the pixels.
				for (int k = 0; k < data_pack.data[i]*3; ++k){
					//now put the data into the array.
					processed_video.data[data_ptr] = character_in;
					//increment the data pointer.
					++data_ptr;
				}

				last_char = data_pack.data[i]; //hold the value of the last input.
				
			}
			else{ //zero. Fill in the old packet.
			//zero: do 255 and then keep the same.


				if ((i != 4) && (last_char != 0)){
					//deswap the character.
					//to undo the last swap.
					if (character_in == 0) character_in = 255;
					else character_in = 0;
				}

				for (int k = 0; k < 762; ++k){
					
					//now put the data into the array.
					processed_video.data[data_ptr] = character_in;
					
					//increment the data pointer
					++data_ptr;
				}

				last_char = 0;
			}
			
			//we return from the function until we get another packet with the full data.
		}

		//once we are done with the first packet, we change state.
		state = 1;
			
	}
	//we are waiting for subsequent frames
	else{
	
		//throw away UDP and packet length, and just pick out the type.
		if (data_pack.data[1] != 'V' && data_pack.data[1] != 'X'){
			
			//We return if we don't get an appropriate video packet.
			//State Machine starts again.
			state = 0;

			frame_finished = 1;

			return;
		}
		//if we did not return, we have recieved a non-start frame.d
		
		//loop through all of the data.
		for (int i = 3; i < 500; ++i){
			
			//loop through the array at 3 times the size and fill in the pixels.
			if (data_pack.data[i] == 255){
				
				//if last-char is nonzero
				if (last_char != 0){
					//deswap the character.
					//to undo the last swap.
					if (character_in == 0) character_in = 255;
					else character_in = 0;
				}

				for (int k = 0; k < 762; ++k){
					
					//now put the data into the array.
					processed_video.data[data_ptr] = character_in;
					
					++data_ptr; //increment the data pointer

					//if we are finished.
					//send the frame out to the image viewer.
					if (data_ptr == FULL_SIZE){

						//do cleanup.
						state = 0;

						frame_finished = 1;
						
						return;
					}
				}
				
				last_char = 255;
			}
			else if (data_pack.data[i] != 0){
				
				//if last-char is nonzero
				if (last_char != 0){
					//deswap the character.
					//to undo the last swap.
					if (character_in == 0) character_in = 255;
					else character_in = 0;
				}

				for (int k = 0; k < data_pack.data[i]*3; ++k){
					//now put the data into the array.
					processed_video.data[data_ptr] = character_in;
					
					//increment the data pointer.
					++data_ptr;

					//if we are finished.
					//send the frame out to the image viewer.
					if (data_ptr == FULL_SIZE){

						//do cleanup.
						state = 0;

						frame_finished = 1;

						return;
					}
				}

				last_char = data_pack.data[i]; //hold the value of the last input.
			}
			else{

				if (last_char != 0){
					if (character_in == 0) character_in = 255;
					else character_in = 0;
				}

				for (int k = 0; k < 762; ++k){

					//now put the data into the array.
					processed_video.data[data_ptr] = character_in;
					
					++data_ptr;

					//if we are finished.
					//send the frame out to the image viewer.
					if (data_ptr == FULL_SIZE){

						//do cleanup.
						state = 0;
						
						frame_finished = 1;
						
						return;
					}
				}
				last_char = 0;
			}
		}
	}
}

//main function.
int main(int argc, char ** argv){
	
	ros::init(argc,argv,"HADES BASE STATION NODE VID PROC");
	ros::NodeHandle nh; //node handle
	
	ROS_INFO("INIT DONE.");
	
	//Initialize the processed video with metadata.
	//the frame size is 320x240
	processed_video.height = 240;
	processed_video.width = 320;
	processed_video.encoding = "bgr8"; //bgr 8 bit per colour 24 bit per px
	processed_video.is_bigendian = 0; //little endian
	processed_video.step = 960; //960 pixels per row.
	
	//fill in metadata in the filter frames.
	for (char i = 0; i < 2; ++i){
		filter_image[i].height = 240;
		filter_image[i].width = 320;
		filter_image[i].encoding = "bgr8"; //bgr 8 bit per colour 24 bit per px
		filter_image[i].is_bigendian = 0; //little endian
		filter_image[i].step = 960; //960 pixels per row.
		filter_image[i].data.resize(FULL_SIZE);
	}
	
	//fill in final frames
	final_out.height = 240;
	final_out.width = 320;
	final_out.encoding = "bgr8"; //bgr 8 bit per colour 24 bit per px
	final_out.is_bigendian = 0; //little endian
	final_out.step = 960; //960 pixels per row.
	final_out.data.resize(FULL_SIZE);

	//set the current time.
	inter_arrival_time_d = ros::Time::now().toSec(); //set the time.
	
	//subscribe to the video
	//The input is the video stream
	video_data_sub = nh.subscribe<hades_base_stn::data_packet>("compressed_vid",2048, &PROCESS_IMAGE);
	
	//publish the video
	//the output is the image stream at 15fps
	video_image_pub = nh.advertise<sensor_msgs::Image>("HADES_video_stream",2048);
	
	//allocate the memory for the video image packet.
	processed_video.data.resize(FULL_SIZE);

	//image rate control
	ros::Rate r(15);
	while (ros::ok()){
		
		//moving average filter over 3 frames
		filter_image_proc();
		
		//publish the images at an appropriate time.
		video_image_pub.publish(final_out);

		process_stats();

		frame_finished = 0;
		pack_err = 0;
		
		//sleep for an appropriate amount of time.
		ros::spinOnce();
		r.sleep();
	}

	//Only callbacks are used.
	ros::spin();

}
