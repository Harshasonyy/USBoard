// ROS includes
#include <ros/ros.h>
#include <iostream>
#include <SerUSBoard.h>

// ROS message includes

#include <std_msgs/Bool.h>
#include <neo_msgs/USBoard.h>

// ROS service includes
//--

// external includes
//--

//####################
//#### node class ####
class neo_usboard_node
{
	//
	public:
	        // create a handle for this node, initialize node
			ros::NodeHandle n;

	        //basic topics:
			ros::Publisher topicPub_usBoard;

			// Constructor
	       neo_usboard_node()
			  {
		            usboard_available = false;
			    usboard_online = false;
			    usboard_timeout_ = 2.0;
			    //usboard_transmission_mode = SerUSBoard::SEND_ON_REQ;
			  }

	       ~neo_usboard_node()
	          {
	            	delete m_SerUSBoard;
	          }


	       void readUSBoard();
	       int readUSBoardData();


	       int init();
	       //int requestBoardParam();
	       //int requestTransMode();
               int requestBoardStatus();
	       int requestActivateChannels();
	       int requestSensorReadings1TO8();
	       int requestSensorReadings9TO16();
	       int requestAnalogreadings();
               
               void readParameter();
	       double getRequestRate();


	private:

	       std::string sComPort;
	       SerUSBoard * m_SerUSBoard;

	       double requestRate;
	       double usboard_timeout_;
	      // int usboard_transmission_mode;

	       ros::Time time_last_message_received_;
	       bool usboard_online; //the usboard is sending messages at regular time
	       bool usboard_available; //the usboard has sent at least one message -> publish topic

	       //log
	       bool log;  //enables or disables the log for neo_usboard
};




