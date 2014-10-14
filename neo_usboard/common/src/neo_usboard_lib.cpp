#include <ros/ros.h>
#include <../include/neo_usboard_node.h>



int neo_usboard_node::init()
{


	if (n.hasParam("ComPort"))
	{
		n.getParam("ComPort", sComPort);
		ROS_INFO("Loaded ComPort parameter from parameter server: %s",sComPort.c_str());
	}

	n.param("message_timeout", usboard_timeout_, 0.5);
	n.param("requestRate", requestRate, 25.0);

	m_SerUSBoard = new SerUSBoard();
        readParameter();

	m_SerUSBoard->init();

	ROS_INFO("Opened USboard at ComPort = %s", sComPort.c_str());

	topicPub_usBoard = n.advertise<neo_msgs::USBoard>("/srb_us_measurements",1);

	//log
	n.getParam("log", log);
	if(log == true)
	{
		ROS_INFO("Log enabled");
		m_SerUSBoard->enable_logging();
	}
	else
	{
		ROS_INFO("Log disabled");
		m_SerUSBoard->disable_logging();
	}

   return 0;
}


//--------------------------------------------------------------------------------

void neo_usboard_node::readParameter()
{
	std::string sNumComPort;
	n.getParam("ComPort", sNumComPort);
	m_SerUSBoard->SetPortConfig(sNumComPort);

}
//--------------------------------------------------------------------------------

double neo_usboard_node::getRequestRate()
{
	return requestRate;
}

//--------------------------------------------------------------------------------

int neo_usboard_node::requestBoardStatus() {

	int ret;
	// Request Status of USBoard
	ret = m_SerUSBoard->sendCmdConnect();
	ros::Duration(0.5).sleep();  // send command interval time 0.5 sec

	if(ret != SerUSBoard::NO_ERROR) {
		ROS_ERROR("Error in sending message to USboard over SerialIO, lost bytes during writing");
	}

	ret = m_SerUSBoard->eval_RXBuffer();
	if(ret==SerUSBoard::NOT_INITIALIZED) {
		ROS_ERROR("Failed to read USBoard data over Serial, the device is not initialized");
		usboard_online = false;
	} else if(ret==SerUSBoard::NO_MESSAGES) {
		ROS_ERROR("For a long time, no messages from USBoard have been received, check com port!");
		if(time_last_message_received_.toSec() - ros::Time::now().toSec() > usboard_timeout_) {usboard_online = false;}
	} else if(ret==SerUSBoard::TOO_LESS_BYTES_IN_QUEUE) {
		ROS_ERROR("USBoard: Too less bytes in queue");
	} else if(ret==SerUSBoard::CHECKSUM_ERROR) {
		ROS_ERROR("A checksum error occurred while reading from usboard data");
	} else if(ret==SerUSBoard::NO_ERROR) {

		usboard_online = true;
		usboard_available = true;
		time_last_message_received_ = ros::Time::now();

	}
	return 0;
}

//--------------------------------------------------------------------------------

int neo_usboard_node::requestActivateChannels()
{
	int ret;
	//Activate the USBoard Sensors
	ret = m_SerUSBoard->sendCmdSetChannelActive();

	if(ret != SerUSBoard::NO_ERROR) {
			ROS_ERROR("Error in sending message to USboard over SerialIO, lost bytes during writing");
		}
	return 0;
}

//--------------------------------------------------------------------------------

int neo_usboard_node::requestSensorReadings1TO8()
{
	int ret;
	//Request Sensor 1 to 8 readings
	ret = m_SerUSBoard->sendCmdGetData1To8();
	ros::Duration(0.5).sleep(); // send command interval time 0.5 sec

	if(ret != SerUSBoard::NO_ERROR) {
		ROS_ERROR("Error in sending message to USboard over SerialIO, lost bytes during writing");
	}

	ret = m_SerUSBoard->eval_RXBuffer();
	if(ret==SerUSBoard::NOT_INITIALIZED) {
		ROS_ERROR("Failed to read USBoard data over Serial, the device is not initialized");
		usboard_online = false;
	} else if(ret==SerUSBoard::NO_MESSAGES) {
		ROS_ERROR("For a long time, no messages from USBoard have been received, check com port!");
		if(time_last_message_received_.toSec() - ros::Time::now().toSec() > usboard_timeout_) {usboard_online = false;}
	} else if(ret==SerUSBoard::TOO_LESS_BYTES_IN_QUEUE) {
		ROS_ERROR("USBoard: Too less bytes in queue");
	} else if(ret==SerUSBoard::CHECKSUM_ERROR) {
		ROS_ERROR("A checksum error occurred while reading from usboard data");
	} else if(ret==SerUSBoard::NO_ERROR) {
		usboard_online = true;
		usboard_available = true;
		time_last_message_received_ = ros::Time::now();

	}

	return 0;
}

//--------------------------------------------------------------------------------

int neo_usboard_node::requestSensorReadings9TO16()
{
	int ret;
	//Request Sensor 9 to 16 readings
	ret = m_SerUSBoard->sendCmdGetData9To16();
	ros::Duration(0.5).sleep(); // send command interval time 0.5 sec

	if(ret != SerUSBoard::NO_ERROR) {
		ROS_ERROR("Error in sending message to USboard over SerialIO, lost bytes during writing");
	}

	ret = m_SerUSBoard->eval_RXBuffer();
	if(ret==SerUSBoard::NOT_INITIALIZED) {
		ROS_ERROR("Failed to read USBoard data over Serial, the device is not initialized");
		usboard_online = false;
	} else if(ret==SerUSBoard::NO_MESSAGES) {
		ROS_ERROR("For a long time, no messages from USBoard have been received, check com port!");
		if(time_last_message_received_.toSec() - ros::Time::now().toSec() > usboard_timeout_) {usboard_online = false;}
	} else if(ret==SerUSBoard::TOO_LESS_BYTES_IN_QUEUE) {
		//ROS_ERROR("USBoard: Too less bytes in queue");
	} else if(ret==SerUSBoard::CHECKSUM_ERROR) {
		ROS_ERROR("A checksum error occurred while reading from usboard data");
	} else if(ret==SerUSBoard::NO_ERROR) {
		usboard_online = true;
		usboard_available = true;
		time_last_message_received_ = ros::Time::now();

	}
	return 0;
}

//--------------------------------------------------------------------------------

int neo_usboard_node::requestAnalogreadings()
{
	int ret;
	//Request Analog readings
	ret = m_SerUSBoard->sendCmdGetAnalogIn();
	ros::Duration(0.5).sleep(); // send command interval time 0.5 sec

	if(ret != SerUSBoard::NO_ERROR) {
		ROS_ERROR("Error in sending message to USboard over SerialIO, lost bytes during writing");
	}

	ret = m_SerUSBoard->eval_RXBuffer();
	if(ret==SerUSBoard::NOT_INITIALIZED) {
		ROS_ERROR("Failed to read USBoard data over Serial, the device is not initialized");
		usboard_online = false;
	} else if(ret==SerUSBoard::NO_MESSAGES) {
		ROS_ERROR("For a long time, no messages from USBoard have been received, check com port!");
		if(time_last_message_received_.toSec() - ros::Time::now().toSec() > usboard_timeout_) {usboard_online = false;}
	} else if(ret==SerUSBoard::TOO_LESS_BYTES_IN_QUEUE) {
		//ROS_ERROR("USBoard: Too less bytes in queue");
	} else if(ret==SerUSBoard::CHECKSUM_ERROR) {
		ROS_ERROR("A checksum error occurred while reading from usboard data");
	} else if(ret==SerUSBoard::NO_ERROR) {
		usboard_online = true;
		usboard_available = true;
		time_last_message_received_ = ros::Time::now();

	}
	return 0;
}

//--------------------------------------------------------------------------------

void neo_usboard_node::readUSBoard()
{
		if(!usboard_available == 1) return;

		int usSensors[4];
                int usAnalog[4];
		neo_msgs::USBoard usBoard;

		m_SerUSBoard->getSensorData1To4(usSensors);
		for(int i=0; i<4; i++) usBoard.sensor[i] = usSensors[i];
		m_SerUSBoard->getSensorData5To8(usSensors);
		for(int i=0; i<4; i++) usBoard.sensor[i+4] = usSensors[i];
		m_SerUSBoard->getSensorData9To12(usSensors);
		for(int i=0; i<4; i++) usBoard.sensor[i+8] = usSensors[i];
		m_SerUSBoard->getSensorData13To16(usSensors);
		for(int i=0; i<4; i++) usBoard.sensor[i+12] = usSensors[i];
		m_SerUSBoard->getAnalogInCh1To4Data(usAnalog);
		for(int i=0; i<4; i++) usBoard.analog[i] = usAnalog[i];

		topicPub_usBoard.publish(usBoard);
		//ROS_INFO("USBoard Data Published");
}


//--------------------------------------------------------------------------------
/* for continues mode use only
int neo_usboard_node::requestBoardParam() {
	int ret;

	// Request Transmission Mode of USBoard
	ret = m_SerUSBoard->sendCmdReadParaSet();
	//ros::Duration(0.5).sleep();

	if(ret != SerUSBoard::NO_ERROR) {
		ROS_ERROR("Error in sending message to USboard over SerialIO, lost bytes during writing");
	}

	ret = m_SerUSBoard->eval_RXBuffer();

	if(ret==SerUSBoard::NOT_INITIALIZED) {
		ROS_ERROR("Failed to read USBoard data over Serial, the device is not initialized");
		usboard_online = false;
	} else if(ret==SerUSBoard::NO_MESSAGES) {
		ROS_ERROR("For a long time, no messages from USBoard have been received, check com port!");
		if(time_last_message_received_.toSec() - ros::Time::now().toSec() > usboard_timeout_) {usboard_online = false;}
	} else if(ret==SerUSBoard::TOO_LESS_BYTES_IN_QUEUE) {
		ROS_ERROR("USBoard: Too less bytes in queue");
	} else if(ret==SerUSBoard::CHECKSUM_ERROR) {
		ROS_ERROR("A checksum error occurred while reading from usboard data");
	} else if(ret==SerUSBoard::NO_ERROR) {

		usboard_online = true;
		usboard_available = true;
		time_last_message_received_ = ros::Time::now();
        usboard_transmission_mode = m_SerUSBoard->getTransModeData();

	}
	return 0;
}

//--------------------------------------------------------------------------------

int neo_usboard_node::requestTransMode()
{

	if(!usboard_available == 1) return -1;
	 return usboard_transmission_mode;
}
*/

//--------------------------------------------------------------------------------
/* for continues mode use only
int neo_usboard_node::readUSBoardData()
{

	if(usboard_transmission_mode == SerUSBoard::SEND_ON_REQ)
	{
		readUSBoard();
	}
	else if(usboard_transmission_mode == ((SerUSBoard::SEND_CONT_RS232) | (SerUSBoard::SEND_CONT_CAN) | (SerUSBoard::SEND_CONT_CAN_RS232)))
	{

		int ret = m_SerUSBoard->eval_RXBuffer();

			if(ret==SerUSBoard::NOT_INITIALIZED) {
				ROS_ERROR("Failed to read USBoard data over Serial, the device is not initialized");
				usboard_online = false;
			} else if(ret==SerUSBoard::NO_MESSAGES) {
				ROS_ERROR("For a long time, no messages from USBoard have been received, check com port!");
				if(time_last_message_received_.toSec() - ros::Time::now().toSec() > usboard_timeout_) {usboard_online = false;}
			} else if(ret==SerUSBoard::TOO_LESS_BYTES_IN_QUEUE) {
				//ROS_ERROR("USBoard: Too less bytes in queue");
			} else if(ret==SerUSBoard::CHECKSUM_ERROR) {
				ROS_ERROR("A checksum error occurred while reading from usboard data");
			} else if(ret==SerUSBoard::NO_ERROR) {
				usboard_online = true;
				usboard_available = true;

				readUSBoard();
				time_last_message_received_ = ros::Time::now();
			}
	}
	return 0;
} */

//--------------------------------------------------------------------------------



