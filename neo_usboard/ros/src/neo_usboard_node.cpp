#include <../include/neo_usboard_node.h>

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "neo_usboard_node");
	neo_usboard_node node;
	if(node.init() != 0) return 1;
	double requestRate = node.getRequestRate();
	ros::Rate r(requestRate); //Cycle-Rate: Frequency of publishing States

	node.requestBoardStatus();
	node.requestActivateChannels();
        
        while(node.n.ok())
        {
          node.requestSensorReadings1TO8();
          node.requestSensorReadings9TO16();
          node.requestAnalogreadings();
          node.readUSBoard();
	  ros::spinOnce();
	  r.sleep();
        }

	return 0;

/*   //for continues mode use only
	node.requestBoardParam();
	int TransMode= node.requestTransMode();

	if(TransMode != SerUSBoard::SEND_ON_REQ)
	{
		node.requestBoardStatus();
		node.requestActivateChannels();
		while(node.n.ok())
		{
			    node.readUSBoardData();
				ros::spinOnce();
				r.sleep();

		}
		return 0;

	}

*/

}
