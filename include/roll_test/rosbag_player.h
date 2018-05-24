#ifndef ROSBAG_PLAYER_HELPER_H
#define ROSBAG_PLAYER_HELPER_H

#define RELATIVEBAGPATH "/Ros_WS/bagfiles/"

#include <ros/console.h>

#include <rosbag/player.h>
#include <string>
#include <boost/thread/thread.hpp>

namespace rosbag
{
class PlayerOptions;
class Player;
}

namespace helper
{

class RosbagPlayerHelper
{
public:
	RosbagPlayerHelper();
	~RosbagPlayerHelper();

	rosbag::Player* player;

private:

	std::vector<std::string> bagfiles;
};

}//end namespace

#endif