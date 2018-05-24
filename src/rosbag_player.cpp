#include <roll_test/rosbag_player.h>
#include <cstdlib>

namespace helper
{

RosbagPlayerHelper::RosbagPlayerHelper()
{
  //insert .bag files
  bagfiles.push_back("2018-02-14-14-01-08.bag");
  bagfiles.push_back("2018-02-14-14-07-54.bag");

  //initialize rosbag player
  rosbag::PlayerOptions options;

  //options.loop = true;

  options.topics.push_back("/elevation_mapping/elevation_map");
  options.topics.push_back("/joint_states");
  options.topics.push_back("/tf");
  options.topics.push_back("/tf_static");
  options.topics.push_back("/traversability_estimation/traversability_map");
  options.topics.push_back("/zed/point_cloud/cloud_registered");
  options.topics.push_back("/zed/right/image_raw_color/compressed");

  char *username;

  std::string cppUsername = "mzidianakis";

  options.bags.push_back("/home/"+cppUsername+RELATIVEBAGPATH+bagfiles[0]);

  player = new rosbag::Player(options);
}

RosbagPlayerHelper::~RosbagPlayerHelper()
{
	delete player;
}

}//end namespace