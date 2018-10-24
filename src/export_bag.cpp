#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <roll_test/point_class.h>
#include <fstream>

#include <pcl_ros/point_cloud.h>

bool sortWay(roll_test::PointClass a, roll_test::PointClass b)
{
	return(a.stamp.toSec() < b.stamp.toSec());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "exportBag");

	std::srand(std::time(nullptr));

	//initialize files
	rosbag::Bag input_bag;
	rosbag::Bag output_bag;

	std::string homepath = std::getenv("HOME");
	std::string input_bagname = argv[1];
	std::string output_bagname = "annotated_" + input_bagname;

	//read csv file
	std::vector<roll_test::PointClass> cluster = roll_test::readcsv();

    //open input bagfile
	try
	{
		std::string bagpath = homepath + "/Ros_WS/bagfiles/" + input_bagname;
		input_bag.open(bagpath);
	}
	catch (rosbag::BagUnindexedException ex) {
        std::cerr << "Bag file " << input_bagname << " is unindexed.  Run rosbag reindex." << std::endl;
        return -1;
    }

    //open output bagfile
    try
	{
		std::string bagpath = homepath + "/Ros_WS/bagfiles/" + output_bagname;
		output_bag.open(bagpath, rosbag::bagmode::Write);
	}
	catch (rosbag::BagUnindexedException ex) {
        std::cerr << "Bag file " << output_bagname << " is unindexed.  Run rosbag reindex." << std::endl;
        return -1;
    }

	if(!ros::ok()){
		input_bag.close();
		output_bag.close();
		return -1;
	}

	rosbag::View bag_view;
	bag_view.addQuery(input_bag);

	//sort clusters according to stamp
	std::sort(cluster.begin(), cluster.end(), sortWay);

	//create timestamp map
	std::map< double, std::vector<size_t> > stamp_map;

	for(rosbag::MessageInstance m : bag_view)
	{
		std::vector<size_t> posvec;

		for(int i=0; i < cluster.size(); i++)
			if(m.getTime().toSec() == cluster[i].stamp.toSec())
				posvec.push_back(i);

		if(!posvec.empty())
			stamp_map.insert(std::make_pair(m.getTime().toSec(), posvec));
	}

	//class count
	std::set<std::string> clusters_set;

	for(int i=0; i < cluster.size(); i++)
		clusters_set.insert(cluster[i].name);

	int class_num = clusters_set.size();

	//class-color map
	std::map<std::string, int> color_map;

	for(auto it = clusters_set.begin(); it != clusters_set.end(); it++)
	{
		int color_val = std::rand() % class_num;
		std::string class_name = *it;

		color_map.insert(std::make_pair(class_name, color_val));
	}

	//write messages to output bagfile, consulting csv annotations
	for(rosbag::MessageInstance m : bag_view)
	{
		if(!m.isType<sensor_msgs::PointCloud2>()){
			output_bag.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
			continue;
		}

		auto it = stamp_map.find(m.getTime().toSec());

        if(it == stamp_map.end()){
        	output_bag.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
        	continue;
        }

		//if pointcloud2, consult csv and then write
		boost::shared_ptr<sensor_msgs::PointCloud2> pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
		pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL(*pc_msg, cloud2);

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromPCLPointCloud2(cloud2, cloud);

        for(int i=0; i < it->second.size(); i++)
        {
        	size_t pos = (it->second)[i];
        	bool found = false;

        	for(int j=0; j < cloud.points.size(); j++)
        	{
        		for(int k=0; k < cluster[pos].points.size(); k++)
        			if(cloud.points[j].x == cluster[pos].points[k].x and
        				cloud.points[j].y == cluster[pos].points[k].y and
        				cloud.points[j].z == cluster[pos].points[k].z)
        			{
        				found = true;
        				
        				//find color for class

        				break;
        			}

        		if(found)
        			break;
        	}

        	if(found)
        		break;
        }
	}

	input_bag.close();
	output_bag.close();

	return 0;
}