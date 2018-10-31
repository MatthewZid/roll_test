#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pointcloud_msgs/PointCloud2_Segments.h>
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

	rosbag::View full_bag_view;
	full_bag_view.addQuery(input_bag);

	ros::Time initial_time = full_bag_view.getBeginTime();
	ros::Time finish_time = ros::TIME_MAX;

	rosbag::View bag_view;
	bag_view.addQuery(input_bag, initial_time, finish_time);

	//sort clusters according to stamp
	//std::sort(cluster.begin(), cluster.end(), sortWay);

	//create timestamp map
	std::map< double, std::vector<size_t> > stamp_map;

	for(rosbag::MessageInstance m : bag_view)
	{
		/*if(!m.isType<sensor_msgs::PointCloud2>() or m.getTopic() != "/pointcloud2_segments_viz/pc2_viz")
			continue;*/
		if(!m.isType<pointcloud_msgs::PointCloud2_Segments>() or m.getTopic() != "/pointcloud2_cluster_tracking/clusters")
			continue;

		std::vector<size_t> posvec;

		for(size_t i=0; i < cluster.size(); i++)
			if(m.getTime().toSec() == cluster[i].segment_stamp.toSec())
				posvec.push_back(i);

		if(!posvec.empty())
			stamp_map.insert(std::make_pair(m.getTime().toSec(), posvec));
	}

	if(stamp_map.empty())
		ROS_WARN("KJKLJLJLLK\n");

	//class count
	std::set<std::string> clusters_set;

	for(int i=0; i < cluster.size(); i++)
		clusters_set.insert(cluster[i].name);

	int class_num = clusters_set.size();

	//class-color map
	std::map<std::string, pcl::PointXYZRGB> color_map;

	for(auto it = clusters_set.begin(); it != clusters_set.end(); it++)
	{
		size_t pos = std::distance(clusters_set.begin(), it);
		int color_id = pos % 3;
		std::string class_name = *it;
		pcl::PointXYZRGB color_val;

		if(color_id == 0){
			color_val.r = 255;
			color_val.g = 255 * pos % 255;
			color_val.b = 255 * pos % 255;
		}
		else if(color_id == 1){
			color_val.r = 255 * pos % 255;
			color_val.g = 255;
			color_val.b = 255 * pos % 255;
		}
		else if(color_id == 2){
			color_val.r = 255 * pos % 255;
			color_val.g = 255 * pos % 255;
			color_val.b = 255;
		}

		color_map.insert(std::make_pair(class_name, color_val));
	}
ROS_WARN("END\n");
	//write messages to output bagfile, consulting csv annotations
	/*for(rosbag::MessageInstance m : bag_view)
	{
		if(!m.isType<pointcloud_msgs::PointCloud2_Segments>() or m.getTopic() != "/pointcloud2_cluster_tracking/clusters"){
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

        for(int i=0; i < cloud.points.size(); i++)
        {
        	bool found = false;

        	for(int j=0; j < it->second.size(); j++)
        	{
        		size_t pos = (it->second)[j];

        		for(int k=0; k < cluster[pos].points.size(); k++)
        			if(cloud.points[i].x == cluster[pos].points[k].x and
        				cloud.points[i].y == cluster[pos].points[k].y and
        				cloud.points[i].z == cluster[pos].points[k].z)
        			{
        				found = true;
        				auto colit = color_map.find(cluster[pos].name);

        				cloud.points[i].r = colit->second.r;
        				cloud.points[i].g = colit->second.g;
        				cloud.points[i].b = colit->second.b;

        				break;
        			}

        		if(found)
        			break;
        	}

        	if(!found)
        	{
        		cloud.points[i].r = 80;
				cloud.points[i].g = 80;
				cloud.points[i].b = 80;
        	}
        }

        boost::shared_ptr<sensor_msgs::PointCloud2> final_pcmsg(boost::make_shared<sensor_msgs::PointCloud2>());

        pcl::PCLPointCloud2 cloud_annot;
        pcl::toPCLPointCloud2(cloud, cloud_annot);
        pcl_conversions::fromPCL(cloud_annot, *final_pcmsg);

        output_bag.write(m.getTopic(), m.getTime(), final_pcmsg, m.getConnectionHeader());
	}*/

	input_bag.close();
	output_bag.close();

	ROS_INFO("Exported annotated bag\n");

	return 0;
}