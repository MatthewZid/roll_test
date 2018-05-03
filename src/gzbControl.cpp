#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/conversions.h"
#include "roll_test/Pos.h"

bool cntrl=true;
ros::Publisher pos_pub;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    
    float min=100.0f;
    float rel_x, rel_y;
    
    for(auto& pt : temp_cloud->points){
        if(pt.z==pt.z and pt.z<min and pt.y==pt.y and pt.y>=-0.1f and pt.y<=0.1f and pt.x==pt.x/*and pt.x>=-0.1f and pt.x<=0.1f*/){
            min=pt.z;
            rel_x=pt.x;
            rel_y=pt.y;
            printf("%f, %f, %f\n",pt.x, pt.y, pt.z);
        }
    }
    
    if(min<=0.8f){
        roll_test::Pos pos_msg;
        cntrl=false;
        printf("Min: %f\n",min);

        //msg frame_id and time stamp
        pos_msg.header.frame_id=msg->header.frame_id;
        pos_msg.header.stamp=ros::Time::now();

        //initial coordinates
        pos_msg.init_pos.x=0.0f;
        pos_msg.init_pos.y=0.0f;
        pos_msg.init_pos.z=0.0f;
        
        //stop coordinates
        pos_msg.stop_pos.x=rel_x;
        pos_msg.stop_pos.y=rel_y;
        pos_msg.stop_pos.z=min;
        
        pos_pub.publish(pos_msg);
    }
    else
        cntrl=true;
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "gzbCntrl");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    ros::Rate loop_rate(0.001);
    
    ros::NodeHandle nh;
    //std::string topic = nh.resolveName("point_cloud");
    uint32_t queue_size = 1;
    
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("camera/depth/points", queue_size, callback);
    
    pos_pub = n.advertise<roll_test::Pos>("robot_pos",1);
    
    while(ros::ok()){
        geometry_msgs::Twist msg;
        
        //forward on x
        if(cntrl)
            msg.linear.x=1.0;
        
        vel_pub.publish(msg);
        
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    
    return 0;
}