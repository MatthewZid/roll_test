#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/conversions.h"

bool cntrl=true;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    float min=100.0f;
    
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    
    for(auto& pt : temp_cloud->points){
        if(pt.z==pt.z and pt.z<min and pt.y==pt.y and pt.y>=-0.1f and pt.y<=0.1f /*and pt.x>=-0.1f and pt.x<=0.1f*/){
            min=pt.z;
            printf("%f, %f\n",pt.y, pt.z);
        }
    }
    
    if(min<=1.f){
        cntrl=false;
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