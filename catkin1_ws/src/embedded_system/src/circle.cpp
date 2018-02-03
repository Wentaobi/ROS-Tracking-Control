#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

// Needed to convert rotation ...
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

class circle_line
{
public:
    circle_line(ros::NodeHandle& nh);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void waypoints(double x, double y);

    void spin();
protected:
    ros::Subscriber odomSub;
    ros::Publisher way_points;

    double odom_x,odom_y,odom_a;
    double t,x,y;
};
circle_line::circle_line(ros::NodeHandle& nh)
{
    odomSub=nh.subscribe("odom",1,&circle_line::odomCallback,this);
    way_points=nh.advertise<std_msgs::Float64MultiArray>("way_points", 1);
    t=0;
}

void circle_line::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_x=msg->pose.pose.position.x;
    odom_y=msg->pose.pose.position.y;
}
void circle_line::waypoints(double x, double y)
{
    std_msgs::Float64MultiArray way_points_value;
    way_points_value.data.clear();

    way_points_value.data.push_back(x);
    way_points_value.data.push_back(y);
    way_points.publish(way_points_value);
}
void circle_line::spin()
{
    ros::Rate rate(10);
    while(ros::ok())
    {
        srand(time(0));

        //lines

        x=7*cos(t);
        y=7*sin(t);

        if(sqrt((x-odom_x)*(x-odom_x)+(y-odom_y)*(y-odom_y))<1) t=t+0.08;

        waypoints(x, y);

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"way_points_publisher");
    ros::NodeHandle n;
    circle_line tra(n);
    tra.spin();
    return 0;
}
