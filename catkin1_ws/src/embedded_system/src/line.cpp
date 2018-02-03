#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

class line
{
public:
    line(ros::NodeHandle& nh);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void waypoints(double x, double y);

    void spin();
protected:
    ros::Subscriber odomSub;
    ros::Publisher way_points;

    double odom_x,odom_y,odom_a;
    double t,x,y;
};
line::line(ros::NodeHandle& nh)
{
    odomSub=nh.subscribe("odom",1,&line::odomCallback,this);
    way_points=nh.advertise<std_msgs::Float64MultiArray>("way_points", 1);
    t=-9;
}

void line::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_x=msg->pose.pose.position.x;
    odom_y=msg->pose.pose.position.y;
}
void line::waypoints(double x, double y)
{
    std_msgs::Float64MultiArray way_points_value;
    way_points_value.data.clear();

    way_points_value.data.push_back(x);
    way_points_value.data.push_back(y);
    way_points.publish(way_points_value);
}
void line::spin()
{
    ros::Rate rate(10);
    while(ros::ok())
    {
        srand(time(0));

        //lines

        x=t;
        y=t;

        if(sqrt((x-odom_x)*(x-odom_x)+(y-odom_y)*(y-odom_y))<1) t=t++;

        waypoints(x, y);

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"way_points_publisher");
    ros::NodeHandle n;
    line tra(n);
    tra.spin();
    return 0;
}
