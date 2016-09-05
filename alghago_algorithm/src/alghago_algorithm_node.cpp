
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "alghago_msgs/BadukalArray.h"

#include "alghago_algorithm.h"



class AlghagoAlgorithmROS
{
    ros::Subscriber badukal_sub_;
    ros::Publisher result_pub_;

    ros::NodeHandle nh_;
    AlghagoAlgorithm AA_;

public:
    AlghagoAlgorithmROS()
    {
        badukal_sub_ = nh_.subscribe("/badukpan/badukals",1, &AlghagoAlgorithmROS::badukalCallback, this);
        result_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/alghago_ai/result", 1);

    }

private:
    void badukalCallback(const alghago_msgs::BadukalArrayConstPtr msg)
    {
        vector<Vector2d> robotNodes;
        vector<Vector2d> userNodes;


        for(unsigned int i=0; i < msg->badukals.size(); i++)
        {
            if(msg->badukals[i].color == alghago_msgs::Badukal::BLACK)
            {
                robotNodes.push_back(Vector2d(msg->badukals[i].x, msg->badukals[i].y));
            }
            else
            {
                userNodes.push_back(Vector2d(msg->badukals[i].x, msg->badukals[i].y));
            }
        }

        AA_.updateNodes(robotNodes,userNodes);
        if(AA_.compute())   // Checking whether target is available.
        {
            Vector2d shootCoord;
            Vector2d targetCoord;
            AA_.write(shootCoord,targetCoord);
            std_msgs::Float32MultiArray coordMsg;
            coordMsg.data.push_back(shootCoord(0));
            coordMsg.data.push_back(shootCoord(1));
            coordMsg.data.push_back(targetCoord(0));
            coordMsg.data.push_back(targetCoord(1));

            result_pub_.publish(coordMsg);
        }
    }

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "alghago_algorithm");

    AlghagoAlgorithmROS aa;

    ros::spin();

    return 0;
}
