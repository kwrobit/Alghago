
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <smach_msgs/SmachContainerStatus.h>
#include "alghago_msgs/BadukalArray.h"

#include "alghago_algorithm.h"



class AlghagoAlgorithmROS
{
    ros::Subscriber badukal_sub_;
    ros::Publisher result_pub_;

    ros::NodeHandle nh_;
    AlghagoAlgorithm AA_;


    ros::Subscriber smach_sub_;
    ros::Publisher smach_pub_;

    std::string smach_state_;
    std::string smach_state_old_;

    bool algorithm_calc_start_;

public:
    AlghagoAlgorithmROS() : algorithm_calc_start_(false)
    {
        badukal_sub_ = nh_.subscribe("/badukpan/badukals",1, &AlghagoAlgorithmROS::badukalCallback, this);
        result_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/alghago_ai/result", 1);

        smach_pub_ = nh_.advertise<std_msgs::String>("alghago_sm/transition",5);
        smach_sub_ = nh_.subscribe("/alghago_sm/smach/container_status", 1,
                                        &AlghagoAlgorithmROS::stateCallback, this);
    }

private:
    void stateTigger()
    {
        if(smach_state_ != smach_state_old_)
        {
            smach_state_old_ = smach_state_;
            if(smach_state_ == "Thinking")
            {
                algorithm_calc_start_ = true;
            }
        }
    }

    void stateTransition(const char *c_str)
    {
        std::string str(c_str);
        std_msgs::String msg;
        msg.data = str;
        smach_pub_.publish(msg);
    }

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
        if(algorithm_calc_start_)
        {
            algorithm_calc_start_ = false;
            stateTransition("think_done");

        }
    }


    void stateCallback(const smach_msgs::SmachContainerStatusConstPtr &msg)
    {
        smach_state_ = msg->active_states[0];
        stateTigger();
    }

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "alghago_algorithm");

    AlghagoAlgorithmROS aa;

    ros::spin();

    return 0;
}
