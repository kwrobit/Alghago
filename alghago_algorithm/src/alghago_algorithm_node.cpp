
#include <ros/ros.h>
#include "alghago_algorithm.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "alghago_algorithm");

    AlghagoAlgorithm AA;

    vector<Vector2d> robotSampleNodes;
    vector<Vector2d> userSampleNodes;

    robotSampleNodes.push_back(Vector2d(100, 100));
    robotSampleNodes.push_back(Vector2d(100, 200));

    userSampleNodes.push_back(Vector2d(100, 150));

    AA.updateNodes(robotSampleNodes,userSampleNodes);
    AA.compute();


    return 0;
}
