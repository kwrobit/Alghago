#include <vector>
#include <eigen3/Eigen/Dense>

#include "common_math.h"

using namespace std;
using namespace Eigen;
using namespace SuhanMath;


// Here, you can add some new probablity gains:)
const double kThreaten = 0.4;
const double kThreating = 0.6;


struct AlghagoNode
{
    Vector2d coordinate;

    Matrix2d rectAvailable;
    Matrix2d rectDisturbance;

    // Here, you can add some new probablity :)
    vector<double> pThreaten;
    vector<double> pThreating;


    vector<double> pTotal;
};


class AlghagoAlgorithm
{
    // Sum of all gains should be 1

    int shootIndex;
    int targetIndex;

    vector<AlghagoNode> robotNodes;
    vector<AlghagoNode> userNodes;

public:
    AlghagoAlgorithm() {}

    void updateNodes(vector<AlghagoNode> &updateRobotNodes,
                     vector<AlghagoNode> &updateUserNodes)
    {
        robotNodes = updateRobotNodes;
        userNodes = updateUserNodes;
    }

    void compute()
    {
        _compute_threating();
        _compute_threated();
        _choose_node();
    }
    void write(Vector2d &shoot, Vector2d &target)
    {
        shoot = robotNodes[shootIndex].coordinate;
        target = userNodes[targetIndex].coordinate;
    }


private:
    void _compute_threating()
    {
        for(int i=0; i<(int)robotNodes.size(); i++)
        {

        }
    }
    void _compute_threated()
    {

    }
    void _choose_node()
    {
        for(int i=0; i<(int)robotNodes.size(); i++)
        {

        }
    }
};
