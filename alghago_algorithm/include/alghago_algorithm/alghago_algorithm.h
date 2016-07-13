#include <vector>
#include "common_math.h"

using namespace std;



struct AlghagoNode
{
    dPoint2D coordinate;

    vector<double> pThreaten;
    vector<double> pThreating;
    vector<double> pTotal;
};

class AlghagoAlgorithm
{
    // Sum of all gains should be 1
    const double kThreaten = 0.4;
    const double kThreating = 0.6;

    int shootIndex;
    int targetIndex;

    vector<AlghagoNode> robotNodes;
    vector<AlghagoNode> userNodes;

public:
    AlghagoAlgorithm() {}

    void updateNodes(vector<AlghagoNode> &updateRobotNodes, vecotr<AlghagoNode> &updateUserNodes)
    {
        robotNodes = updateRobotNodes;
        userNodes = updateUserNodes;
    }

    void choose()
    {
        for(int i=0; i<(int)robotNodes.size(); i++)
        {

        }
    }

    void compute()
    {

    }

};
