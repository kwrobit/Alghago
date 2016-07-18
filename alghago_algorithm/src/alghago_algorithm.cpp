#include "alghago_algorithm.h"

const double R_NODE = 8.;

const double AlghagoAlgorithm::BOARD_HEIGHT = 450.0;
const double AlghagoAlgorithm::BOARD_WIDTH = 420.0;


void AlghagoAlgorithm::_initialize()
{
    boardVertics[LEFT_TOP] =    Vector2d(0,             0);
    boardVertics[RIGHT_TOP] =   Vector2d(BOARD_WIDTH,   0);
    boardVertics[LEFT_BOT] =    Vector2d(0,             BOARD_HEIGHT);
    boardVertics[RIGHT_BOT] =   Vector2d(BOARD_WIDTH,   BOARD_HEIGHT);
}
void AlghagoAlgorithm::_update_variables()
{
    for(vector<AlghagoNode>::iterator robotNodeItr = robotNodes.begin();
            robotNodeItr != robotNodes.end(); ++robotNodeItr)
    {
        robotNodeItr->pThreaten.resize(userNodes.size());
        robotNodeItr->pThreating.resize(userNodes.size());
        robotNodeItr->pTotal.resize(userNodes.size());
    }
}

void AlghagoAlgorithm::_compute_threating()
{
    for(vector<AlghagoNode>::iterator robotNodeItr = robotNodes.begin();
            robotNodeItr != robotNodes.end(); ++robotNodeItr)
    {
        for(vector<AlghagoNode>::size_type i=0; i<userNodes.size(); i++)
        {
            bool _isAvailable = true;

            double rho, theta, rho_hat;

            _get_rect_available(*robotNodeItr, userNodes[i]);
            _get_line_equation(*robotNodeItr, userNodes[i], rho, theta);

            for(vector<AlghagoNode>::iterator localNode = robotNodes.begin();
                    localNode != robotNodes.end(); ++localNode)
            {
                if (_check_in_range(*localNode, robotNodeItr->rectAvailable))
                {
                    if(_check_on_line(*localNode, rho, theta, R_NODE, rho_hat))
                    {
                        _isAvailable = false;
                        break;
                    }
                }
            }

            if(_isAvailable == false) { robotNodeItr->pThreating[i] = 0.0; continue; }

            for(vector<AlghagoNode>::const_iterator localNode = userNodes.begin();
                    localNode != userNodes.end(); ++localNode)
            {
                if (_check_in_range(*localNode, robotNodeItr->rectAvailable))
                {
                    if(_check_on_line(*localNode, rho, theta, R_NODE, rho_hat))
                    {
                        _isAvailable = false;
                        break;
                    }
                }
            }

            if(_isAvailable == false) { robotNodeItr->pThreating[i] = 0.0; continue; }

            Vector2d _intersectionPoint;

        }

    }
    /*
    for(vector<AlghagoNode>::size_type i=0; i<robotNodes.size(); i++)
    {
        for(vector<AlghagoNode>::size_type j=0; j<userNodes.size(); j++)
        {

            // get rect_disturbance

        }
    }
    */
}

bool AlghagoAlgorithm::_check_in_range(const AlghagoNode &node, const Matrix2d &range)
{
    if (node.coordinate(0) < range(0,0)) return false;
    if (node.coordinate(1) < range(0,1)) return false;
    if (node.coordinate(0) > range(1,0)) return false;
    if (node.coordinate(1) > range(1,1)) return false;
    return true;
}

bool AlghagoAlgorithm::_check_on_line(const AlghagoNode &node, double rho, double theta, double r, double & rho_hat)
{
    double D = -node.coordinate(0) * sin(theta) + node.coordinate(1) * cos(theta) - rho; // pan byel sik

    if ( D > 2 * r)
        return false;
    if (D < - 2 * r)
        return false;

    rho_hat = D + rho;
    return true;
}

void AlghagoAlgorithm::_get_line_equation(const AlghagoNode& node1, const AlghagoNode& node2, double& rho, double& theta)
{
    theta = atan2(node1.coordinate(1) - node2.coordinate(1), node1.coordinate(0) - node2.coordinate(0));
    rho = -node1.coordinate(0) * sin(theta) + node1.coordinate(1) * cos(theta);
}

void AlghagoAlgorithm::_get_line_equation(const AlghagoNode& node1, const AlghagoNode& node2, Vector2d& intersection)
{
    Vector2d _vectorSub = node1.coordinate - node2.coordinate; // dst - src
    Vector2d _tempPoint;


    if(_vectorSub(0) >= 0)
    {
        if(lineIntersection(robotNodeItr->coordinate, userNode[i].coordinate,
                         boardVertics[RIGHT_TOP], boardVertics[RIGHT_BOT], _tempPoint))
        {
            if(_tempPoint(1) >= 0 && _tempPoint(1) <= BOARD_HEIGHT)
            {
                intersection = _tempPoint;
                return;
            }
        }
    }
    else
    {
        if(lineIntersection(robotNodeItr->coordinate, userNode[i].coordinate,
                         boardVertics[LEFT_TOP], boardVertics[LEFT_BOT], _tempPoint))
        {
            if(_tempPoint(1) >= 0 && _tempPoint(1) <= BOARD_HEIGHT)
            {
                intersection = _tempPoint;
                return;
            }
        }
    }
    if(_vectorSub(1) >= 0)
    {
        if(lineIntersection(robotNodeItr->coordinate, userNode[i].coordinate,
                         boardVertics[LEFT_BOT], boardVertics[RIGHT_BOT], _tempPoint))
        {
            if(_tempPoint(0) >= 0 && _tempPoint(0) <= BOARD_WIDTH)
            {
                intersection = _tempPoint;
                return;
            }
        }
    }
    else
    {
        if(lineIntersection(robotNodeItr->coordinate, userNode[i].coordinate,
                         boardVertics[LEFT_TOP], boardVertics[RIGHT_TOP], _tempPoint))
        {
            if(_tempPoint(0) >= 0 && _tempPoint(0) <= BOARD_WIDTH)
            {
                intersection = _tempPoint;
                return;
            }
        }
    }

}
void AlghagoAlgorithm::_get_rect_available(AlghagoNode& robotNode, const AlghagoNode& userNode)
{
    // get rect_available
    robotNode.rectAvailable(0,0) =
            MIN2d(robotNode.coordinate(0),
                  userNode.coordinate(0));
    robotNode.rectAvailable(0,1) =
            MIN2d(robotNode.coordinate(1),
                  userNode.coordinate(1));
    robotNode.rectAvailable(1,0) =
            MAX2d(robotNode.coordinate(0),
                  userNode.coordinate(0));
    robotNode.rectAvailable(1,1) =
            MAX2d(robotNode.coordinate(1),
                  userNode.coordinate(1));

}

void AlghagoAlgorithm::_get_rect_disturbance(AlghagoNode& robotNode, const AlghagoNode& userNode)
{

    // TODO: make it.

    // get rect_available
    robotNode.rectAvailable(0,0) =
            MIN2d(robotNode.coordinate(0),
                  userNode.coordinate(0));
    robotNode.rectAvailable(0,1) =
            MIN2d(robotNode.coordinate(1),
                  userNode.coordinate(1));
    robotNode.rectAvailable(1,0) =
            MAX2d(robotNode.coordinate(0),
                  userNode.coordinate(0));
    robotNode.rectAvailable(1,1) =
            MAX2d(robotNode.coordinate(1),
                  userNode.coordinate(1));

}

void AlghagoAlgorithm::_compute_threated()
{

}
void AlghagoAlgorithm::_choose_node()
{
    for(int i=0; i<(int)robotNodes.size(); i++)
    {

    }
}
