#include "alghago_algorithm.h"


const double AlghagoAlgorithm::R_NODE = 15.;
const double AlghagoAlgorithm::BOARD_HEIGHT = 424.0;
const double AlghagoAlgorithm::BOARD_WIDTH = 396.0;


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
    for(vector<AlghagoNode>::size_type i=0; i<robotNodes.size(); i++)
    {
        for(vector<AlghagoNode>::size_type j=0; j<userNodes.size(); j++)
        {
            double rho, theta, occupancy, dist;


            _get_rect_available(robotNodes[i], userNodes[j]);
            _get_line_equation(robotNodes[i], userNodes[j], rho, theta);
            ROS_INFO("rho=%.2lf, theta=%.2lf",rho,theta*RAD2DEG);

            Vector2d _intersectionPoint;
            _get_board_intersection(robotNodes[i], userNodes[j], _intersectionPoint);
            _get_rect_disturbance(robotNodes[i],userNodes[j],_intersectionPoint);

            // if an obstacle is on line, pass to next;
            robotNodes[i].pThreating[j] = 0.0;
            if(_local_search_in_rect(robotNodes[i].rectAvailable, rho, theta, i,j)) continue;

            ROS_INFO("pass");

            _local_search_in_rect(robotNodes[i].rectDisturbance, rho, theta, i, j, occupancy);
            dist = _get_distance(robotNodes[i], userNodes[j]);
            robotNodes[i].pThreating[j] = (1.0 - occupancy) / dist;

        }
    }
}

void AlghagoAlgorithm::_compute_threated()
{
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

    ROS_INFO("D=%.2lf",D);
    if ( D > 2 * r)
        return false;
    if (D < - 2 * r)
        return false;

    rho_hat = D + rho;
    ROS_INFO("rho_hat=%.2lf, rho=%.2lf",rho_hat,rho);
    return true;
}
double AlghagoAlgorithm::_get_distance(const AlghagoNode& node1, const AlghagoNode& node2)
{
    return sqrt(SQR(node1.coordinate(0) - node2.coordinate(0)) +
                SQR(node1.coordinate(1) - node2.coordinate(1)));
}

void AlghagoAlgorithm::_get_line_equation(const AlghagoNode& node1, const AlghagoNode& node2, double& rho, double& theta)
{
    theta = atan2(node1.coordinate(1) - node2.coordinate(1), node1.coordinate(0) - node2.coordinate(0));
    rho = -node1.coordinate(0) * sin(theta) + node1.coordinate(1) * cos(theta);
    ROS_INFO("-- %.2lf %.2lf %.2lf %.2lf",node1.coordinate(0), node1.coordinate(1), node2.coordinate(0), node2.coordinate(1));

}

void AlghagoAlgorithm::_get_board_intersection(const AlghagoNode& node1, const AlghagoNode& node2, Vector2d& intersection)
{
    Vector2d _vectorSub = node2.coordinate - node1.coordinate; // dst - src
    Vector2d _tempPoint;


    if(_vectorSub(0) >= 0)
    {
        if(lineIntersection(node1.coordinate, node2.coordinate,
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
        if(lineIntersection(node1.coordinate, node2.coordinate,
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
        if(lineIntersection(node1.coordinate, node2.coordinate,
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
        if(lineIntersection(node1.coordinate, node2.coordinate,
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
                  userNode.coordinate(0)) - R_NODE;
    robotNode.rectAvailable(0,1) =
            MIN2d(robotNode.coordinate(1),
                  userNode.coordinate(1)) - R_NODE;
    robotNode.rectAvailable(1,0) =
            MAX2d(robotNode.coordinate(0),
                  userNode.coordinate(0)) + R_NODE;
    robotNode.rectAvailable(1,1) =
            MAX2d(robotNode.coordinate(1),
                  userNode.coordinate(1)) + R_NODE;

}

void AlghagoAlgorithm::_get_rect_disturbance(AlghagoNode& robotNode, const AlghagoNode& userNode, const Vector2d& intersection)
{
    // get rect_disturbance
    robotNode.rectDisturbance(0,0) =
            MIN2d(userNode.coordinate(0),
                  intersection(0)) - R_NODE;
    robotNode.rectDisturbance(0,1) =
            MIN2d(userNode.coordinate(1),
                  intersection(1)) - R_NODE;
    robotNode.rectDisturbance(1,0) =
            MAX2d(userNode.coordinate(0),
                  intersection(0)) + R_NODE;
    robotNode.rectDisturbance(1,1) =
            MAX2d(userNode.coordinate(1),
                  intersection(1)) + R_NODE;

}

/**
 * @brief AlghagoAlgorithm::_local_search_in_rect
 * @param range
 * @param rho
 * @param theta
 * @return return true if at least one node is on line
 */
bool AlghagoAlgorithm::_local_search_in_rect(const Matrix2d &range, double rho, double theta, int robot_start, int user_start)
{
    double _rho_hat;
    for(vector<AlghagoNode>::size_type i=0; i<robotNodes.size(); i++)
    {
        if (i == robot_start) continue;
        if (_check_in_range(robotNodes[i], range))
        {
            if(_check_on_line(robotNodes[i], rho, theta, R_NODE, _rho_hat))
            {
                return true;
            }
        }
    }
    for(vector<AlghagoNode>::size_type i=0; i<userNodes.size(); i++)
    {
        if (i == user_start) continue;
        if (_check_in_range(userNodes[i], range))
        {
            if(_check_on_line(userNodes[i], rho, theta, R_NODE, _rho_hat))
            {
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief AlghagoAlgorithm::_local_search_in_rect
 * @param range
 * @param rho
 * @param theta
 * @param occupancy 0.0 ~ 1.0
 * @return return true if at least one node is on line
 */
bool AlghagoAlgorithm::_local_search_in_rect(const Matrix2d &range, double rho, double theta, int robot_start, int user_start, double& occupancy)
{
    double _rho_hat;
    bool _isOnLine = false;
    double _occupancy;
    occupancy = 0.0;
    _occupancy = 1.0;

    for(vector<AlghagoNode>::size_type i=0; i<robotNodes.size(); i++)
    {
        if (i == robot_start) continue;
        if (_check_in_range(robotNodes[i], range))
        {
            if(_check_on_line(robotNodes[i], rho, theta, R_NODE, _rho_hat))
            {
                occupancy = 1.0;
                return true;
            }
        }
    }
    for(vector<AlghagoNode>::size_type i=0; i<userNodes.size(); i++)
    {
        if (i == user_start) continue;
        if (_check_in_range(userNodes[i], range))
        {
            if(_check_on_line(userNodes[i], rho, theta, R_NODE, _rho_hat))
            {

                _occupancy = fabs(rho-_rho_hat) / (2 * R_NODE);

                if(_occupancy > occupancy) occupancy = _occupancy;

                _isOnLine = true;
            }
        }
    }

    /*

    for(vector<AlghagoNode>::iterator localNode = robotNodes.begin();
            localNode != robotNodes.end(); ++localNode)
    {
        if (_check_in_range(*localNode, range))
        {
            if(_check_on_line(*localNode, rho, theta, R_NODE, _rho_hat))
            {
            }
        }
    }

    for(vector<AlghagoNode>::const_iterator localNode = userNodes.begin();
            localNode != userNodes.end(); ++localNode)
    {
        if (_check_in_range(*localNode, range))
        {
            if(_check_on_line(*localNode, rho, theta, R_NODE, _rho_hat))
            {
                _occupancy = fabs(rho-_rho_hat) / (2 * R_NODE);
                if(_occupancy > occupancy) occupancy = _occupancy;

                _isOnLine = true;
            }
        }
    }
    */
    return _isOnLine;
}

bool AlghagoAlgorithm::_choose_node()
{
    int robotIndex;
    int userIndex;
    double maxProb = 0.0;
    for(int i=0; i<(int)robotNodes.size(); i++)
    {
        for(int j=0; j<(int)robotNodes[i].pThreating.size(); j++)
        {
            if(maxProb < robotNodes[i].pThreating[j])
            {
                maxProb = robotNodes[i].pThreating[j];
                robotIndex = i;
                userIndex = j;
            }
            ROS_INFO("%d node to %d: pThreating = %.5lf",i,j,robotNodes[i].pThreating[j]);
        }
    }



    if ( robotNodes.size() != 0 && userNodes.size() != 0 )    // size available
    {
        ROS_INFO("My %d Node will attack %d Node", robotIndex, userIndex);
        ROS_INFO("%.1lf, %.1lf -> %.1lf, %.1lf",
                 robotNodes[robotIndex].coordinate(0), robotNodes[robotIndex].coordinate(1),
                 userNodes[userIndex].coordinate(0), userNodes[userIndex].coordinate(1));
        shootIndex = robotIndex;
        targetIndex = userIndex;
        return true;
    }

    return false;
}
