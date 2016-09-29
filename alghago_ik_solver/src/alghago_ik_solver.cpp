#include "alghago_ik_solver/alghago_ik_solver.h"

using namespace SuhanMath;

void AlghagoIKSolver::solve(const Vector3d& target_position, // 0 = X, 1 = Y, 2 = Theta
           Vector3d& target_q)
{
    // TODO: Write the variable information.
    double x = target_position(0);
    double y = target_position(1);
    double l1 = link_length_(0);
    double l2 = link_length_(1);
    double theta = target_position(2);
    double k1, k2;
    double q1, q2, q3;
    double sin1, cos1;

    cos1 = ( SQR(x) + SQR(y) - SQR(l1) - SQR(l2) ) / (2 * l1 * l2);

    if(fabs(cos1) >= 1.0)  // fabs makes you more comfortable.
    {
        ROS_ERROR("IK: Out of Range!");
    }

    // Calculation
    sin1 = sqrt(1-SQR(cos1));
    q2 = atan2(sin1, cos1);

    k1 = l1 + l2*cos(q2);
    k2 = l2 * sin(q2);

    q1 = atan2(y,x) - atan2(k2,k1);
    q3 = -(theta - q1 - q2);   // Every unit should be SI. I fixed it to radian

    if(fabs(q3) > 166.667 * DEG2RAD)
    {
        ROS_INFO("IK: Down Elbow");
        // Why did you calculate the same thing twice?
        //cos1 = ( SQR(X) + SQR(Y) - ( SQR(l1) + SQR(l2) ) ) / (2 * l1 * l2);

        sin1 = - sin1;  // sin1 = - sqrt(1-SQR(cos1));

        q2 = atan2(sin1, cos1);

        k1 = l1 + l2*cos(q2);
        k2 = l2 * sin(q2);

        q1 = atan2(y,x) - atan2(k2,k1);
        q3 = -(theta - q1 - q2);   // Every unit should be SI. I fixed it to radian
    }
    else
    {
        ROS_INFO("IK: Up Elbow");
    }

    // Writing
    target_q(0) = q1;
    target_q(1) = q2;
    target_q(2) = q3;
}



