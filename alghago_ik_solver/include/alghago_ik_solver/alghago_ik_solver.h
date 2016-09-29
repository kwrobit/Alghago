
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <suhan_math/common_math.h>
#include <cmath>

using namespace Eigen;

class AlghagoIKSolver
{

    Vector2d link_length_;

public:
    AlghagoIKSolver(const Vector2d& link_length) : link_length_(link_length) {}

    void solve(const Vector3d& target_position, // 0 = X, 1 = Y, 2 = Theta
                     Vector3d& target_q);

};
