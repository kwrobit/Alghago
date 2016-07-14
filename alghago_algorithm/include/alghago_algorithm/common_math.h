
#include <cstdarg>
#include <cstdlib>
#include <cmath>

#include <eigen3/Eigen/Dense>


using namespace Eigen;



namespace SuhanMath
{

const double EPSILON = 0.00000001;

#define MAX2d(X,Y) ((X>Y)?X:Y)
#define MIN2d(X,Y) ((X<Y)?X:Y)

/**
 * @brief max
 * @param count Max값을 구할 가변 파라미터의 길이
 * @return 최대값을 반환
 * @bug int, long, double, char만 사용 가능 unsigned char, float 등 사용시 오류 발생(Qt5.2 기준)
 */
template <class T>
T Max(int count, ...)
{
    T max = 0;
    int i;

    va_list ap;
    va_start(ap, count);

    for(i=0; i<count; i++)
    {
        T argValue = va_arg(ap, T);
        if(argValue > max)
        {
            max = argValue;
        }
    }
    va_end(ap);
    return max;
}

/**
 * @brief min
 * @param count Max값을 구할 가변 파라미터의 길이
 * @return 최대값을 반환
 * @bug int, long, double, char만 사용 가능 unsigned char, float 등 사용시 오류 발생(Qt5.2 기준)
 */
template <class T>
T Min(int count, ...)
{
    T min;
    int i;

    va_list ap;
    va_start(ap, count);

    min = va_arg(ap, T);
    for(i=1; i<count; i++)
    {
        T argValue = va_arg(ap, T);
        if(argValue < min)
        {
            min = argValue;
        }
    }
    va_end(ap);
    return min;
}


template <class T>
T Square(T &value)
{
    return (value * value);
}



/**
 * @brief cross2D 2D Cross product \n
 * When all points are in 2D plane, this function analysis the magnitude of cross producted vector \n
 * 한글이 잘 안 쳐져서 영어로 썼습니다 양해부탁드려요.
 * @param a
 * @param b
 * @return
 */
double cross2D(const Vector2d &a,
               const Vector2d &b)
{
    return a(0) * b(1) - a(1) * b(0);
}

/**
 * @brief lineIntersection 교점을 구하는 함수
 * @param a_start
 * @param a_end
 * @param b_start
 * @param b_end
 * @param intersection 교점이 저장되는 곳
 * @return 평행할 경우 false 평행이 아닐경우 true
 */
bool lineIntersection(const Vector2d & a_start,
                      const Vector2d & a_end,
                      const Vector2d & b_start,
                      const Vector2d & b_end,
                      Vector2d & intersection)
{
    Vector2d a = a_end - a_start;
    Vector2d b = b_end - b_start;

    double det = cross2D(a,b);

    if(fabs(det) < EPSILON) return false;

    intersection = a_start+(a)*( cross2D((b_start-a_start), b) / det);
    return true;

}


}
