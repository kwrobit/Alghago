#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseArray.h>
#include "alghago_msgs/BadukalArray.h"


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen3/Eigen/Dense>


using namespace std;
using namespace cv;

class BadukpanExtractor
{
    image_transport::ImageTransport it_;
    image_transport::Subscriber camera_sub_;

    image_transport::Publisher processed_pub_;

    ros::Publisher badukal_pub_;

    Mat warpPerspectiveMatrix_;

    bool isFirst_;
    int count_;
    int filterCount_;

    vector<Eigen::Vector2d> badukalPoints_;


public:

    BadukpanExtractor(ros::NodeHandle& nh);
    virtual ~BadukpanExtractor() {}


private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void fitBadukpan(Mat &srcImg);
    void scanBadukal(Mat &srcImg, Mat &badukpanImg);

};

