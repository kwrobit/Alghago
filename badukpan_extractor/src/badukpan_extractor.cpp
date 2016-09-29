#include "badukpan_extractor.h"

using namespace std;
using namespace cv;


BadukpanExtractor::BadukpanExtractor(ros::NodeHandle& nh) : it_(nh), count_(0), isFirst_(true), filterCount_(0)
{
    //image_transport::TransportHints hints("compressed", ros::TransportHints(), nh);
    camera_sub_ = it_.subscribe("/usb_cam/image_raw", 1,  &BadukpanExtractor::imageCallback, this);
    processed_pub_ = it_.advertise("/badukpan/image_raw", 1);
    badukal_pub_ = nh.advertise<alghago_msgs::BadukalArray>("/badukpan/badukals", 1);
    badukpan_fit_sub_ = nh.subscribe("/badukpan/fit", 1, &BadukpanExtractor::badukpanFitCallback, this);
}

void BadukpanExtractor::fitBadukpan(Mat& srcImg)
{
    cv::Mat grayImg;
    cv::cvtColor(srcImg,grayImg,CV_BGR2GRAY);
    cv::blur( grayImg, grayImg, Size(3,3) );

    Mat element = getStructuringElement( MORPH_RECT,
                                         Size( 2*5 + 1, 2*5+1 ),
                                         Point(5, 5) );


    cv::erode(grayImg,grayImg,element);
    cv::erode(grayImg,grayImg,element);
    cv::dilate(grayImg,grayImg,element);
    cv::dilate(grayImg,grayImg,element);
    cv::Mat dstImg(srcImg.size(), srcImg.type());
    cv::Mat cannyImg;

    // ROS_INFO("Type= %d, Real= %d",CV_8UC1,grayImg.type());
    // http://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/find_contours/find_contours.html
    // http://hongkwan.blogspot.kr/2013/01/opencv-7-4-example.html
    cv::Canny(grayImg, cannyImg, 50, 200);

    std::vector< std::vector<cv::Point> > contours;
    std::vector<Vec4i> hierarchy;
    cv::findContours(cannyImg,contours, hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);

    vector< vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );

    int maxIndex = -1;
    int maxValue = 0;
    for( int i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        //minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
        if(maxValue < boundRect[i].area()){
            maxValue = boundRect[i].area();
            maxIndex = i;
        }
            //rectangle( dstImg, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
    }
    if(maxIndex == -1) return;
    dstImg = srcImg.clone();// (boundRect[maxIndex]);


    Rect & detected_rect = boundRect[maxIndex];
    std::vector<cv::Point> & detected_contours_poly= contours_poly[maxIndex];


    Scalar color_blue = Scalar( 255, 0, 0 );
    Scalar color_red = Scalar( 0, 0, 255 );
    rectangle( dstImg, detected_rect.tl(), detected_rect.br(), color_blue, 2, 8, 0 );

    enum side {LEFT_UP, RIGHT_UP, LEFT_BOTTOM, RIGHT_BOTTOM};
    int min[4];
    int index[4] = {0, };
    for(int i=0; i<4;i++)
    {
        min[i] = (detected_rect.width + detected_rect.height);
    }
    for(int i=0; i< detected_contours_poly.size(); i++)
    {
        cv::Point & _p = detected_contours_poly[i];
        int _dists[4];
        _dists[LEFT_UP] = abs(_p.x - detected_rect.x) + abs(_p.y - detected_rect.y);
        _dists[RIGHT_UP] = abs(_p.x - detected_rect.x - detected_rect.width) + abs(_p.y - detected_rect.y);
        _dists[LEFT_BOTTOM] = abs(_p.x - detected_rect.x) + abs(_p.y - detected_rect.y - detected_rect.height);
        _dists[RIGHT_BOTTOM] = abs(_p.x - detected_rect.x- detected_rect.width) + abs(_p.y - detected_rect.y - detected_rect.height);
        for(int j=0; j<4; j++)
        {
            if(_dists[j] < min[j])
            {
                min[j] = _dists[j];
                index[j] = i;
            }
        }
    }
    for(int i=0; i<4; i++)
    {
        circle(dstImg, detected_contours_poly[index[i]],2,color_red,3);
    }

    cv::Point2f src_vertices[3];
    src_vertices[0] = detected_contours_poly[index[0]];
    src_vertices[1] = detected_contours_poly[index[1]];
    src_vertices[2] = detected_contours_poly[index[2]];
    src_vertices[3] = detected_contours_poly[index[3]];

    cv::Point2f dst_vertices[3];
    dst_vertices[0] = cv::Point2f(0,0);
    dst_vertices[1] = cv::Point2f(396,0);
    dst_vertices[2] = cv::Point2f(0,424);
    dst_vertices[3] = cv::Point2f(396,424);

    warpPerspectiveMatrix_ = getPerspectiveTransform(src_vertices,dst_vertices);
}

void BadukpanExtractor::scanBadukal(Mat& srcImg, Mat& badukpanImg)
{
    cv::Size size(396, 424);
    warpPerspective(srcImg, badukpanImg, warpPerspectiveMatrix_, size, INTER_CUBIC, BORDER_CONSTANT);
    Mat grayImg;

    cvtColor( badukpanImg, grayImg, COLOR_BGR2GRAY );

    //GaussianBlur( grayImg, grayImg, Size(9, 9), 2, 2 );
    //medianBlur(grayImg, grayImg, 3);

    vector<Vec3f> circles;
    HoughCircles( grayImg, circles,CV_HOUGH_GRADIENT, 2, 12, 40, 25, 10, 13);


    // clear badukal vector
    badukalPoints_[filterCount_].clear();

    for( size_t i = 0; i < circles.size(); i++ )
    {

       Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

       int radius = cvRound(circles[i][2]);

       float centerPixSum = 0;
       for(int x=-10; x<11; x++)
       {
           for(int y=-10; y<11; y++)
           {
               Vec3b pix = badukpanImg.at<Vec3b>(center.y,center.x);
               centerPixSum += (pix[0] + pix[1] + pix[2]) / (20.0*20.0);
           }
       }

       const int BLACK_THRESH = 200;
       const int WHITE_THRESH = 600;

       if(centerPixSum > WHITE_THRESH)
       {
           Badukal _al;
           _al.color = Badukal::WHITE;
           _al.x = center.y;
           _al.y = center.x;

           badukalPoints_[filterCount_].push_back(_al);

           // circle center
           circle( badukpanImg, center, 3, Scalar(0,255,0), -1, 8, 0 );
           // circle outline
           circle( badukpanImg, center, radius, Scalar(255,0,0), 2 , 8, 0 );
           //ROS_INFO("WHT, r=%d",radius);

       }
       else if(centerPixSum < BLACK_THRESH)
       {
           Badukal _al;
           _al.color = Badukal::BLACK;
           _al.x = center.y;
           _al.y = center.x;

           badukalPoints_[filterCount_].push_back(_al);
           // circle center
           circle( badukpanImg, center, 3, Scalar(0,255,0), -1, 8, 0 );
           // circle outline
           circle( badukpanImg, center, radius, Scalar(0,0,255), 2 , 8, 0 );
           //ROS_INFO("BLK, r=%d",radius);
       }
     }

    filterCount_ ++;
    if(filterCount_ == FILTER_CNT)
    {
        filterCount_ = 0;
        int _maxNum = 0;
        int _maxIndex = 0;
        for(int i=0; i<FILTER_CNT; i++)
        {
            if(_maxNum < badukalPoints_[i].size())
            {
                _maxIndex = i;
                _maxNum = badukalPoints_[i].size();
            }
        }
        alghago_msgs::BadukalArray msg;
        msg.header.stamp = ros::Time::now();
        for(int i=0; i<badukalPoints_[_maxIndex].size(); i++)
        {
            alghago_msgs::Badukal b;
            b.x = badukalPoints_[_maxIndex][i].x;
            b.y = badukalPoints_[_maxIndex][i].y;
            b.color = badukalPoints_[_maxIndex][i].color;
            msg.badukals.push_back(b);
        }
        badukal_pub_.publish(msg);
    }
}

void BadukpanExtractor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat& srcImg = cv_ptr->image;

    if(isFirst_)
    {
        fitBadukpan(srcImg);
        isFirst_ = false;
        return ;
    }
    else
    {
        cv::Mat badukpanImg;
        scanBadukal(srcImg, badukpanImg);

        // write
        std_msgs::Header header;
        header.seq = count_;
        header.stamp = ros::Time::now();

        count_ ++;
        cv_bridge::CvImage dstImage_bridge(header, sensor_msgs::image_encodings::BGR8, badukpanImg);
        //cv_bridge::CvImage dstImage_bridge(header, sensor_msgs::image_encodings::MONO8, grayImg);
        processed_pub_.publish(dstImage_bridge.toImageMsg());
    }
}

void BadukpanExtractor::badukpanFitCallback(const std_msgs::EmptyConstPtr msg)
{
    (void)msg;
    isFirst_ = true;
}
