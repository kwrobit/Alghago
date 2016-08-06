#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class BadukpanExtractor
{
    image_transport::ImageTransport it;
    image_transport::Subscriber camera_sub;

    image_transport::Publisher processed_pub;

    Mat warpPerspectiveMatrix;

    bool isFirst;
    int count;


public:

    BadukpanExtractor(ros::NodeHandle& nh) : it(nh), count(0), isFirst(true)
    {
        //image_transport::TransportHints hints("compressed", ros::TransportHints(), nh);
        camera_sub = it.subscribe("/usb_cam/image_rect_color", 1,  &BadukpanExtractor::imageCallback, this);
        processed_pub = it.advertise("/badukpan/image_raw", 1);
    }
    virtual ~BadukpanExtractor() {}

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
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
        cv::Mat badukpanImg;
        cv::Mat resultImg;


        if(isFirst)
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
            dst_vertices[1] = cv::Point2f(420,0);
            dst_vertices[2] = cv::Point2f(0,450);
            dst_vertices[3] = cv::Point2f(420,450);

            warpPerspectiveMatrix = getPerspectiveTransform(src_vertices,dst_vertices);
            isFirst = false;
            return ;
        }
        else
        {
            cv::Size size(420, 450);
            warpPerspective(srcImg, badukpanImg, warpPerspectiveMatrix, size, INTER_CUBIC, BORDER_CONSTANT);
            Mat grayImg;

            cvtColor( badukpanImg, grayImg, COLOR_BGR2GRAY );

            medianBlur(grayImg, grayImg, 9);
            //GaussianBlur( grayImg, grayImg, Size(9, 9), 2, 2 );

            vector<Vec3f> circles;
            HoughCircles( grayImg, circles,CV_HOUGH_GRADIENT, 2, 12, 40, 25, 9, 14);

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

               const int BLACK_THRESH = 130;
               const int WHITE_THRESH = 600;

               if(centerPixSum > WHITE_THRESH)
               {
                   // circle center
                   circle( badukpanImg, center, 3, Scalar(0,255,0), -1, 8, 0 );
                   // circle outline
                   circle( badukpanImg, center, radius, Scalar(255,0,0), 2 , 8, 0 );
                   //ROS_INFO("WHT, r=%d",radius);

               }
               else if(centerPixSum < BLACK_THRESH)
               {
                   // circle center
                   circle( badukpanImg, center, 3, Scalar(0,255,0), -1, 8, 0 );
                   // circle outline
                   circle( badukpanImg, center, radius, Scalar(0,0,255), 2 , 8, 0 );
                   //ROS_INFO("BLK, r=%d",radius);
               }
             }

        }



        // write
        std_msgs::Header header;
        header.seq = count;
        header.stamp = ros::Time::now();

        count ++;
        cv_bridge::CvImage dstImage_bridge(header, sensor_msgs::image_encodings::BGR8, badukpanImg);
        //cv_bridge::CvImage dstImage_bridge(header, sensor_msgs::image_encodings::MONO8, grayImg);
        processed_pub.publish(dstImage_bridge.toImageMsg());

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "badukpan_extractor");
    ros::NodeHandle nh;

    BadukpanExtractor baduk(nh);

    ros::spin();

	return 0;
}
