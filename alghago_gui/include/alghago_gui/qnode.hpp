/**
 * @file /include/alghago_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef alghago_gui_QNODE_HPP_
#define alghago_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <smach_msgs/SmachContainerStatus.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <QThread>
#include <QStringListModel>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace alghago_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void node_init();
	void run();


    void state_transition(const char *c_str)
    {
        std::string str(c_str);
        std_msgs::String msg;
        msg.data = str;
        smach_publisher.publish(msg);
    }

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

    std::string getState() {
        return smach_state;
    }
    cv::Mat & getImage() {
        return badukpan_image;
    }

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void updateImage();
    void updateState();


private:
	int init_argc;
	char** init_argv;

    ros::NodeHandle *nh;
    image_transport::ImageTransport *it;
	ros::Publisher chatter_publisher;
    ros::Subscriber smach_subscriber;
    ros::Publisher smach_publisher;
    QStringListModel logging_model;

    std::string smach_state;
    image_transport::Subscriber badukpan_subscriber;
    cv::Mat badukpan_image;


private:
    void state_callback(const smach_msgs::SmachContainerStatusConstPtr& msg);
    void badukpan_image_callback(const sensor_msgs::ImageConstPtr& msg);
};

}  // namespace alghago_gui

#endif /* alghago_gui_QNODE_HPP_ */
