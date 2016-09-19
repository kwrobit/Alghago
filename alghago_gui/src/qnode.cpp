/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/alghago_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace alghago_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
    init_argv(argv),
    nh(0), it(0)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    if(nh) delete nh;
    if(it) delete it;
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"alghago_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
    ros::start(); // explicitly needed since our nodehandle is going out of scope.

    node_init();

	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"alghago_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
    ros::start(); // explicitly needed since our nodehandle is going out of scope.

    node_init();

    return true;
}

void QNode::node_init()
{
    // Add your ros communications here.
    nh = new ros::NodeHandle;
        it = new image_transport::ImageTransport(*nh);
        smach_publisher = nh->advertise<std_msgs::String>("alghago_sm/transition",5);
    chatter_publisher = nh->advertise<std_msgs::String>("chatter", 1000);
    badukpan_subscriber = it->subscribe("/badukpan/image_raw", 1,
                                       &QNode::badukpan_image_callback, this);
    smach_subscriber = nh->subscribe("/alghago_sm/smach/container_status", 1,
                                    &QNode::state_callback, this);
    start();
}
void QNode::run() {
    ros::spin();
    /*
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
    */
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}



void QNode::state_callback(const smach_msgs::SmachContainerStatusConstPtr &msg)
{
    smach_state = msg->active_states[0];

    Q_EMIT updateState();
}

void QNode::badukpan_image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    badukpan_image = cv_ptr->image;
    Q_EMIT updateImage();

}



}  // namespace alghago_gui
