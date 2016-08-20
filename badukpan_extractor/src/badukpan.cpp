#include "badukpan_extractor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "badukpan_extractor");
    ros::NodeHandle nh;

    BadukpanExtractor baduk(nh);

    ros::spin();

	return 0;
}
