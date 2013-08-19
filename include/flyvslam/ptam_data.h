#ifndef _PTAMDATA_H
#define _PTAMDATA_H

#include <TooN/TooN.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ros/ros.h"

///Class to store the PTAM data received.
///This class stores n previous PTAM data frames and provides a differential velocity.
class ptam_data
{
	public:
		//Empty constructor.
		ptam_data();

		//Update the PTAM info with a new ptam frame.
		void update(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
};
#endif
//eof
