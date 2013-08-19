#include <flyvslam/ptam_data.h>

//Constructor sets the number of previous frames to store.
ptam_data::ptam_data()
{
	
}


//Extract the current position and yaw from the ptam message and update the velocity. 
void ptam_data::update(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{

}
//eof
