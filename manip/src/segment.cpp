#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;
namespace it = image_transport;

class Segmenter
{
    public:
        Segmenter() : it_(nh_)
        {
            image_out_ = it_.advertise("image_out", 1);
            image_in_ = it_.subscribe("image_in", 1, &Segmenter::kick, this);
	    sifter_ = cv::FeatureDetector::create("SIFT");
        }

        void kick(const sensor_msgs::ImageConstPtr& msg)
        {
            /* convert from ROS to OpenCV */
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge derped: %s", e.what());
                return;
            }
            
            /* process image */
	    std::vector<cv::KeyPoint> keypoints;
	    sifter_->detect(cv_ptr->image, keypoints);
	    cv::drawKeypoints(cv_ptr->image, keypoints, cv_ptr->image);

            /* convert back to ROS */
            image_out_.publish(cv_ptr->toImageMsg());
        }

    private:
        ros::NodeHandle nh_;
        it::ImageTransport it_;
        it::Publisher image_out_;
        it::Subscriber image_in_;
	cv::Ptr<cv::FeatureDetector> sifter_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "manip_segment");
    cv::initModule_nonfree();

    Segmenter seg;
    ros::spin();

    return 0;
}

