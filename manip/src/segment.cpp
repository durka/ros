#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
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
            prev_.create(480, 640, CV_8UC3);
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
            cv::Mat prev = cv_ptr->image.clone();
            cv_ptr->image -= prev_; // subtractive filtering
            prev_ = prev;

            /* convert back to ROS */
            image_out_.publish(cv_ptr->toImageMsg());
        }

    private:
        ros::NodeHandle nh_;
        it::ImageTransport it_;
        it::Publisher image_out_;
        it::Subscriber image_in_;
        cv::Mat prev_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "manip_segment");

    Segmenter seg;
    ros::spin();

    return 0;
}

