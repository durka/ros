#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_view_plus/Click.h>

namespace enc = sensor_msgs::image_encodings;
namespace it = image_transport;

class Segmenter
{
    public:
        Segmenter() : it_(nh_)
        {
            image_out_ = it_.advertise("image_out", 1);
            image_in_ = it_.subscribe("image_in", 1, &Segmenter::kick, this);
            click_sub_ = nh_.subscribe("click", 10, &Segmenter::click, this);
            started_ = false;
            learned_ = 0;
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
            cv::Mat output;

            if (started_)
            {
                output = cv_ptr->image.clone();
            }

            /* convert back to ROS */
            if (!started_)
            {
                image_out_.publish(cv_ptr->toImageMsg());
            }
            else
            {
                cv_bridge::CvImage out_msg;
                out_msg.header   = msg->header;
                out_msg.encoding = enc::BGR8;
                out_msg.image    = output;

                image_out_.publish(out_msg.toImageMsg());
            }

            last_image_ = cv_ptr->image.clone();
        }

        void click(const image_view_plus::ClickConstPtr& msg)
        {
            // sample from a 21x21 rectangle around the point
            ROS_INFO("click received at (%d, %d)", msg->x, msg->y);
            if (!started_)
            {
                ROS_INFO("rect is (%d, %d) - (%d, %d)", msg->x - 10, msg->y - 10, msg->x + 10, msg->y + 10);
                //cv::Mat mask(last_image_.rows, last_image_.cols, CV_8UC3, 0);
                //cv::rectangle(mask, cv::Point(msg->x - 10, msg->y - 10), cv::Point(msg->x + 10, msg->y + 10), CV_RGB(1, 1, 1), CV_FILLED);
                cv::Mat rect(last_image_, cv::Rect(msg->x - 10, msg->y - 10, 21, 21));
                //color_[learned_] = cv::mean(last_image_, mask);
                color_[learned_] = cv::mean(rect);
                ++learned_;

                if (learned_ == 3)
                {
                    started_ = true;
                    ROS_INFO("Learning complete! Tracking from now on.");
                    ROS_INFO("\tColors are:");
                    for (int i = 0; i < 3; ++i)
                    {
                        ROS_INFO("\t\t(%g, %g, %g)", color_[i].val[2], color_[i].val[1], color_[i].val[0]);
                    }
                }
            }
        }

    private:
        ros::NodeHandle nh_;
        it::ImageTransport it_;
        it::Publisher image_out_;
        it::Subscriber image_in_;
        ros::Subscriber click_sub_;
        cv::Mat last_image_;
        bool started_;
        int learned_;
        cv::Scalar color_[3];
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "manip_segment");
    cv::initModule_nonfree();

    Segmenter seg;
    ros::spin();

    return 0;
}

