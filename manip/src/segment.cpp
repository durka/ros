#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_view_plus/Click.h>
#include <geometry_msgs/PoseArray.h>

namespace enc = sensor_msgs::image_encodings;
namespace it = image_transport;
namespace geom = geometry_msgs;

class Segmenter
{
    public:
        Segmenter() : it_(nh_)
        {
            image_out_ = it_.advertise("image_out", 1);
            image_in_ = it_.subscribe("image_in", 1, &Segmenter::kick, this);
            click_sub_ = nh_.subscribe("click", 10, &Segmenter::click, this);
            obj_pub_ = nh_.advertise<geom::PoseArray>("objects", 100);
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
            geom::PoseArray obj_msg;
            geom::Pose pose;

            if (started_)
            {
                const double THRESH = 40;
                output = cv_ptr->image.clone();
                cv::Mat r, g, b,
                        rl, gl, bl,
                        rh, gh, bh,
                        mask = cv::Mat::zeros(last_image_.rows, last_image_.cols, CV_8UC1);
                std::vector<cv::Mat> channels(3);
                channels[0] = b; channels[1] = g; channels[2] = r;
                cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8, 8), cv::Point(4, 4));
                output = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC3);
                for (int i = 0; i < 3; ++i)
                {
                    // mask out color
                    cv::split(cv_ptr->image, channels);
                    r = channels[2]; g = channels[1]; b = channels[0];
                    cv::threshold(r, rl, color_[i][2] - THRESH, 1, CV_THRESH_BINARY);
                    cv::threshold(g, gl, color_[i][1] - THRESH, 1, CV_THRESH_BINARY);
                    cv::threshold(b, bl, color_[i][0] - THRESH, 1, CV_THRESH_BINARY);
                    cv::threshold(r, rh, color_[i][2] + THRESH, 1, CV_THRESH_BINARY_INV);
                    cv::threshold(g, gh, color_[i][1] + THRESH, 1, CV_THRESH_BINARY_INV);
                    cv::threshold(b, bh, color_[i][0] + THRESH, 1, CV_THRESH_BINARY_INV);

                    mask = rl.mul(gl).mul(bl).mul(rh).mul(gh).mul(bh) * 255;
                    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
                    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);

                    // find contours
                    std::vector<std::vector<cv::Point> > contours;
                    std::vector<cv::Vec4i> hierarchy;
                    cv::findContours(mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
                    std::vector<cv::RotatedRect> ellipses(contours.size());
                    int maxj = -1, max = 0;
                    for (int j = 0; j < contours.size(); ++j)
                    {
                        if (contours[j].size() > 5)
                        {
                            ellipses[j] = cv::fitEllipse(cv::Mat(contours[j]));
                            if (ellipses[j].size.width > max)
                            {
                                maxj = j;
                                max = ellipses[j].size.width;
                            }
                            if (ellipses[j].size.height > max)
                            {
                                maxj = j;
                                max = ellipses[j].size.height;
                            }
                        }
                    }
                    cv::drawContours(output, contours, maxj, color_[i], CV_FILLED);
                    if (maxj != -1)
                    {
                        pose.position.x = ellipses[maxj].center.x;
                        pose.position.y = ellipses[maxj].center.y;
                        pose.position.z = 0;
                        pose.orientation.x = 0;
                        pose.orientation.y = 0;
                        pose.orientation.z = sin(ellipses[maxj].angle/2);
                        pose.orientation.w = cos(ellipses[maxj].angle/2);
                    }
                    else
                    {
                        pose.position.x = 0;
                        pose.position.y = 0;
                        pose.position.z = 0;
                        pose.orientation.x = 0;
                        pose.orientation.y = 0;
                        pose.orientation.z = 0;
                        pose.orientation.w = 0; // invalid quaternion means no data
                    }
                    obj_msg.poses.push_back(pose);
                }
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

                obj_pub_.publish(obj_msg);
            }

            last_image_ = cv_ptr->image.clone();
        }

        void click(const image_view_plus::ClickConstPtr& msg)
        {
            // sample from a rectangle around the point
            ROS_INFO("click received at (%d, %d)", msg->x, msg->y);
            if (!started_)
            {
                cv::Mat rect(last_image_, cv::Rect(msg->x - 20, msg->y - 20, 41, 41));
                color_[learned_] = cv::sum(rect)/(41.0*41.0);
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
        ros::Publisher obj_pub_;
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

