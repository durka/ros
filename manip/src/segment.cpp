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
            static bool first = true;
            static cv::Mat old_image;
            static std::vector<cv::KeyPoint> old_keypoints;
            static cv::Mat old_descriptors;
            static cv::Mat output;
            static std::map<int, std::vector<cv::Point> > trajectories;
            static std::map<int, int> indices;

            std::vector<cv::KeyPoint> new_keypoints;
            cv::Mat new_descriptors;
            sift_(cv_ptr->image, cv::Mat(), new_keypoints, new_descriptors);
            if (first)
            {
                first = false;
                cv::drawKeypoints(cv_ptr->image, new_keypoints, cv_ptr->image);

                // populate trajectories
                for (unsigned i = 0; i < new_keypoints.size(); ++i)
                {
                    trajectories[i].push_back(new_keypoints[i].pt);
                    indices[i] = i;
                }
            }
            else
            {
                cv::BFMatcher matcher(2);
                std::vector<cv::DMatch> matches;
                matcher.match(old_descriptors, new_descriptors, matches);

                // continue trajectories
                std::map<int, int> old_indices = indices;
                std::sort(matches.begin(), matches.end());
                for (unsigned i = 0; i < matches.size(); ++i)
                {
                    indices[matches[i].queryIdx] = old_indices[matches[i].trainIdx];
                    if (i < 50)
                    {
                        trajectories[indices[matches[i].trainIdx]].push_back(new_keypoints[matches[i].queryIdx].pt);
                    }
                }
                int i = indices[matches[0].trainIdx];
                ROS_INFO("[%3d][%3d](%5d %5d)", 
                          i,
                          trajectories[i].size(),
                          trajectories[i].back().x,
                          trajectories[i].back().y);

                std::vector<cv::DMatch> good_matches(50);
                std::copy(matches.begin(), matches.begin() + 50, good_matches.begin());
                cv::drawMatches(old_image, old_keypoints, cv_ptr->image, new_keypoints, good_matches, output);
            }
            old_image = cv_ptr->image.clone();
            old_keypoints = new_keypoints;
            old_descriptors = new_descriptors.clone();

            /* convert back to ROS */
            if (first)
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
        }

    private:
        ros::NodeHandle nh_;
        it::ImageTransport it_;
        it::Publisher image_out_;
        it::Subscriber image_in_;
        cv::SURF sift_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "manip_segment");
    cv::initModule_nonfree();

    Segmenter seg;
    ros::spin();

    return 0;
}

