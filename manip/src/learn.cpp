#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <manip/transform2d.h>

namespace geom = geometry_msgs;

class Learner
{
    public:
        Learner()
        {
            obj_sub_ = nh_.subscribe("objects", 100, &Learner::kick, this);
            command_sub_ = nh_.subscribe("command", 100, &Learner::command, this);
        }

        void kick(const geom::PoseArrayConstPtr& msg)
        {
            // for now, just collect the trajectories (and throw away orientation)
            for (int i = 0; i < msg->poses.size(); ++i)
            {
                if (msg->poses[i].orientation.x != 0 // check for valid data
                 || msg->poses[i].orientation.y != 0
                 || msg->poses[i].orientation.z != 0
                 || msg->poses[i].orientation.w != 0)
                {
                    if (trajectories_.size() < i+1)
                    {
                        trajectories_.push_back(std::vector<geom::Point>());
                    }
                    trajectories_[i].push_back(msg->poses[i].position);
                }
            }
        }

        void command(const std_msgs::StringConstPtr& msg)
        {
            if (msg->data == "help")
            {
                ROS_INFO("command list:");
                ROS_INFO("\thelp\t\tprint command list");
                ROS_INFO("\tgo\t\trun learner on recorded trajectories");
                ROS_INFO("\tshow\t\tdisplay recorded object trajectories");
                ROS_INFO("\ttell\t\tdisplay learned model");
            }
            else if (msg->data == "go")
            {
                ROS_INFO("not implemented");
            }
            else if (msg->data == "show")
            {
                ROS_INFO("dumping trajectories");
                for (int i = 0; i < trajectories_.size(); ++i)
                {
                    ROS_INFO("\tobject %d", i);
                    for (int j = 0; j < trajectories_[i].size(); ++j)
                    {
                        ROS_INFO("\t\t(%g, %g, %g)", trajectories_[i][j].x, trajectories_[i][j].y, trajectories_[i][j].z);
                    }
                }
            }
            else if (msg->data == "tell")
            {
                ROS_INFO("not implemented");
            }
            else
            {
                ROS_WARN("invalid command");
            }
        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber obj_sub_;
        ros::Subscriber command_sub_;
        std::vector<std::vector<geom::Point> > trajectories_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "manip_learn");

    Learner student;
    ros::spin();

    return 0;
}

