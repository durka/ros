#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>

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
                if (trajectories_.size() < i+1)
                {
                    ROS_INFO("now tracking object %d...", i);
                    trajectories_.push_back(std::vector<geom::Point>());
                }
                if (msg->poses[i].orientation.x != 0 // check for valid data
                 || msg->poses[i].orientation.y != 0
                 || msg->poses[i].orientation.z != 0
                 || msg->poses[i].orientation.w != 0)
                {
                    trajectories_[i].push_back(msg->poses[i].position);
                }
                else
                {
                    trajectories_[i].push_back(trajectories_[i].back());
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
                ROS_INFO("\treset\t\tforget trajectories");
            }
            else if (msg->data == "go")
            {
                // implementing "Interactive Segmentation, Tracking and Kinematic Modeling of Unknown Articulated Objects", Katz et al, 2012
                
                /* check for joints between all combinations of objects */
                for (int i = 0; i < trajectories_.size(); ++i)
                {
                    for (int j = i+1; j < trajectories_.size(); ++j)
                    {
                        std::vector<geom::Point> traj_i = trajectories_[i],
                                                 traj_j = trajectories_[j];

                        /* trajectory relative to first object */
                        for (int k = 0; k < traj_j.size(); ++k)
                        {
                            traj_j[k].x -= traj_i[k].x;
                            traj_j[k].y -= traj_i[k].y;
                            traj_j[k].z -= traj_i[k].z;
                        }

                        /* rigid?
                         *
                         * find maximum distance as percentage of initial distance
                         * if it's within 10%, rigid
                         * if not, keep going
                         * note: I made this up, not from paper
                         */
                        double initial_dist = sqrt(pow(traj_j[0].x, 2)
                                                 + pow(traj_j[0].y, 2)
                                                 + pow(traj_j[0].z, 2));
                        double max_dist = initial_dist;
                        double min_dist = initial_dist;

                        for (int k = 1; k < traj_i.size(); ++k)
                        {
                            double dist = sqrt(pow(traj_j[k].x, 2)
                                             + pow(traj_j[k].y, 2)
                                             + pow(traj_j[k].z, 2));
                            
                            if (dist > max_dist)
                            {
                                max_dist = dist;
                            }
                            if (dist < min_dist)
                            {
                                min_dist = dist;
                            }
                        }

                        if (max_dist/initial_dist < 1.1 && min_dist/initial_dist > 0.9)
                        {
                            ROS_INFO("DETECTED JOINT %d-%d AS RIGID", i, j);
                            continue;
                        }

                        /* prismatic?
                         *
                         * fit a line to the second body's movement
                         * if the fit is good, prismatic
                         * if not, keep going
                         * note: in the paper, they have many features and many line fits
                         */


                        /* revolute?
                         *
                         * fit a circle to the second body's movement
                         * if the fit is good, revolute
                         * if not, disconnected
                         * note: in the paper, they have many features and many circle fits
                         */
                    }
                }
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
            else if (msg->data == "reset")
            {
                trajectories_.clear();
                ROS_INFO("EVERYTHING IS FORGOTTEN");
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

