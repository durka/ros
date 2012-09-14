#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <dlib/optimization.h>

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

                        /* trajectory relative to first object, but no negatives */
                        double minx = 0, miny = 0, minz = 0;
                        for (int k = 0; k < traj_j.size(); ++k)
                        {
                            traj_j[k].x -= traj_i[k].x;
                            if (traj_j[k].x < minx) minx = traj_j[k].x;

                            traj_j[k].y -= traj_i[k].y;
                            if (traj_j[k].y < miny) miny = traj_j[k].y;

                            traj_j[k].z -= traj_i[k].z;
                            if (traj_j[k].z < minz) minz = traj_j[k].z;
                        }
                        for (int k = 0; k < traj_j.size(); ++k)
                        {
                            traj_j[k].x -= minx;
                            traj_j[k].y -= miny;
                            traj_j[k].z -= minz;
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
                        /*std::vector<std::pair<dlib::matrix<double,2,1>, double> > data; // (x,y)=>z
                        dlib::matrix<double,6,1> params;
                        for (int k = 0; k < traj_j.size(); ++k)
                        {
                            dlib::matrix<double,2,1> input;
                            input(0) = traj_j[i].x;
                            input(1) = traj_j[i].y;
                            data.push_back(std::make_pair(input, traj_j[i].z));
                        }
                        params = 1;
                        dlib::solve_least_squares(dlib::objective_delta_stop_strategy(1e-7).be_verbose(),
                                                  &Learner::line_residual,
                                                  dlib::derivative(&Learner::line_residual),
                                                  data,
                                                  params);*/
                        
                        // so we are just going to fit a 2D line
                        double sum_xy = 0, sum_x = 0, sum_y = 0, sum_x2 = 0;
                        for (int k = 0; k < traj_j.size(); ++k)
                        {
                            sum_xy += traj_j[k].x * traj_j[k].y;
                            sum_x += traj_j[k].x;
                            sum_y += traj_j[k].y;
                            sum_x2 += pow(traj_j[k].x, 2);
                        }
                        double slope = (sum_xy - sum_x*sum_y/traj_j.size())/(sum_x2 - pow(sum_x, 2)/traj_j.size());
                        double intercept = sum_y/traj_j.size() - slope*sum_x/traj_j.size();
                        double error = 0;
                        for (int k = 0; k < traj_j.size(); ++k)
                        {
                            error += pow(traj_j[k].x*slope + intercept - traj_j[k].y, 2);
                        }

                        if (error < 1000)
                        {
                            ROS_INFO("DETECTED JOINT %d-%d AS PRISMATIC", i, j);
                            continue;
                        }

                        /* revolute?
                         *
                         * fit a circle to the second body's movement
                         * if the fit is good, revolute
                         * if not, disconnected
                         * note: in the paper, they have many features and many circle fits
                         */
                        // note: 2D hack
                        double center_x = 0, center_y = 0;
                        for (int k = 0; k < traj_j.size(); ++k)
                        {
                            center_x += traj_j[k].x;
                            center_y += traj_j[k].y;
                        }
                        center_x /= traj_j.size();
                        center_y /= traj_j.size();
                        double radius = 0;
                        for (int k = 0; k < traj_j.size(); ++k)
                        {
                            radius += pow(traj_j[k].x - center_x, 2)
                                    + pow(traj_j[k].y - center_y, 2);
                        }
                        radius = sqrt(radius / traj_j.size());

                        error = 0;
                        for (int k = 0; k < traj_j.size(); ++k)
                        {
                            error += pow(  sqrt(  pow(traj_j[k].x - center_x, 2)
                                                + pow(traj_j[k].y - center_y, 2))
                                         - radius, 2);
                        }

                        if (error < 50000)
                        {
                            ROS_INFO("DETECTED JOINT %d-%d AS REVOLUTE", i, j);
                            continue;
                        }


                        ROS_INFO("JOINT %d-%d IS NOT CONNECTED (err=%g)", i, j, error);
                    }
                }
                ROS_INFO("DONE");
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

        static double line_residual(const std::pair<dlib::matrix<double,2,1>, double>& data,
                             const dlib::matrix<double,6,1>& params)
        {
            /* params is {x0, y0, z0, a, b, c}
             * where the line equation is (x-x0)/a = (y-y0)/b = (z-z0)/c
             * this means we have two equations to minimize:
             *      z = c/a(x-x0) + z0
             *      z = b/a(y-y0) + z0
             */

            double x0 = params(0),
                   y0 = params(1),
                   z0 = params(2),
                   a = params(3),
                   b = params(4),
                   c = params(5),

                   x = data.first(0),
                   y = data.first(1),
                   z = data.second;

            return pow(b/a*(x-x0)+y0 - y, 2) + pow(b/c*(z-z0)+y0 - y, 2);
        }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "manip_learn");

    Learner student;
    ros::spin();

    return 0;
}

