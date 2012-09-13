#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <manip/transform2d.h>

namespace geom = geometry_msgs;

class Learner
{
    public:
        Learner()
        {
            obj_sub_ = nh_.subscribe("objects", 100, &Learner::kick, this);
        }

        void kick(const geom::PoseArrayConstPtr& msg)
        {
        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber obj_sub_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "manip_learn");

    Learner student;
    ros::spin();

    return 0;
}

