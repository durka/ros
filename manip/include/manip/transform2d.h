#ifndef _TRANSFORM2D_H_
#define _TRANSFORM2D_H_

#include <opencv2/opencv.hpp>

class Transform2D // scale, then rotate, then translate
{
    public:
        float scale, tx, ty, theta;

        Transform2D()
            : scale(1), tx(0), ty(0), theta(0) {}
        Transform2D(float scale_, float tx_, float ty_, float theta_)
            : scale(scale_), tx(tx_), ty(ty_), theta(theta_) {}
        Transform2D inv() const
        {
            return Transform2D(1/scale,
                               -(tx*cos(theta) + ty*sin(theta))/scale,
                               -(ty*cos(theta) - tx*sin(theta))/scale,
                               -theta);
        }
        void apply(float& x, float& y, float& th) const
        {
            cv::Mat_<float> M = (cv::Mat)*this;
            cv::Mat_<float> p(3, 1);

            p(0) = x; p(1) = y; p(2) = 1;
            p = M*p;
            x = p(0); y = p(1);

            th = angle_add(th, theta);
        }

        Transform2D operator*(const Transform2D& rhs) const
        {
            return Transform2D(scale * rhs.scale,
                               tx + scale*rhs.tx*cos(theta) - scale*rhs.ty*sin(theta),
                               ty + scale*rhs.ty*cos(theta) + scale*rhs.tx*sin(theta),
                               angle_add(theta, rhs.theta));
        }
        operator cv::Mat() const
        {
            cv::Mat_<float> M(3, 3);

            M(0,0) = scale * cos(theta);    M(0,1) = scale * -sin(theta);   M(0,2) = tx;
            M(1,0) = scale * sin(theta);    M(1,1) = scale * cos(theta);    M(1,2) = ty;
            M(2,0) = 0;                     M(2,1) = 0;                     M(2,2) = 1;

            return M;
        }

        static double angle_add(double a, double b)
        {
            double result = a + b;
            
            if (result > M_PI)
            {
                result -= 2*M_PI;
            }
            else if (result < -M_PI)
            {
                result += 2*M_PI;
            }

            return result;
        }
};

#endif

