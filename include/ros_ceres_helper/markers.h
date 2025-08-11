#ifndef CERES_HELPER_MARKERS_H
#define CERES_HELPER_MARKERS_H

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace cerise {
    struct Marker: public visualization_msgs::msg::Marker {
        Marker() : visualization_msgs::msg::Marker() {
            initialize();
        }
        Marker(const Marker & m) : visualization_msgs::msg::Marker(m) {}
        Marker(const visualization_msgs::msg::Marker & m) : visualization_msgs::msg::Marker(m) {}

        void initialize();

        void setHeader(const std::string & frame, const rclcpp::Time & t = rclcpp::Time());
        void setNameId(const std::string & ns, size_t id);
        void setType(int type);
        void setLifeTime(double duration);
        void setPosition(double x, double y, double z);
        void setRotation(double x, double y, double z, double w);
        void setRotation(const geometry_msgs::msg::Quaternion & q);
        void setScale(double x, double y, double z);
        void setColor(double r, double g, double b, double a=1.0);
        void pushPoint(double x, double y, double z);
        void pushColor(double r, double g, double b, double a=1.0);

        void createLineStrip(double w);
        void createSphereList(double r);
        void createSphere(double x, double y, double z, double r);
        void createEllipse(double x, double y, double z, 
                double rx, double ry, double rz);
        void createCubeList(double r);
        void createCube(double x, double y, double z, double r);
        void createBox(double x, double y, double z, 
                double rx, double ry, double rz);

    };

}




#endif // CERES_HELPER_MARKERS_H
