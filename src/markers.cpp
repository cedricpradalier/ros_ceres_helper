
#include <ros_ceres_helper/markers.h>

namespace cerise {

    void Marker::initialize() {
        header.frame_id = "";
        header.stamp = ros::Time();
        ns = "";
        id = 0;
        type = visualization_msgs::Marker::SPHERE;
        action = visualization_msgs::Marker::ADD;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        scale.x = 0;
        scale.y = 0;
        scale.z = 0;
        color.a = 1.0; // Don't forget to set the alpha!
        color.r = 0.0;
        color.g = 0.0;
        color.b = 0.0;
        lifetime = ros::Duration();
        frame_locked = false;
        points.clear();
        colors.clear();
        text.clear();
        //only if using a MESH_RESOURCE marker type:
        // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        mesh_resource.clear();
        mesh_use_embedded_materials = false;
    }

    void Marker::setNameId(const std::string & name, size_t i) {
        ns = name;
        id = i;
    }

    void Marker::setHeader(const std::string & frame, const ros::Time & t) {
        header.frame_id = frame;
        header.stamp = t;
    }

    void Marker::setType(int t) {
        type = t;
    }

    void Marker::setLifeTime(double duration) {
        lifetime = ros::Duration(duration);
    }

    void Marker::setPosition(double x, double y, double z) {
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
    }

    void Marker::setRotation(double x, double y, double z, double w) {
        pose.orientation.x = x;
        pose.orientation.y = y;
        pose.orientation.z = z;
        pose.orientation.w = w;
    }

    void Marker::setRotation(const geometry_msgs::Quaternion & q) {
        pose.orientation = q;
    }

    void Marker::setScale(double x, double y, double z) {
        scale.x = x;
        scale.y = y;
        scale.z = z;
    }

    void Marker::setColor(double r, double g, double b, double a) {
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
    }

    void Marker::pushPoint(double x, double y, double z) {
        geometry_msgs::Point P;
        P.x = x;
        P.y = y;
        P.z = z;
        points.push_back(P);
    }

    void Marker::pushColor(double r, double g, double b, double a) {
        std_msgs::ColorRGBA c;
        c.r = r;
        c.g = g;
        c.b = b;
        c.a = a;
        colors.push_back(c);
    }

    void Marker::createLineStrip(double w) {
        initialize();
        type = visualization_msgs::Marker::LINE_STRIP;
        setScale(w,0,0);
    }

    void Marker::createSphereList(double r) {
        initialize();
        type = visualization_msgs::Marker::SPHERE_LIST;
        setScale(r,r,r);
    }

    void Marker::createSphere(double x, double y, double z, double r) {
        initialize();
        type = visualization_msgs::Marker::SPHERE;
        setPosition(x,y,z);
        setScale(r,r,r);
    }

    void Marker::createEllipse(double x, double y, double z, 
            double rx, double ry, double rz) {
        initialize();
        type = visualization_msgs::Marker::SPHERE;
        setPosition(x,y,z);
        setScale(rx,ry,rz);
    }

    void Marker::createCubeList(double r) {
        initialize();
        type = visualization_msgs::Marker::CUBE_LIST;
        setScale(r,r,r);
    }

    void Marker::createCube(double x, double y, double z, double r) {
        initialize();
        type = visualization_msgs::Marker::CUBE;
        setPosition(x,y,z);
        setScale(r,r,r);
    }

    void Marker::createBox(double x, double y, double z, 
            double rx, double ry, double rz) {
        initialize();
        type = visualization_msgs::Marker::CUBE;
        setPosition(x,y,z);
        setScale(rx,ry,rz);
    }



}
