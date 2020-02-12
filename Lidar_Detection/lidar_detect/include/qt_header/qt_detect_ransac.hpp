#ifndef __QT_DETECT_RANSAC__
#define __QT_DETECT_RANSAC__

#include "qt_header/qt_detect_node.hpp"
#include "qt_header/find_road_points.hpp"
/* RANSAC code */
/*
    *estimate Road Plane
    *Using RANSAC algorithm 
*/

class Plane
{
    public:
        Plane() 
            : cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>)
            , private_nh("~")
            , QT(new QuadTree())
            , matrix_size(17)
            , check_width(1)
            , ok(false)
        {   
            if(!private_nh.getParam("/quadtree_params/QTsub_topic", subtopic)) throw std::runtime_error("set subtopic");
            sub = nh.subscribe(subtopic.c_str(), 10000, &Plane::callback, this);
            sub_filtered_points_raw = nh.subscribe("/my_points_raw",10000, &Plane::Callback_filtered_points_raw, this);
            pub_plane_points = nh.advertise<visualization_msgs::Marker>("/plane_points", 10);
            pub4 = nh.advertise<sensor_msgs::PointCloud2>("/projected_cloud", 10000);
        }
        void Callback_filtered_points_raw(const sensor_msgs::PointCloud2ConstPtr& ptr);
        void print_Ransac_plane();
        void callback(const sensor_msgs::PointCloud2::ConstPtr &ptr);
        void make_plane_RANSAC();
        void extract_normal_vector();
        void projection_onto_plane();
        
    private:
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Publisher pub;
        ros::Publisher pub4;
        ros::Publisher pub_plane_points;

        ros::Subscriber sub;
        ros::Subscriber sub_filtered_points_raw;
  
        QuadTree* QT;
        geometry_msgs::Point normal_vector; 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, filtered_points_raw;
        pcl::PointCloud<pcl::PointXYZ> scan, filtered_points_raw_scan; 
        pcl::PointCloud<pcl::PointXYZI> filterd_scan, pcl_scan;
        std::string subtopic;
        queue< geometry_msgs::Point > normal_vector_queue; 
        queue< float > D_queue;

        vector< geometry_msgs::Point > plane_point_vec;           

        int matrix_size;
        int check_width; 
        float D; 
        bool ok;
};

#endif