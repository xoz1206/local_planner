#include "qt_header/qt_detect_ransac.hpp"

/**/
void Plane::Callback_filtered_points_raw(const sensor_msgs::PointCloud2ConstPtr& ptr)
{
    pcl::fromROSMsg(*ptr, filtered_points_raw_scan);
    ok = true;
}

void Plane::callback(const sensor_msgs::PointCloud2ConstPtr& ptr)
{
    if(ok == true)
    {
        sensor_msgs::PointCloud2 point_msg, filtered_msg, pcl_msg;
        
        pcl::fromROSMsg(*ptr, scan);
        pcl::fromROSMsg(*ptr, filterd_scan);
        pcl::fromROSMsg(*ptr, pcl_scan);
        filterd_scan.clear();
        pcl_scan.clear();
        // ROS_INFO("points raw : %ld", scan.points.size());
        pcl::VoxelGrid<pcl::PointXYZ> vg;

        vg.setInputCloud(scan.makeShared());//scan PointCloud data copy
        vg.setLeafSize(0.15f,0.15f,0.15f);//set the voxel grid size //10cm
        vg.filter(*cloud_filtered);//create the filtering object
        // ROS_INFO("voxel points : %ld", cloud_filtered->points.size());
        make_plane_RANSAC();
        projection_onto_plane();
        //print_Ransac_plane();

        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(filterd_scan));
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_scan));
        pcl::toROSMsg(*scan_ptr, filtered_msg);
        pcl::toROSMsg(*pcl_ptr, pcl_msg);

        QT->pointCallBack(&filtered_msg, &pcl_msg);
    }
    else
    {
        ROS_INFO("we need to get /my_points_raw's filtered_point_data !");
    }
}
void Plane::make_plane_RANSAC()
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    pcl::PointCloud<pcl::PointXYZ> filtered_points_cloud_z;
            
    // z 값이 filtering 된 point들을 가지고 pointcloud 만드는 작업. RANSAC 알고리즘에 넣어주기 위해
    for(size_t k = 0; k < cloud_filtered->points.size(); ++k)
    {
        if(fabs(cloud_filtered->points[k].x) < 10 && fabs(cloud_filtered->points[k].y) < 10 && cloud_filtered->points[k].z < -1.5)
        {
            pcl::PointXYZ z_filtered_point;
            z_filtered_point.x = cloud_filtered->points[k].x;
            z_filtered_point.y = cloud_filtered->points[k].y;
            z_filtered_point.z = cloud_filtered->points[k].z;
            filtered_points_cloud_z.push_back(z_filtered_point);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_points_cloud_z));
    seg.setInputCloud (point_ptr);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }
    normal_vector.x = coefficients->values[0];
    normal_vector.y = coefficients->values[1];
    normal_vector.z = coefficients->values[2];

    D = (-1)*coefficients->values[3];
    normal_vector_queue.push(normal_vector);
    D_queue.push(D);
    extract_normal_vector(); // normal_vector의 n개의 평균을 구한다.
}

void Plane::projection_onto_plane()
{
    Eigen::Vector4f coeffs;
    coeffs << normal_vector.x, normal_vector.y, normal_vector.z, -D;
            
    int count = 0;
    for(size_t i = 0; i < filtered_points_raw_scan.points.size(); ++i)
    {
        // projection이 수행되어야 하는 영역안의 points 추출 후, projection, projection data = /my_points_raw
        if( fabs(filtered_points_raw_scan.points[i].x) < x_limit &&
            fabs(filtered_points_raw_scan.points[i].y) < y_limit &&
            filtered_points_raw_scan.points[i].z < z_high_limit)
        {
            float z = normal_vector.x * filtered_points_raw_scan.points[i].x + normal_vector.y * filtered_points_raw_scan.points[i].y * normal_vector.z * filtered_points_raw_scan.points[i].z - D;
                    
            if( z < 0 ) continue;

            pcl::PointXYZI projection, point;

            projection.x = filtered_points_raw_scan.points[i].x;  
            projection.y =filtered_points_raw_scan.points[i].y;
            projection.z = (-1) * (normal_vector.x * filtered_points_raw_scan.points[i].x + normal_vector.y * filtered_points_raw_scan.points[i].y - D) / normal_vector.z;
                    
            //======//
            if(filtered_points_raw_scan.points[i].z > -1.5){
                point.x = filtered_points_raw_scan.points[i].x;  
                point.y = filtered_points_raw_scan.points[i].y;
                point.z = filtered_points_raw_scan.points[i].z;
            }
            //======//

            filterd_scan.points.emplace_back(point);
            projection.intensity = 2.0;
            pcl_scan.points.emplace_back(projection);
        }
    }
    filterd_scan.width = static_cast<uint32_t>(filterd_scan.points.size());
    filterd_scan.height = 1;
    sensor_msgs::PointCloud2 projected_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr n_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_scan));
    pcl::toROSMsg(*n_ptr, projected_cloud);

    projected_cloud.header.frame_id = "velodyne";
    pub4.publish(projected_cloud);

            // ROS_INFO("filtered points : %ld", filterd_scan.points.size());
}

void Plane::print_Ransac_plane()
{
    for(float x = -10.0; x < 10.0; x += 0.5)
    {
        for(float y = -10.0; y < 10.0; y += 0.5)
        {
            geometry_msgs::Point plane_point;
            plane_point.x = x;
            plane_point.y = y;
            plane_point.z = (-1) * (normal_vector.x * x + normal_vector.y * y - D) / normal_vector.z;
            plane_point_vec.push_back(plane_point);
        }
    }

    visualization_msgs::Marker points;
    std_msgs::ColorRGBA c;

    int id = 0;
    c.r = 0.0;
    c.g = 0.0;
    c.b = 1.0;
    c.a = 1.0;

    for(size_t k = 0; k < plane_point_vec.size(); ++k)
    {
        points.header.frame_id = "velodyne";
        points.header.stamp = ros::Time::now();
        points.ns = "points";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.lifetime = ros::Duration(0.1);
        points.id = id++;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.3;
        points.scale.y = 0.3;

        points.color.r = 0.0;
        points.color.g = 0.0;
        points.color.b = 1.0;
        points.color.a = 1.0;

        points.points.push_back(plane_point_vec[k]);
        points.colors.push_back(c);
    }
            
    pub_plane_points.publish(points);
    points.points.clear();
    points.colors.clear();
}

void Plane::extract_normal_vector()
{
    if(normal_vector_queue.size() == normal_vector_queue_size && D_queue.size() == normal_vector_queue_size)
    {
        float sum_x = 0.0;
        float sum_y = 0.0;
        float sum_z = 0.0;
        float sum_d = 0.0;

        for(int k = 0; k<normal_vector_queue.size(); ++k)
        {
            sum_x += normal_vector_queue.front().x;
            sum_y += normal_vector_queue.front().y;
            sum_z += normal_vector_queue.front().z;
            sum_d += D_queue.front();
        }
        //다시 갱신.
        normal_vector.x = sum_x / normal_vector_queue.size();
        normal_vector.y = sum_y / normal_vector_queue.size();
        normal_vector.z = sum_z / normal_vector_queue.size();
        D = sum_d /D_queue.size();
        
        //최신의 4개의 data point를 가지고 평균을 내기때문
        normal_vector_queue.pop(); //맨 앞 원소 제거
        D_queue.pop();
    }
}
