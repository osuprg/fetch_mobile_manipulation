
 // Include the ROS library
 #include <ros/ros.h>

 // Include pcl
 #include <pcl_conversions/pcl_conversions.h>
 #include <pcl/point_cloud.h>
 #include <pcl/point_types.h>
 #include <pcl/filters/voxel_grid.h>
 #include <pcl/filters/passthrough.h>
 #include <pcl/filters/conditional_removal.h>


 // Include PointCloud2 message
 #include <sensor_msgs/PointCloud2.h>

 // Topics
 static const std::string IMAGE_TOPIC = "/head_camera/depth_registered/points";
 static const std::string PUBLISH_TOPIC = "/pcl/points";

 // ROS Publisher
 static ros::Publisher pub;

 void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
 {
     // Container for original & filtered data
//     pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
//     //pcl::PCLPointCloud2 cloud_filtered;

//     pcl_conversions::toPCL(*cloud_msg, *cloud);

     pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
     pcl::fromROSMsg(*cloud_msg, *rgb_cloud);
     pcl::PCLPointCloud2 cloud_filtered_ros;

     // Perform voxel downsampling
      const float leaf_size_uniform = 0.01f;
      pcl::VoxelGrid<pcl::PointXYZRGB> downsampler;
      downsampler.setInputCloud (rgb_cloud);
      downsampler.setLeafSize (leaf_size_uniform, leaf_size_uniform, leaf_size_uniform);
      downsampler.filter (*rgb_cloud);

     // Convert to PCL data typek

     pcl::PassThrough<pcl::PointXYZRGB> pass;
     pass.setInputCloud (rgb_cloud);
     pass.setFilterFieldName ("y");
     pass.setFilterLimits (-0.3, 2);
     pass.filter (*rgb_cloud);
     //std::cout<<static_cast<unsigned int>(rgb_cloud->points[0].r)<<std::endl;

     pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

     pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
         red_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT, 80));
     pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
     color_cond->addComparison (red_condition);
//     //range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GT, 0.0)));
//        // Build the filter
    color_filter.setInputCloud(rgb_cloud);
     color_filter.setCondition (color_cond);
    color_filter.filter(*rgb_cloud);

     pcl::toPCLPointCloud2(*rgb_cloud, cloud_filtered_ros);

     cloud_filtered_ros.header.frame_id = "/head_camera_rgb_optical_frame";
     pcl_conversions::toPCL(ros::Time::now(), cloud_filtered_ros.header.stamp);
     pub.publish (cloud_filtered_ros);
 }

 int main (int argc, char** argv)
 {
     // Initialize the ROS Node "roscpp_pcl_example"
     ros::init (argc, argv, "roscpp_pcl_example");
     ros::NodeHandle nh;

     // Print "Hello" message with node name to the terminal and ROS log file
     //ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

     // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
     auto sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

     // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
     pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);

     // Spin
     ros::spin();

     // Success
     return 0;
 }
