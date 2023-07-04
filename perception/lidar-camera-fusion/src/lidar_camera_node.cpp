#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <math.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <armadillo>

#include <chrono> 

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

//Publisher


ros::Publisher pcOnimg_pub;
ros::Publisher pcOnimg_pub_blue;
ros::Publisher pc_pub;
ros::Publisher id_pub;


float maxlen =100.0; // 라이다의 최대 거리
float minlen = 0.01; // 라이다의 최소 거리
float max_FOV = 3.0; // 카메라의 최대 시야각 (라디안 단위)
float min_FOV = 0.4; // 카메라의 최소 시야각 (라디안 단위)

/// 포인트 클라우드를 이미지로 변환하기 위한 매개변수
float angular_resolution_x =0.5f; // x 방향 각도 해상도
float angular_resolution_y = 2.1f; // y 방향 각도 해상도
float max_angle_width= 360.0f; // 이미지의 최대 너비 각도
float max_angle_height = 180.0f; // 이미지의 최대 높이 각도
float z_max = 100.0f; // 클라우드 포인트의 최대 z 좌표값
float z_min = 100.0f; // 클라우드 포인트의 최소 z 좌표값

float max_depth =100.0; // 이미지에서의 최대 깊이
float min_depth = 8.0; // 이미지에서의 최소 깊이

float interpol_value = 20.0; // 보간법에 사용되는 값

// input topics 
std::string imgTopic = "/right_image";
std::string imgTopic_blue = "/left_image";
std::string pcTopic = "/velodyne_points";
std::string bboxTopic = "/bounding_boxes";

//matrix calibration lidar and camera

Eigen::MatrixXf Tlc(3,1); // translation matrix lidar-camera
Eigen::MatrixXf Rlc(3,3); // rotation matrix lidar-camera
Eigen::MatrixXf Mc(3,4);  // camera calibration matrix

Eigen::MatrixXf Tlc_blue(3,1); // translation matrix lidar-camera
Eigen::MatrixXf Rlc_blue(3,3); // rotation matrix lidar-camera
Eigen::MatrixXf Mc_blue(3,4);  // camera calibration matrix

// range image parametros
boost::shared_ptr<pcl::RangeImageSpherical> rangeImage;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;


///////////////////////////////////////callback

void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& in_pc2 , const ImageConstPtr& in_image, const ImageConstPtr& in_image_blue, const geometry_msgs::PoseArray::ConstPtr& msg)
{
  //ROS_INFO("Received Image and PointCloud message.");
    cv_bridge::CvImagePtr cv_ptr , color_pcl, cv_ptr_blue, color_pcl_blue;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(in_image, sensor_msgs::image_encodings::BGR8);
          color_pcl = cv_bridge::toCvCopy(in_image, sensor_msgs::image_encodings::BGR8);
          cv_ptr_blue = cv_bridge::toCvCopy(in_image_blue, sensor_msgs::image_encodings::BGR8);
          color_pcl_blue = cv_bridge::toCvCopy(in_image_blue, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

  //Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud<T>
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*in_pc2,pcl_pc2);
  PointCloud::Ptr msg_pointCloud(new PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2,*msg_pointCloud);
  ///

  ////// filter point cloud 
  if (msg_pointCloud == NULL) return;

  PointCloud::Ptr cloud_in (new PointCloud);
  //PointCloud::Ptr cloud_filter (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);

  //PointCloud::Ptr cloud_aux (new PointCloud);
 // pcl::PointXYZI point_aux;

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*msg_pointCloud, *cloud_in, indices);
  
  for (int i = 0; i < (int) cloud_in->points.size(); i++)
  {
      double distance = sqrt(cloud_in->points[i].x * cloud_in->points[i].x + cloud_in->points[i].y * cloud_in->points[i].y);     
      if(distance<minlen || distance>maxlen)
       continue;        
      cloud_out->push_back(cloud_in->points[i]);     
    
  }  


  //                                                  point cloud to image 

  //============================================================================================================
  //============================================================================================================

  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  rangeImage->pcl::RangeImage::createFromPointCloud(*cloud_out, pcl::deg2rad(angular_resolution_x), pcl::deg2rad(angular_resolution_y),
                                       pcl::deg2rad(max_angle_width), pcl::deg2rad(max_angle_height),
                                       sensorPose, coordinate_frame, 0.0f, 0.0f, 0);

  

  int cols_img = rangeImage->width;
  int rows_img = rangeImage->height;


  arma::mat Z;  // interpolation de la imagen
  arma::mat Zz; // interpolation de las alturas de la imagen

  Z.zeros(rows_img,cols_img);         
  Zz.zeros(rows_img,cols_img);       

  Eigen::MatrixXf ZZei (rows_img,cols_img);
 
  for (int i=0; i< cols_img; ++i)
      for (int j=0; j<rows_img ; ++j)
      {
        float r =  rangeImage->getPoint(i, j).range;     
        float zz = rangeImage->getPoint(i, j).z; 
       
       // Eigen::Vector3f tmp_point;
        //rangeImage->calculate3DPoint (float(i), float(j), r, tmp_point);
        if(std::isinf(r) || r<minlen || r>maxlen || std::isnan(zz)){
            continue;
        }             
        Z.at(j,i) = r;   
        Zz.at(j,i) = zz;
        //ZZei(j,i)=tmp_point[2];


        //point_aux.x = tmp_point[0];
        //point_aux.y = tmp_point[1];
        //point_aux.z = tmp_point[2];
      
       // cloud_aux->push_back(point_aux);



        //std::cout<<"i: "<<i<<" Z.getpoint: "<<zz<<" tmpPoint: "<<tmp_point<<std::endl;
       
      }

  ////////////////////////////////////////////// interpolation
  //============================================================================================================
  
  arma::vec X = arma::regspace(1, Z.n_cols);  // X = horizontal spacing
  arma::vec Y = arma::regspace(1, Z.n_rows);  // Y = vertical spacing 

  

  arma::vec XI = arma:: regspace(X.min(), 1.0, X.max()); // magnify by approx 2
  arma::vec YI = arma::regspace(Y.min(), 1.0/interpol_value, Y.max()); // 


  arma::mat ZI_near;  
  arma::mat ZI;
  arma::mat ZzI;

  arma::interp2(X, Y, Z, XI, YI, ZI,"lineal");  
  arma::interp2(X, Y, Zz, XI, YI, ZzI,"lineal");  

  //===========================================fin filtrado por imagen=================================================
  /////////////////////////////

  // reconstruccion de imagen a nube 3D
  //============================================================================================================
  

  PointCloud::Ptr point_cloud (new PointCloud);
  PointCloud::Ptr cloud (new PointCloud);
  point_cloud->width = ZI.n_cols; 
  point_cloud->height = ZI.n_rows;
  point_cloud->is_dense = false;
  point_cloud->points.resize (point_cloud->width * point_cloud->height);

  arma::mat Zout = ZI;
  
  
  //////////////////filtrado de elementos interpolados con el fondo
  for (uint i=0; i< ZI.n_rows; i+=1)
   {       
      for (uint j=0; j<ZI.n_cols ; j+=1)
      {             
       if((ZI(i,j)== 0 ))
       {
        if(i+interpol_value<ZI.n_rows)
          for (int k=1; k<= interpol_value; k+=1) 
            Zout(i+k,j)=0;
        if(i>interpol_value)
          for (int k=1; k<= interpol_value; k+=1) 
            Zout(i-k,j)=0;
        }
      }      
    }

///////// Range 이미지에서 포인트 클라우드로 변환
  int num_pc = 0; 
  for (uint i=0; i< ZI.n_rows - interpol_value; i+=1)
   {       
      for (uint j=0; j<ZI.n_cols ; j+=1)
      {

        float ang = M_PI-((2.0 * M_PI * j )/(ZI.n_cols));

        if (ang < min_FOV-M_PI/2.0|| ang > max_FOV - M_PI/2.0) 
          continue;

        if(!(Zout(i,j)== 0 ))
        {  
          float pc_modulo = Zout(i,j);
          float pc_x = sqrt(pow(pc_modulo,2)- pow(ZzI(i,j),2)) * cos(ang);
          float pc_y = sqrt(pow(pc_modulo,2)- pow(ZzI(i,j),2)) * sin(ang);

          float ang_x_lidar = 0.6*M_PI/180.0;  

          Eigen::MatrixXf Lidar_matrix(3,3); //matrix  transformation between lidar and range image. It rotates the angles that it has of error with respect to the ground
          Eigen::MatrixXf result(3,1);
          Lidar_matrix <<   cos(ang_x_lidar) ,0                ,sin(ang_x_lidar),
                            0                ,1                ,0,
                            -sin(ang_x_lidar),0                ,cos(ang_x_lidar) ;


          result << pc_x,
                    pc_y,
                    ZzI(i,j);
          
          result = Lidar_matrix*result;  // rotacion en eje X para correccion

          point_cloud->points[num_pc].x = result(0);
          point_cloud->points[num_pc].y = result(1);
          point_cloud->points[num_pc].z = result(2);

          cloud->push_back(point_cloud->points[num_pc]); 

          num_pc++;
        }
      }
   }  

  //============================================================================================================

   PointCloud::Ptr P_out (new PointCloud);
 
   //filremove noise of point cloud
  /*pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50.0);
  sor.setStddevMulThresh (1.0);
  sor.filter (*P_out);*/

  // dowsmapling
  /*pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*P_out);*/


  P_out = cloud;


  Eigen::MatrixXf RTlc(4,4); // translation matrix lidar-camera
  RTlc<<   Rlc(0), Rlc(3) , Rlc(6) ,Tlc(0)
          ,Rlc(1), Rlc(4) , Rlc(7) ,Tlc(1)
          ,Rlc(2), Rlc(5) , Rlc(8) ,Tlc(2)
          ,0       , 0        , 0  , 1    ;

  Eigen::MatrixXf RTlc_blue(4,4); // translation matrix lidar-camera
  RTlc_blue<<   Rlc_blue(0), Rlc_blue(3) , Rlc_blue(6) ,Tlc_blue(0)
          ,Rlc_blue(1), Rlc_blue(4) , Rlc_blue(7) ,Tlc_blue(1)
          ,Rlc_blue(2), Rlc_blue(5) , Rlc_blue(8) ,Tlc_blue(2)
          ,0       , 0        , 0  , 1    ;

  //std::cout<<RTlc<<std::endl;

  int size_inter_Lidar = (int) P_out->points.size();

  Eigen::MatrixXf Lidar_camera(3,size_inter_Lidar);
  Eigen::MatrixXf Lidar_cam(3,1);
  Eigen::MatrixXf Lidar_cam_blue(3,1);
  Eigen::MatrixXf pc_matrix(4,1);
  Eigen::MatrixXf pointCloud_matrix(4,size_inter_Lidar);

  unsigned int cols = in_image->width;
  unsigned int rows = in_image->height;

  uint px_data = 0; uint py_data = 0;


  pcl::PointXYZRGB point;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_color (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr id_color (new pcl::PointCloud<pcl::PointXYZRGB>);

  //std::cout << "Length of poses array: " << msg->poses.size() << std::endl;
  int msg_len = msg->poses.size();
  int bbox_xmax[msg->poses.size()-1];
  int bbox_xmin[msg->poses.size()-1];
  int bbox_ymax[msg->poses.size()-1];
  int bbox_ymin[msg->poses.size()-1];
  //int xrange[msg->poses.size()-1];
  //int yrange[msg->poses.size()-1]; 

  for (int j = 0; j < msg_len; j++)
  {
    if(msg->poses[j].orientation.z < 640)
    {
      bbox_xmax[j] =  msg->poses[j].orientation.z;
      bbox_xmin[j] = msg->poses[j].orientation.x;
      bbox_ymax[j] =  msg->poses[j].orientation.w;
      bbox_ymin[j] =  msg->poses[j].orientation.y;
      //std::cout <<" z index : " << msg->poses[j].orientation.z << " x  index : " << msg->poses[j].orientation.x << std::endl;
      //std::cout <<" x max: " << msg->poses[j].orientation.z/1000 << " x min: " << msg->poses[j].orientation.x/1000 << std::endl;
      //std::cout << "y max: " <<  fmod(msg->poses[j].orientation.z,1000.0) << " y min: " << fmod(msg->poses[j].orientation.x,1000.0) << std::endl;
      
      id_color->points.clear();
      for (int i = 0; i < size_inter_Lidar; i++)
      {
        pc_matrix(0,0) = -P_out->points[i].y;   
        pc_matrix(1,0) = -P_out->points[i].z;   
        pc_matrix(2,0) =  P_out->points[i].x;  
        pc_matrix(3,0) = 1.0;

        Lidar_cam = Mc * (RTlc * pc_matrix);

        px_data = (int)(Lidar_cam(0,0)/Lidar_cam(2,0));
        py_data = (int)(Lidar_cam(1,0)/Lidar_cam(2,0));
        //std::cout << "px_data " <<  px_data << " py_data " << py_data  << std::endl;
      
        //if(px_data<0.0 || px_data>=cols || py_data<0.0 || py_data>=rows)
          //continue;
        if(px_data < (unsigned int)bbox_xmin[j] || px_data >= (unsigned int)bbox_xmax[j] || py_data < (unsigned int)bbox_ymin[j] || py_data >= (unsigned int)bbox_ymax[j])
          continue;
        
        int color_dis_x = (int)(255*((P_out->points[i].x)/maxlen));
        int color_dis_z = (int)(255*((P_out->points[i].x)/10.0));
        if(color_dis_z>255)
            color_dis_z = 255;


        //point cloud con color
        cv::Vec3b & color = color_pcl->image.at<cv::Vec3b>(py_data,px_data);

        point.x = P_out->points[i].x;
        point.y = P_out->points[i].y;
        point.z = P_out->points[i].z;
        

        point.r = (int)color[2]; 
        point.g = (int)color[1]; 
        point.b = (int)color[0];

        
        pc_color->points.push_back(point);   
        
        cv::circle(cv_ptr->image, cv::Point(px_data, py_data), 1, CV_RGB(255-color_dis_x,(int)(color_dis_z),color_dis_x),cv::FILLED);
        //std::cout <<" b" << j << std::endl;
        id_color->header.frame_id = (msg->poses[j].position.x == 0) ? "b" : "y" ;
        id_color->header.frame_id += to_string(j);
        id_color->header.stamp = pcl_pc2.header.stamp;
        id_color->points.push_back(point);
      }
      if(!(id_color->points.empty()))
        id_pub.publish (id_color);
   }

  if( msg->poses[j].orientation.z > 640 )
  {
    //std::cout <<" x max: " << msg->poses[j].orientation.z/1000 << " x min: " << msg->poses[j].orientation.x/1000 << std::endl;
    //std::cout << "y max: " <<  fmod(msg->poses[j].orientation.z,1000.0) << " y min: " << fmod(msg->poses[j].orientation.x,1000.0) << std::endl;
      
    bbox_xmax[j] = msg->poses[j].orientation.z - 640 ;

    if (msg->poses[j].orientation.x < 640)
      bbox_xmin[j] = 0;
    else
      bbox_xmin[j] = msg->poses[j].orientation.x - 640;

    bbox_ymax[j] = msg->poses[j].orientation.w;
    bbox_ymin[j] = msg->poses[j].orientation.y;

    //std::cout <<" x2 max: " << (unsigned int)bbox_xmin[j] << " x2 min: " << (unsigned int)bbox_xmax[j] << std::endl;

    //xrange[j] = bbox_xmax[j] - bbox_xmin[j];
    //yrange[j] = bbox_ymax[j] - bbox_ymin[j];
    id_color->points.clear();
    for (int i = 0; i < size_inter_Lidar; i++)
    {
        
      pc_matrix(0,0) = -P_out->points[i].y;   
      pc_matrix(1,0) = -P_out->points[i].z;   
      pc_matrix(2,0) =  P_out->points[i].x;  
      
      pc_matrix(3,0) = 1.0;
      Lidar_cam = Mc_blue * (RTlc_blue * pc_matrix);
      px_data = (int)(Lidar_cam(0,0)/Lidar_cam(2,0));
      py_data = (int)(Lidar_cam(1,0)/Lidar_cam(2,0));
      
      //std::cout << "px_data " <<  px_data << " x2 max: " << (unsigned int)bbox_xmin[j] << " x2 min: " << (unsigned int)bbox_xmax[j] <<  " py_data " << py_data <<" y2 min: " <<  (unsigned int)bbox_ymin[j] << " y2 max: " << (unsigned int)bbox_ymax[j]  << std::endl;
      //if(px_data<0.0 || px_data>=cols || py_data<0.0 || py_data>=rows)
        //continue;
      
      if(px_data < (unsigned int)bbox_xmin[j] || px_data >= (unsigned int)bbox_xmax[j] || py_data < (unsigned int)bbox_ymin[j] || py_data >= (unsigned int)bbox_ymax[j])
        continue;

      // if(msg->bounding_boxes[j].xmax<640)
      //     continue;
      
      int color_dis_x = (int)(255*((P_out->points[i].x)/maxlen));
      int color_dis_z = (int)(255*((P_out->points[i].x)/10.0));

      if(color_dis_z>255)
        color_dis_z = 255;
      //point cloud con color
      cv::Vec3b & color = color_pcl_blue->image.at<cv::Vec3b>(py_data,px_data);
      //if (P_out->points[i].z < -1.2)
      //    continue;
      point.x = P_out->points[i].x;
      point.y = P_out->points[i].y;
      point.z = P_out->points[i].z;
            
      // cout << "min_z:" << point.z << endl;
      // cout << "i :" << i << endl;
      // cout << "j :" << j << endl;
      point.r = (int)color[2]; 
      point.g = (int)color[1]; 
      point.b = (int)color[0];
      
      pc_color->points.push_back(point);   
      cv::circle(cv_ptr_blue->image, cv::Point(px_data, py_data), 1, CV_RGB(255-color_dis_x,(int)(color_dis_z),color_dis_x),cv::FILLED);
      //to sorting code
      
      //std::cout << msg->bounding_boxes[j].Class << "["<<j <<"]"<< std::endl;
      //cout << "point:" << point << endl;
      // cout << msg->bounding_boxes.size() << endl;
      //std::cout <<" y " << j << std::endl;

      id_color->header.frame_id = (msg->poses[j].position.x == 0) ? "b" : "y";
      id_color->header.frame_id += to_string(j);
      id_color->header.stamp = pcl_pc2.header.stamp;
      id_color->points.push_back(point);
      // id_pub.publish (id_color);
    }

      if(!(id_color->points.empty()))
        id_pub.publish (id_color);
    }

  }


  pc_color->is_dense = true;
  pc_color->width = (int) pc_color->points.size();
  pc_color->height = 1;
  pc_color->header.frame_id = "velodyne";

  pcOnimg_pub.publish(cv_ptr->toImageMsg());
  pcOnimg_pub_blue.publish(cv_ptr_blue->toImageMsg());

  pc_pub.publish (pc_color);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pontCloudOntImage");
  ros::NodeHandle nh;  
  
  /// Load Parameters

  nh.getParam("/maxlen", maxlen);
  nh.getParam("/minlen", minlen);
  nh.getParam("/max_ang_FOV", max_FOV);
  nh.getParam("/min_ang_FOV", min_FOV);
  nh.getParam("/pcTopic", pcTopic);
  nh.getParam("/imgTopic", imgTopic);
  nh.getParam("/imgTopic_blue",imgTopic_blue);

  nh.getParam("/bbox_Topic", bboxTopic);

  nh.getParam("/x_resolution", angular_resolution_x);
  nh.getParam("/y_interpolation", interpol_value);

  nh.getParam("/ang_Y_resolution", angular_resolution_y);
  

  XmlRpc::XmlRpcValue param;

  nh.getParam("/matrix_file/tlc", param);
  Tlc <<  (double)param[0]
         ,(double)param[1]
         ,(double)param[2];

  nh.getParam("/matrix_file/rlc", param);


  Rlc <<  (double)param[0] ,(double)param[1] ,(double)param[2]
         ,(double)param[3] ,(double)param[4] ,(double)param[5]
         ,(double)param[6] ,(double)param[7] ,(double)param[8];

  nh.getParam("/matrix_file/camera_matrix", param);

  Mc  <<  (double)param[0] ,(double)param[1] ,(double)param[2] ,(double)param[3]
         ,(double)param[4] ,(double)param[5] ,(double)param[6] ,(double)param[7]
         ,(double)param[8] ,(double)param[9] ,(double)param[10],(double)param[11];

  nh.getParam("/matrix_file_blue/tlc", param);
  Tlc_blue <<  (double)param[0]
              ,(double)param[1]
              ,(double)param[2];

  nh.getParam("/matrix_file_blue/rlc", param);

  Rlc_blue <<  (double)param[0] ,(double)param[1] ,(double)param[2]
         ,(double)param[3] ,(double)param[4] ,(double)param[5]
         ,(double)param[6] ,(double)param[7] ,(double)param[8];

  nh.getParam("/matrix_file_blue/camera_matrix", param);

  Mc_blue  <<  (double)param[0] ,(double)param[1] ,(double)param[2] ,(double)param[3]
         ,(double)param[4] ,(double)param[5] ,(double)param[6] ,(double)param[7]
         ,(double)param[8] ,(double)param[9] ,(double)param[10],(double)param[11];

  message_filters::Subscriber<PointCloud2> pc_sub(nh, pcTopic , 1);
  message_filters::Subscriber<Image> img_sub(nh, imgTopic, 1);
  message_filters::Subscriber<Image> img_sub_blue(nh, imgTopic_blue, 1);
  message_filters::Subscriber<geometry_msgs::PoseArray> bbox_sub(nh, bboxTopic, 1);
    
  typedef sync_policies::ApproximateTime<PointCloud2, Image, Image, geometry_msgs::PoseArray> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), pc_sub, img_sub, img_sub_blue, bbox_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
  
  pcOnimg_pub = nh.advertise<sensor_msgs::Image>("/pcOnImage_image", 1);
  pcOnimg_pub_blue = nh.advertise<sensor_msgs::Image>("/pcOnImage_image_blue", 1);

  rangeImage = boost::shared_ptr<pcl::RangeImageSpherical>(new pcl::RangeImageSpherical);

  id_pub = nh.advertise<PointCloud> ("/id_points2", 1);
  pc_pub = nh.advertise<PointCloud> ("/points2", 1);   

  ros::spin();
}
