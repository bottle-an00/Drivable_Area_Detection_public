#include "data_struction.h"
#include "../road_detector/data_structures.hpp"

class Drivable_Ground{
private:
    ros::NodeHandle nh;
    ros::Subscriber subGroundCloud;
    
    ros::Publisher pubRoadCloud;// 도로에 해당하는 point cloud publisher
    ros::Publisher pubCurbCloud;// 도로 이외의 지면(도보, 들판 등등에.. 해당하는 point cloud publisher)
    ros::Publisher pubRoiRoadCloud; // 도로에 해당하는 point cloud 중 차량의 최소회전 반경을  고려한 point cloud

    pcl::PointCloud<PointType>::Ptr groundCloudIn;// patchwork++에 의해 들어온 ground cloud를 저장

    pcl::PointCloud<PointType>::Ptr RoadCloud;//도로에 해당하는 point cloud
    pcl::PointCloud<PointType>::Ptr CurbCloud;// 도로 이외의 영역에 해당하는 cloud

    pcl::PointCloud<PointType>::Ptr ROIRoadCloud;//도로에 해당하는 point cloud 중 최소 회전 반경을 고려

    pcl::VoxelGrid<PointType> groundDownSampler;
    vector<PointType> cone_center_point;
    vector<pcl::PointCloud<PointType>::Ptr> ground_cloud_channel_vec;
    visualization_msgs::MarkerArray cone_markerarray;    

    std_msgs::Header cloudHeader;
    ERP42 erp42_info;
    Detector road_detector;
    int max_value =0;

public:

    Drivable_Ground():
    nh("~"){

        subGroundCloud = nh.subscribe<sensor_msgs::PointCloud2>("/ground_segmentation/ground", 1, &Drivable_Ground::cloudHandler, this);

        pubRoadCloud = nh.advertise<sensor_msgs::PointCloud2> ("/Road_Cloud", 1);

        pubCurbCloud = nh.advertise<sensor_msgs::PointCloud2> ("/Non_Road_Cloud", 1);

        pubRoiRoadCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ROI_Road_cloud", 1);
        
        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){

        groundCloudIn.reset(new pcl::PointCloud<PointType>());
        
        ground_cloud_channel_vec.resize(32);
        for(int i=0; i<32; i++){
            pcl::PointCloud<PointType>::Ptr tmpPC(new pcl::PointCloud<PointType>);
            ground_cloud_channel_vec[i] = tmpPC;
        }

        RoadCloud.reset(new pcl::PointCloud<PointType>());

        CurbCloud.reset(new pcl::PointCloud<PointType>());

        ROIRoadCloud.reset(new pcl::PointCloud<PointType>());

        groundDownSampler.setLeafSize(1.0f, 2.f, 2.f); // 2.0, 1.0, 0.5, 0.2 순으로 설정해보자
    }

    void resetParameters(){
        groundCloudIn->clear();

        for(int i=0; i<32; i++){
            pcl::PointCloud<PointType>::Ptr tmpPC(new pcl::PointCloud<PointType>);
            ground_cloud_channel_vec[i] = tmpPC;
        }

        RoadCloud->clear();

        CurbCloud->clear();

        ROIRoadCloud->clear();

        max_value =0;
    }

    ~Drivable_Ground(){}

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;
        cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line

        pcl::PointCloud<PointType>::Ptr tmpgroundCloudIn(new pcl::PointCloud<PointType>);
        pcl::fromROSMsg(*laserCloudMsg, *tmpgroundCloudIn);

        size_t cloud_size = tmpgroundCloudIn->points.size();
        for (int i =0 ; i<cloud_size; i++){
            
            PointType thisPoint;    

            thisPoint.x = tmpgroundCloudIn->points[i].x;
            thisPoint.y = tmpgroundCloudIn->points[i].y;
            thisPoint.z = tmpgroundCloudIn->points[i].z;

            float verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            int rowIdn = static_cast<int>((verticalAngle + 30.67) / (41.33/float(31)));    
            
            thisPoint.intensity = rowIdn;
            if(max_value < rowIdn) max_value = rowIdn;

            if(abs(atan2(thisPoint.y, thisPoint.x)) < 90*PI/180) ground_cloud_channel_vec[rowIdn]->push_back(thisPoint);
        } 

    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        copyPointCloud(laserCloudMsg);

        //road_detector.filtered(groundCloudIn,RoadCloud, CurbCloud);
        //*RoadCloud = *groundCloudIn;

        //downsampling()과 set_ROI()의 순서는 유지해줘야함
        downSampling(ground_cloud_channel_vec);

        set_ROI(groundCloudIn);
        
        publishCloud();

        resetParameters();
    }

    void set_ROI(const pcl::PointCloud<PointType>::Ptr cloud ){//roi 설정 
        PointType thisPoint;
        size_t cloud_size = cloud->points.size();
        size_t rowIdn;

        for(int i =0; i< cloud_size; i++){
            thisPoint = cloud->points[i];
            
            // double dx = thisPoint.x;
            // double dy = abs(thisPoint.y)-erp42_info.minimum_steering_radius;
            // double dist = sqrt(dx*dx + dy*dy);
            
            if(abs(thisPoint.y) < 10 && (thisPoint.x) > 0 ){
                ROIRoadCloud->push_back(thisPoint);
            }
        }


    }

    void downSampling(vector<pcl::PointCloud<PointType>::Ptr> cloud_vec){

        for(int j =1; j<max_value; j++){
            pcl::PointCloud<PointType>::Ptr cloud = cloud_vec[j];
            *RoadCloud+=*cloud; 
            //각 채널의 상황에 따라 몇등분할지를 결정 -> voxel size를 결정

            float x_box,y_box;
            PointType maxPoint,minPoint;
            pcl::getMinMax3D(*cloud, minPoint, maxPoint);

            x_box = maxPoint.x - minPoint.x - 0.001;
            y_box = (maxPoint.y - minPoint.y) - 0.001;

            groundDownSampler.setLeafSize(x_box, y_box/10, 0.5f);
            groundDownSampler.setInputCloud(cloud);
            groundDownSampler.filter(*cloud);
            *CurbCloud+=*cloud; 

            size_t cloud_size = cloud->points.size();

            // if(cloud_size > 1){
            //     float radius_sum = 0.0;
            //     float z_sum = 0.0;
            //     PointType tmp_point;

            //     for(int i =0; i<cloud_size; i++){
                
            //         float radius = sqrt(cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y);
            //         float z_value = cloud->points[i].z;

            //         radius_sum += radius;
            //         z_sum += z_value;
            //     }

            //     tmp_point.x = radius_sum/cloud_size;
            //     tmp_point.z = z_sum/cloud_size;
            //     tmp_point.intensity = j;

            //     cloud->push_back(tmp_point);

                *groundCloudIn += *cloud;
            //}

        }
    }

    // void downSampling(vector<pcl::PointCloud<PointType>::Ptr> cloud_vec){

        // for(int j =1; j<max_value; j++){
            // pcl::PointCloud<PointType>::Ptr cloud = cloud_vec[j];
            // *RoadCloud+=*cloud; 
            // //각 채널의 상황에 따라 몇등분할지를 결정 -> voxel size를 결정
            // size_t cloud_size = cloud->points.size();
            
            // groundDownSampler.setInputCloud(cloud);
            // groundDownSampler.filter(*cloud);
            // *CurbCloud+=*cloud; 

            // if(cloud_size > 1){
                // for(int i =0; i<cloud_size-1; i++){
                
                    // PointType tmp_point;
                    // tmp_point.x = (cloud->points[i].x + cloud->points[i+1].x)/2;
                    // tmp_point.y = (cloud->points[i].y + cloud->points[i+1].y)/2;
                    // tmp_point.z = (cloud->points[i].z + cloud->points[i+1].z)/2;

                    // tmp_point.intensity = cloud->points[i].intensity;

                    // cloud->push_back(tmp_point);
                // }

                // *groundCloudIn += *cloud;
            // }
        // }
    // }
    
    void publishCloud(){
        sensor_msgs::PointCloud2 laserCloudTemp;

        if (pubRoiRoadCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*ROIRoadCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id =cloudHeader.frame_id;
            pubRoiRoadCloud.publish(laserCloudTemp);
        }

        if (pubRoadCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*RoadCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id =cloudHeader.frame_id;
            pubRoadCloud.publish(laserCloudTemp);
        }

        if (pubCurbCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*CurbCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id =cloudHeader.frame_id;
            pubCurbCloud.publish(laserCloudTemp);
        }

    
    }
};
