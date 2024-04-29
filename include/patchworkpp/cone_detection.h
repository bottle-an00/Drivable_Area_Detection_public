#include "data_struction.h"
#include "fixedQueue.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

class Cone_Detection{
private:
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::PointCloud2> subNonGroundCloud;
    message_filters::Subscriber<sensor_msgs::PointCloud2> subGroundCloud;
    ros::Subscriber subLocalMsg;
    boost::shared_ptr<Sync> sync;
    
    ros::Publisher pubConeCloud; // 도로에 해당하는 point cloud 중 차량의 최소회전 반경을  고려한 point cloud
    ros::Publisher pubConesCenter;
    ros::Publisher pubConesBoundary;
    ros::Publisher pubROICloud;
    ros::Publisher pubVector;
    ros::Publisher pub_prev_cone;
    ros::Publisher pubConesCenterKF;

    pcl::PointCloud<PointType>::Ptr groundCloudIn;// patchwork++에 의해 들어온 ground cloud를 저장
    pcl::PointCloud<PointType>::Ptr nongroundCloudIn;// patchwork++에 의해 들어온  non ground cloud를 저장

    vector<pcl::PointCloud<PointType>::Ptr> ConeCandidateCloud[2];// 콘 후보군 cloud
    pcl::PointCloud<PointType>::Ptr ConeCloud;// 콘에 해당하는 cloud
    pcl::PointCloud<PointType>::Ptr prev_ConeCloud;// 콘에 해당하는 cloud
    FixedSizeQueue<pcl::PointCloud<PointType>::Ptr>prev_ConeCloud_que;// 콘에 해당하는 cloud
    pcl::PointCloud<PointType>::Ptr Clustered_Cloud;// 클러스터링된 cloud
    pcl::PointCloud<PointType>::Ptr ROICloud;// 클러스터링된 cloud중 z값이 0 이하인 점들만 남김

    visualization_msgs::MarkerArray cone_boundary_markerarray;    
    visualization_msgs::MarkerArray cone_center_markerarray;    
    visualization_msgs::MarkerArray cone_center_markerarrayKF;    
    visualization_msgs::MarkerArray normal_vectors;    
    vector<Cones> obj_center_point;
    vector<PointType> lastest_cones;

    std_msgs::Header cloudHeader;
    pcl::KdTreeFLANN<PointType> Ground_kdtree;
    
    Ego_status ego_info;

    vector<int> id_list;
    
    int cone_id=0;
    
    float dx, dy, dyaw;
public:

    Cone_Detection():
        nh("~"){

        subNonGroundCloud.subscribe(nh, "/ground_segmentation/nonground", 2);
        subGroundCloud.subscribe(nh, "/ground_segmentation/ground", 2);
        subLocalMsg = nh.subscribe<tf2_msgs::TFMessage>("/tf", 100, &Cone_Detection::localMsgHandler, this);

        sync.reset(new Sync(MySyncPolicy(20), subNonGroundCloud, subGroundCloud));
        sync->registerCallback(boost::bind(&Cone_Detection::cloudHandler, this, _1, _2));

        pubConeCloud = nh.advertise<sensor_msgs::PointCloud2> ("/cone_cloud", 1);
        pubROICloud = nh.advertise<sensor_msgs::PointCloud2> ("/ROI_cloud", 1); 

        pubConesCenter = nh.advertise<visualization_msgs::MarkerArray>("/cone_center_markers", 1);
        pubConesCenterKF = nh.advertise<visualization_msgs::MarkerArray>("/cone_center_markers_kf", 1);
        pubConesBoundary = nh.advertise<visualization_msgs::MarkerArray>("/cone_BoundingBox", 1);
        pubVector =  nh.advertise<visualization_msgs::MarkerArray>("/vectors", 1);

        pub_prev_cone = nh.advertise<sensor_msgs::PointCloud2> ("/prev_conecloud", 1); 

        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){

        groundCloudIn.reset(new pcl::PointCloud<PointType>());
        nongroundCloudIn.reset(new pcl::PointCloud<PointType>());
        ConeCloud.reset(new pcl::PointCloud<PointType>());
        prev_ConeCloud.reset(new pcl::PointCloud<PointType>());
        Clustered_Cloud.reset(new pcl::PointCloud<PointType>());
        ROICloud.reset(new pcl::PointCloud<PointType>());
        
        lastest_cones.resize(10000);

        cone_boundary_markerarray.markers.clear();
        cone_center_markerarray.markers.clear();
        cone_center_markerarrayKF.markers.clear();
        normal_vectors.markers.clear();

    }

    void resetParameters(){
        groundCloudIn->clear();
        nongroundCloudIn->clear();
        
        for(int i=0; i<2; i++)
            ConeCandidateCloud[i].clear();

        ROICloud->clear();
        ConeCloud->clear();
        prev_ConeCloud->clear();

        id_list.clear();
        obj_center_point.clear();
        Clustered_Cloud->clear();
        
        cone_center_markerarray.markers.clear();
        cone_center_markerarrayKF.markers.clear();
        cone_boundary_markerarray.markers.clear();
        normal_vectors.markers.clear();

    }

    ~Cone_Detection(){}
    
    void groundcloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        
        pcl::fromROSMsg(*laserCloudMsg, *groundCloudIn);
        //LiDARGpsSync(groundCloudIn);

        Ground_kdtree.setInputCloud(groundCloudIn);
    }

    void localMsgHandler(const tf2_msgs::TFMessage::ConstPtr& localMsg){
        if(ego_info.is_initialize){
            ego_info.prev = ego_info.curr;
        }
        
        for (auto& transform : localMsg->transforms) {
            if (transform.child_frame_id == "/base_link2") { // Adjust "your_vehicle_frame_id" accordingly
                ego_info.curr.x = transform.transform.translation.x;
                ego_info.curr.y = transform.transform.translation.y;
                
                Eigen::Quaterniond q(transform.transform.rotation.w,
                                  transform.transform.rotation.x,
                                  transform.transform.rotation.y,
                                  transform.transform.rotation.z);

                Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

                double yaw = euler[2];

                ego_info.curr.z = yaw; // testEKF.bag의 경우 차량의 heading값이  -6도 틀어짐
            }

        }

        if(ego_info.is_initialize){
        dx = (-1) * abs(ego_info.curr.x - ego_info.prev.x);
        dy = (-1) * abs(ego_info.curr.y - ego_info.prev.y);
        dyaw = (-1) * abs(ego_info.curr.z - ego_info.prev.z);   

        }

        //cout << "x : " <<  ego_info.curr.x << " y : " << ego_info.curr.y << " heading : "  << ego_info.curr.z << " diff x : " << dx <<" diff y : " << dy << " diff yaw "<< dyaw<< endl << endl; 
    }

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;
        cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *nongroundCloudIn);
        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*nongroundCloudIn, *nongroundCloudIn, indices);
        
        //LiDARGpsSync(nongroundCloudIn);
    }

    void cloudHandler(const boost::shared_ptr<const sensor_msgs::PointCloud2>& laserCloudMsg,
                    const boost::shared_ptr<const sensor_msgs::PointCloud2>& GroundlaserCloudMsg){

        groundcloudHandler(GroundlaserCloudMsg);
        
        auto start_time = std::chrono::high_resolution_clock::now();
        copyPointCloud(laserCloudMsg);

        /*1. Object filtering*/ 
        clustering(nongroundCloudIn,ConeCandidateCloud[0], 1, 2, 1000);
        
        /*2. Set_ROI*/
        set_ROI(ConeCandidateCloud[0],ROICloud);
        //
        /*3. Cone Detection*/
        clustering(ROICloud,ConeCandidateCloud[1], 0.3, 2, 400);
        detect_cones(ConeCandidateCloud[1]);
        
        /*4. match detected Cones with previous Cones*/
        matchCones(obj_center_point);
        id_list = check_curr_id();

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        //cout << "process time :: " << duration.count() << " ms" << endl;

        visual_cones_center();
        visual_cones_boundary();

        publishCloud();
        resetParameters();
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn){

        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;
        PointType pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->resize(cloudSize);
        
        for (int i = 0; i < cloudSize; ++i){

            pointFrom = &cloudIn->points[i];
            float x1 = cos(ego_info.curr.z*PI/180) * pointFrom->x - sin(ego_info.curr.z*PI/180) * pointFrom->y;
            float y1 = sin(ego_info.curr.z*PI/180) * pointFrom->x + cos(ego_info.curr.z*PI/180)* pointFrom->y;
            float z1 = pointFrom->z;

            pointTo.x = x1 + ego_info.curr.x;
            pointTo.y = y1 + ego_info.curr.y;
            pointTo.z = pointFrom->z;
            pointTo.intensity = pointFrom->intensity;

            cloudOut->points[i] = pointTo;
        }
        return cloudOut;
    }

    vector<int> check_curr_id(){
        size_t pc_size = prev_ConeCloud->points.size();
        
        for(int i = 0; i < pc_size; i++){
            int point_id = static_cast<int>(prev_ConeCloud->points[i].intensity);
            
            id_list.push_back(point_id);
        }
        std::unordered_set<int> uniqueIds(id_list.begin(), id_list.end());

        std::vector<int> sortedIds(uniqueIds.begin(), uniqueIds.end());
        std::sort(sortedIds.begin(), sortedIds.end());

        return sortedIds;
    }

    void matchCones(vector<Cones>& cone_info){

        pcl::PointCloud<PointType>::Ptr curr_Cones_loc(new pcl::PointCloud<PointType>);
        gather_prev_frame_pc();
        
        size_t cone_num = cone_info.size();
        for(vector<Cones>::iterator it = cone_info.begin(); it != cone_info.end(); it++){
            PointType thispoint;

            thispoint.x = (*it).mid_point.x;
            thispoint.y = (*it).mid_point.y;
            thispoint.z = 0.0;

            if(ego_info.is_initialize && prev_ConeCloud->points.size() > 0){//이미 초기화가 완료된 상황
                std::vector<int> indices;
                std::vector<float> distances;
                pcl::KdTreeFLANN<PointType> prev_Cones_kdtree;
                
                prev_Cones_kdtree.setInputCloud(prev_ConeCloud);
                prev_Cones_kdtree.nearestKSearch(thispoint, 1 ,indices, distances);
                
                if(distances[0] < 0.5 && distances[0] > 0){
                    //만약 이전의 cone들의 위치를 현재 프레임의 좌표계로 옮겨왔을때 가장 가까운 것이 threshold보다 작다면 matching
                    thispoint.intensity = prev_ConeCloud->points[indices[0]].intensity;
                    lastest_cones[prev_ConeCloud->points[indices[0]].intensity] = thispoint;
                }else{
                    //아니라면 새로운 id 부여
                    thispoint.intensity = static_cast<float>(cone_id);
                    lastest_cones[cone_id] = thispoint;
                    cone_id++;
                }
            }else{//맨 처음 실행하는 상황
                thispoint.intensity = static_cast<float>(cone_id);
                lastest_cones[cone_id] = thispoint;
                cone_id++;
            }
            curr_Cones_loc->push_back(thispoint);
            (*it).id = static_cast<int>(thispoint.intensity); // 현재 detect한 cone들의 id를 옮겨주는 작업

        }

        prev_ConeCloud_que.push(curr_Cones_loc);
        
        ego_info.is_initialize = true;

    }
    
    void gather_prev_frame_pc(){
        deque<pcl::PointCloud<PointType>::Ptr> pc;
        pc = prev_ConeCloud_que.getData();
        
        for(auto& iter : pc){
            /*gps정보를 이용*/
            movePrevCones2curr_coordinate_system(iter);
            *prev_ConeCloud += *(iter);
        }
    }

    void movePrevCones2curr_coordinate_system(pcl::PointCloud<PointType>::Ptr prev_PC){
        
        size_t cone_num = prev_PC->points.size();
        for(int i=0; i<cone_num; i++){

            float G2L_dx = dx*cos((ego_info.curr.z)*PI/180) + dy*cos((90-ego_info.curr.z)*PI/180);

            float x = prev_PC->points[i].x;
            float y = prev_PC->points[i].y;


            prev_PC->points[i].x = x*cos((dyaw)*PI/180) - y*sin((dyaw)*PI/180);
            prev_PC->points[i].y = y*cos((dyaw)*PI/180) + x*sin((dyaw)*PI/180);

            prev_PC->points[i].x =prev_PC->points[i].x + G2L_dx;

        }

    }

    void detect_cones(vector<pcl::PointCloud<PointType>::Ptr> input_cloud_vec){
        int count =0;

        for (auto iter = input_cloud_vec.begin(); iter != input_cloud_vec.end(); ++iter){
            pcl::CentroidPoint<PointType> centroid;

            size_t cloud_size = (*iter)->points.size();
            
            for(int i=0; i< cloud_size; i++){
                PointType thispoint = (*iter)->points[i];
                centroid.add(thispoint);
            }
            //clustered point의 중간값을 계산
            PointType c1;
            centroid.get(c1);

            // kd-tree를 활용해 중심점의 2m 반경 내의 지면 PC를 추출
            std::vector<int> indices;
            std::vector<float> distances;
            pcl::PointCloud<PointType>::Ptr tmp_ground(new pcl::PointCloud<PointType>);

            float ground_min_z = 0.0; //우선 센서 높이를 기본값으로 설정

            if(Ground_kdtree.radiusSearch(c1,2.0,indices, distances) > 0){
                int count = 0;

                for (size_t j = 0; j < indices.size(); ++j) {

                    if(ground_min_z >  groundCloudIn->points[indices[j]].z){
                        ground_min_z = groundCloudIn->points[indices[j]].z;
                    }
                    
                    tmp_ground->push_back(groundCloudIn->points[indices[j]]);
                }

            }
            
            // 지면 일부분 point cloud에서 얻은 법선 벡터와 콘의 주축 벡터를 활용해 각도를 계산
            VectorXf ground_normal(3,1);
            if(tmp_ground->points.size() > 20) ground_normal = conduct_PCA(tmp_ground,2);
            else {
                ground_normal[0] =0;ground_normal[1] =0;ground_normal[2] =1;
            }

            VectorXf cone_principle_vec = conduct_PCA((*iter),0);

            visual_vector(c1,vector2point(cone_principle_vec,c1), 1.0, 1.0, 1.0,count);
            visual_vector(c1,vector2point(ground_normal,c1), 0.0, 0.0,1.0,count+100);
            
            double diff_angle = (acos(ground_normal.dot(cone_principle_vec)) /(magnitude(ground_normal)*magnitude(cone_principle_vec)))*180/PI;
            
            if(diff_angle < 60 ){
               
                //publish할 ConeCloud에 추가
                *ConeCloud += *(*iter);
                //*Clustered_Cloud += *transformPointCloud(*iter);

                //clustered point의 최대 최소값
                PointType maxPoint,minPoint;
                pcl::getMinMax3D(*(*iter), minPoint, maxPoint);
                        
                Cones cone_info;
                cone_info.max_point = maxPoint;
                cone_info.min_point = minPoint;
                cone_info.mid_point = c1;

                obj_center_point.push_back(cone_info); 
            }

            count++;
        }
    }

    void Add_points_keep_vertical_resolution(pcl::PointCloud<PointType>::Ptr candidate_cloud, float ground_min, pcl::CentroidPoint<PointType>& centroid){
        pcl::PointCloud<PointType>::Ptr seed_cloud(new pcl::PointCloud<PointType>);

        PointType maxPoint, minPoint;
        pcl::getMinMax3D(*candidate_cloud,maxPoint, minPoint);

        size_t cloud_size = candidate_cloud->points.size();
        for(int i=0; i < cloud_size; i++){
            PointType thisPoint;
            thisPoint = candidate_cloud->points[i];

            if(thisPoint.z < minPoint.z + 0.05) seed_cloud->push_back(thisPoint);
        }

        size_t seed_num = seed_cloud->points.size();
        if(seed_num > 0){
            PointType new_point;
            
            for (int i =0; i< seed_num; i++){
                new_point = seed_cloud->points[i];
                
                double range = cal_range(new_point);
                double diff_z = range * (1.4*PI/180);

                while(new_point.z - diff_z > ground_min){
                    new_point.z -= diff_z;
                    candidate_cloud->push_back(new_point);
                    centroid.add(new_point);
                }
            }

        }
    }

    double cal_range(PointType point){
        return sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    }
    double cal_diff(PointType point){
        return sqrt(point.x*point.x + point.y*point.y);
    }
    double cal_diff(PointType saved_Cone , PointType detected_Cone){
        double dx = saved_Cone.x - detected_Cone.x;
        double dy = saved_Cone.y - detected_Cone.y; 
        double dz = saved_Cone.z - detected_Cone.z;

        return sqrt(dx*dx + dy*dy + dz*dz);
    }

    double magnitude(const VectorXf& v) {
       return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    PointType vector2point(const VectorXf& v, PointType start_point){
        PointType tmp_point;
        tmp_point.x = v[0]+start_point.x;
        tmp_point.y = v[1]+start_point.y;
        tmp_point.z = v[2]+start_point.z;

        return tmp_point;
    } 
     
    VectorXf conduct_PCA (pcl::PointCloud<PointType>::Ptr input_cloud, int num){
        //num = 0이면 주축(분산도가 가장 큰 방향의 벡터를 의미함) , num=2이면 분산도가 가장 작은 방향의 벡터를 의미함

        //covariance matrix 생성
        Eigen::Matrix3f cov_;
        Eigen::Vector4f pc_mean_;
        VectorXf normal_;
        VectorXf singular_values_;

        pcl::computeMeanAndCovarianceMatrix(*input_cloud, cov_, pc_mean_);
        
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_, Eigen::DecompositionOptions::ComputeFullU);
        singular_values_ = svd.singularValues();
        
        // use the least singular vector as normal::PCA 수행
        normal_ = (svd.matrixU().col(num));
        if (normal_(2) < 0) { for(int i=0; i<3; i++) normal_(i) *= -1; }
        
        return normal_;// 법선 벡터를 추출
    }

    void removeOutliar(pcl::PointCloud<PointType>::Ptr cloud){
        int num_neigbor_points = 20;
        double std_multiplier = 0.1;

        pcl::StatisticalOutlierRemoval<PointType> sor;
        sor.setInputCloud (cloud);
        sor.setMeanK (num_neigbor_points);
        sor.setStddevMulThresh (std_multiplier);
        sor.filter(*cloud);
    }

    void set_ROI(vector<pcl::PointCloud<PointType>::Ptr> input_cloud_vec , pcl::PointCloud<PointType>::Ptr output_cloud){
        for (auto iter = input_cloud_vec.begin(); iter != input_cloud_vec.end(); ++iter){
            bool is_cone = true;

            size_t cloud_size = (*iter)->points.size();
            
            for(int i=0; i< cloud_size; i++){
                if((*iter)->points[i].z > 0.5) {
                    is_cone = false;
                    break;
                }
            }

            if(is_cone) {
                *output_cloud += *(*iter);
            }
        }
    }
    
    void clustering(pcl::PointCloud<PointType>::Ptr input_cloud, vector<pcl::PointCloud<PointType>::Ptr>& output_cloud_vec,
     double clusterTolerance, int minSize , int maxSize){
        
        int clusternum =0;
        
        std::vector<int> indice;
        pcl::removeNaNFromPointCloud(*input_cloud,*input_cloud,indice);
        pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>);
        
        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud(input_cloud);  // 입력 클라우드 설정
        sor.setLeafSize(1.f, 1.f, .1f);  // Voxel 크기 설정 (x, y, z)
        
        //다운샘플링을 수행
        sor.filter(*downsampled_cloud);

        if (downsampled_cloud->points.size() > 0){
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
            tree->setInputCloud(downsampled_cloud);
            std::vector<pcl::PointIndices> clusterIndices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            ec.setClusterTolerance(clusterTolerance);
            ec.setMinClusterSize(minSize);
            ec.setMaxClusterSize(maxSize);
            ec.setSearchMethod(tree);
            ec.setInputCloud(downsampled_cloud);
            ec.extract(clusterIndices);

            for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
            {
                pcl::PointCloud<PointType>::Ptr ClusterCloud(new pcl::PointCloud<PointType>);
                PointType minPoint,maxPoint;

                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                {
                    pcl::PointXYZI pt = downsampled_cloud->points[*pit];
                    if(pt.x > 0) ClusterCloud->points.push_back(pt);
                }

                if(maxSize == 1000){
                    *Clustered_Cloud += *ClusterCloud;
                }
                    output_cloud_vec.push_back(ClusterCloud);
                    clusternum++;
            }       
        }
        
    }

    void visual_cones_center(){
        //visual marker
        visualization_msgs::Marker marker;

        for(const auto& id : id_list){            
            marker.header.frame_id = cloudHeader.frame_id;
            marker.header.stamp = ros::Time().now();
            marker.ns ="cone's center point";
            marker.id = id;
            
            //cout << "marker id :: " << marker.id <<endl;
            marker.type = visualization_msgs::Marker::SPHERE; 
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = lastest_cones[id].x;
            marker.pose.position.y = lastest_cones[id].y;
            marker.pose.position.z = lastest_cones[id].z;
            
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;

            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1;
            marker.lifetime = ros::Duration(0.11);

            cone_center_markerarray.markers.push_back(marker);

        }
    }


    void visual_vector(PointType start_position, PointType end_position,float r, float g, float b, int Id){
        //visual marker
        
        for(vector<Cones>::const_iterator iter = obj_center_point.begin();iter != obj_center_point.end();iter++){
            visualization_msgs::Marker marker;
            
            marker.header.frame_id = cloudHeader.frame_id;
            marker.header.stamp = ros::Time().now();
            marker.ns ="vector";
            marker.id = Id;
            
            marker.type = visualization_msgs::Marker::ARROW; 
            marker.action = visualization_msgs::Marker::ADD;

            geometry_msgs::Point start_point;
            start_point.x = start_position.x;
            start_point.y = start_position.y;
            start_point.z = start_position.z;

            geometry_msgs::Point end_point;
            end_point.x = end_position.x;
            end_point.y = end_position.y;
            end_point.z = end_position.z;

            marker.points.push_back(geometry_msgs::Point());
            marker.points[0] = start_point;
            marker.points.push_back(geometry_msgs::Point()); 
            marker.points[1] = end_point;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;

            marker.scale.x = 0.05;
            marker.scale.y = 0.1;

            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = 1;
            marker.lifetime = ros::Duration(0.2);

            normal_vectors.markers.push_back(marker);

        }
    }

    void visual_cones_boundary(){
        //visual marker
        visualization_msgs::Marker marker;
        int Id=1000;
      
        for(vector<Cones>::const_iterator iter = obj_center_point.begin();iter != obj_center_point.end();iter++){
            
            marker.header.frame_id = cloudHeader.frame_id;
            marker.header.stamp = ros::Time().now();
            marker.id =  Id;
            
            marker.type = visualization_msgs::Marker::LINE_LIST; // 선 리스트 유형
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.02; // 선의 두께

            geometry_msgs::Point minPoint;
            minPoint.x = iter->min_point.x;
            minPoint.y = iter->min_point.y;
            minPoint.z = iter->min_point.z;

            geometry_msgs::Point maxPoint;
            maxPoint.x = iter->max_point.x;
            maxPoint.y = iter->max_point.y;
            maxPoint.z = iter->max_point.z;

            // 바운딩 박스를 구성하는 선들을 추가
            marker.points.push_back(minPoint);
            marker.points.push_back(make_Geo_Point(minPoint.x, minPoint.y, maxPoint.z));

            marker.points.push_back(minPoint);
            marker.points.push_back(make_Geo_Point(minPoint.x, maxPoint.y, minPoint.z));

            marker.points.push_back(minPoint);
            marker.points.push_back(make_Geo_Point(maxPoint.x, minPoint.y, minPoint.z));

            marker.points.push_back(make_Geo_Point(maxPoint.x, maxPoint.y, minPoint.z));
            marker.points.push_back(make_Geo_Point(maxPoint.x, maxPoint.y, maxPoint.z));

            marker.points.push_back(make_Geo_Point(maxPoint.x, minPoint.y, minPoint.z));
            marker.points.push_back(make_Geo_Point(maxPoint.x, minPoint.y, maxPoint.z));

            marker.points.push_back(make_Geo_Point(minPoint.x, minPoint.y, maxPoint.z));
            marker.points.push_back(make_Geo_Point(minPoint.x, maxPoint.y, maxPoint.z));

            marker.points.push_back(make_Geo_Point(maxPoint.x, minPoint.y, maxPoint.z));
            marker.points.push_back(make_Geo_Point(maxPoint.x, maxPoint.y, maxPoint.z));

            marker.points.push_back(make_Geo_Point(maxPoint.x, maxPoint.y, maxPoint.z));
            marker.points.push_back(make_Geo_Point(minPoint.x, maxPoint.y, maxPoint.z));

            marker.points.push_back(make_Geo_Point(maxPoint.x, maxPoint.y, minPoint.z));
            marker.points.push_back(make_Geo_Point(minPoint.x, maxPoint.y, minPoint.z));

            marker.points.push_back(make_Geo_Point(maxPoint.x, minPoint.y, minPoint.z));
            marker.points.push_back(make_Geo_Point(maxPoint.x, maxPoint.y, minPoint.z));

            marker.points.push_back(make_Geo_Point(minPoint.x, minPoint.y, maxPoint.z));
            marker.points.push_back(make_Geo_Point(maxPoint.x, minPoint.y, maxPoint.z));

            marker.points.push_back(make_Geo_Point(minPoint.x, maxPoint.y, minPoint.z));
            marker.points.push_back(make_Geo_Point(minPoint.x, maxPoint.y, maxPoint.z));

            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1;
            marker.lifetime = ros::Duration(0.5);

            Id++;
            cone_boundary_markerarray.markers.push_back(marker);

        }
    }

    geometry_msgs::Point make_Geo_Point(float x, float y, float z){
        geometry_msgs::Point point;
        
        point.x = x;
        point.y = y;
        point.z = z;
        
        return point;
    }
    
    void publishCloud(){
        // 2. Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;

        if (pubConeCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*ConeCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id =cloudHeader.frame_id;
            pubConeCloud.publish(laserCloudTemp);
        }

        if (pubROICloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*Clustered_Cloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = cloudHeader.frame_id;
            pubROICloud.publish(laserCloudTemp);
        }

        if (pub_prev_cone.getNumSubscribers() != 0){
            pcl::toROSMsg(*prev_ConeCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = cloudHeader.frame_id;
            pub_prev_cone.publish(laserCloudTemp);
        }

        
        pubConesCenter.publish(cone_center_markerarray);
        pubConesCenterKF.publish(cone_center_markerarrayKF);
        pubConesBoundary.publish(cone_boundary_markerarray);
        pubVector.publish(normal_vectors);
    }
};

