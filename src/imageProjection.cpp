#include "utility.h"
#include "lio_sam/cloud_info.h"

#include "algorithm"

#include "opencv2/opencv.hpp"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                  (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                  (uint16_t, ring, ring) (float, time, time)
)


struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                 (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                 (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
                                 (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

typedef pcl::PointXYZRGB PointTypeRGB;

class ImageProjection : public ParamServer
{
private:

    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;
    
    image_transport::Publisher pubImg;
    cv::Mat laserImg;

    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr        laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr  tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr          fullCloud;
    pcl::PointCloud<PointType>::Ptr          extractedCloud;

    pcl::PointCloud<PointTypeRGB>::Ptr       mlxCloud; // added by kbk

    int deskewFlag;
    cv::Mat rangeMat;

    bool  odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;
    
    lio_sam::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    vector<int> columnIdnCountVec;

public:
    ImageProjection()
    : deskewFlag(0)
    {
        //subOdom = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, & ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 2000, & ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay()); // substitute lidar odometry
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1);
    
        image_transport::ImageTransport it(nh);
        pubImg = it.advertise("laser/image", 10);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        mlxCloud.reset(new pcl::PointCloud<PointTypeRGB>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();       
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();

        mlxCloud->clear();

        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        laserImg = cv::Mat(N_SCAN, Horizon_SCAN, CV_8UC3); // for opencv debug   

        firstPointFlag = true;
        odomDeskewFlag = false;

        columnIdnCountVec.assign(N_SCAN, 0);
    }

    ~ImageProjection() {}

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        auto start = std::chrono::high_resolution_clock::now();

        if (!cachePointCloud(laserCloudMsg))
            return;
        
        if (!deskewInfo())
            return;
        
        projectPointCloud();

        cloudExtraction();

        publishClouds();

        resetParameters();
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (ipTime)
        std::cout << "imageProjection::cloudHandler() : " << duration.count() << " ms" << std::endl;
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
        {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else if (sensor == SensorType::OUSTER)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else if (sensor == SensorType::MLX)
        {
            // Convert to Velodyne format
            //pcl::PointCloud<PointTypeRGB> mlxCloud;
            pcl::moveFromROSMsg(currentCloudMsg, *mlxCloud);
            laserCloudIn->points.resize(mlxCloud->size());
            laserCloudIn->is_dense = mlxCloud->is_dense;

            if (sensor_mode == SensorMode::VERTICAL)
            {
                float theta = M_PI / 2; // The angle of rotation in radians
                Eigen::Affine3f transform = Eigen::Affine3f::Identity();
                transform.translation() << 0.0, 0.0, 0.0;
                transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX())); // vertical to horizon
                // Executing the transformation
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
                pcl::transformPointCloud(*mlxCloud, *transformedCloud, transform);
                for (size_t i = 0; i < laserCloudIn->size(); i++)
                {
                    auto &src = transformedCloud->points[N_SCAN*(Horizon_SCAN-(1+(i%Horizon_SCAN)))+(i/Horizon_SCAN)];
                    auto &dst = laserCloudIn->points[i];
                    dst.x = src.x;
                    dst.y = src.y;
                    dst.z = src.z;
                    laserImg.at<cv::Vec3b>(i/Horizon_SCAN, i%Horizon_SCAN) = cv::Vec3b(src.b, src.g, src.r);
                    //dst.intensity = src.intensity;
                    dst.ring = (int) i / Horizon_SCAN;
                    //dst.time = src.t * 1e-9f;
                }
            }
            else // horizon
            {
                for (size_t i = 0; i < laserCloudIn->size(); i++)
                {
                    auto &src = mlxCloud->points[i];
                    auto &dst = laserCloudIn->points[i];
                    dst.x = src.x;
                    dst.y = src.y;
                    dst.z = src.z;
                    laserImg.at<cv::Vec3b>(i/Horizon_SCAN, i%Horizon_SCAN) = cv::Vec3b(src.b, src.g, src.r);
                    //dst.intensity = src.intensity;
                    dst.ring = (int) i / mlxCloud->width;
                    //dst.time = src.t * 1e-9f;
                }
            }
            
        }
        else
        {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }
        
        // get timestamp
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // check ring channel
        static int ringFlag = 1; // 0 // Do not check the ringFlag by bkkim
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }
        
        // check point time
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t")
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock(odoLock);
        cloudInfo.imuAvailable = false;
        odomDeskewInfo();
        return true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.1 /*0.01*/)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beginning of the scan
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }
        
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw = yaw;

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;        
    }

    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            int columnIdn = -1;
            if (sensor == SensorType::VELODYNE || sensor == SensorType::OUSTER)
            {
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                static float ang_res_x = 360.0/float(Horizon_SCAN);
                columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
                if (columnIdn >= Horizon_SCAN)
                    columnIdn -= Horizon_SCAN;
            }
            else if (sensor == SensorType::LIVOX)
            {
                columnIdn = columnIdnCountVec[rowIdn];
                columnIdnCountVec[rowIdn] += 1;
            }
            else if (sensor == SensorType::MLX)
            {
                if (sensor_mode == SensorMode::VERTICAL)
                {
                    float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                    static float ang_res_x = 35.0 / float(Horizon_SCAN);
                    columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
                    if (columnIdn >= Horizon_SCAN)
                        columnIdn -= Horizon_SCAN;
                }
                else
                {
                    float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                    static float ang_res_x = 120.0 / float(Horizon_SCAN);
                    columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
                    if (columnIdn >= Horizon_SCAN)
                        columnIdn -= Horizon_SCAN;
                }
            }

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            //thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.pointColInd[count] = j;
                    // save range info
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count - 1 - 5;
        }
    }

    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        
        // delever the original sensor_msgs (kbk)
        sensor_msgs::PointCloud2 tempCloud;
        pcl::toROSMsg(*mlxCloud, tempCloud);
        cloudInfo.cloud = tempCloud;

        pubLaserCloudInfo.publish(cloudInfo);

        // cv::Mat to ROS image message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", laserImg).toImageMsg();
        pubImg.publish(msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    ImageProjection IP;

    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
