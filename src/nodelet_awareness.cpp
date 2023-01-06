#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <utils/include/all_utils.h>
#include <map_awareness.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rviz_vis.h>
#include <msg_awareness2local.h>
#include <msg_awareness.h>
#include <chrono>
#include <numeric>
#include <iostream>
namespace mlmapping_ns
{

    using namespace std;

    class AwarenessMapNodeletClass : public nodelet::Nodelet
    {
    public:
        AwarenessMapNodeletClass() { ; }
        ~AwarenessMapNodeletClass() { ; }

    private:
        // subscriber
        message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
        message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
        typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> ExactSyncPolicy;
        typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> ExactSyncPolicyOdom;
        message_filters::Synchronizer<ExactSyncPolicy> *exactSync_;
        message_filters::Synchronizer<ExactSyncPolicyOdom> *OdomexactSync_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> ApproxSyncPolicy;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> ApproxSyncPolicyOdom;
        message_filters::Synchronizer<ApproxSyncPolicy> *approxSync_;
        message_filters::Synchronizer<ApproxSyncPolicyOdom> *OdomapproxSync_;
        // publisher
        msg_awareness2local *a2w_pub;
        msg_awareness *awarenessmap_pub;
        tf2_ros::TransformBroadcaster br;
        // Timer
        ros::Timer timer_;
        int count = 0;
        double time_total = 0;
        // variable
        awareness_map_cylindrical *awareness_map;
        int pc_sample_cnt;
        bool publish_T_wb;
        bool publish_T_bs;
        geometry_msgs::TransformStamped transformStamped_T_wb;
        geometry_msgs::TransformStamped transformStamped_T_wa;
        geometry_msgs::TransformStamped transformStamped_T_bs;

        void pc_odom_input_callback(const sensor_msgs::PointCloud2::ConstPtr &pc_Ptr,
                                    const nav_msgs::Odometry::ConstPtr &pose_Ptr)
        {
            //        tic_toc_ros tt;
            //        static double sum_t = 0;
            //        static int count = 0;
            auto t1 = std::chrono::system_clock::now();
            SE3 T_wb(SO3(Quaterniond(pose_Ptr->pose.pose.orientation.w,
                                     pose_Ptr->pose.pose.orientation.x,
                                     pose_Ptr->pose.pose.orientation.y,
                                     pose_Ptr->pose.pose.orientation.z)),
                     Vec3(pose_Ptr->pose.pose.position.x,
                          pose_Ptr->pose.pose.position.y,
                          pose_Ptr->pose.pose.position.z));
            if (publish_T_wb)
            {
                transformStamped_T_wb.header.stamp = pose_Ptr->header.stamp;
                transformStamped_T_wb.transform.translation.x = T_wb.translation().x();
                transformStamped_T_wb.transform.translation.y = T_wb.translation().y();
                transformStamped_T_wb.transform.translation.z = T_wb.translation().z();
                transformStamped_T_wb.transform.rotation.x = T_wb.so3().unit_quaternion().x();
                transformStamped_T_wb.transform.rotation.y = T_wb.so3().unit_quaternion().y();
                transformStamped_T_wb.transform.rotation.z = T_wb.so3().unit_quaternion().z();
                transformStamped_T_wb.transform.rotation.w = T_wb.so3().unit_quaternion().w();
                br.sendTransform(transformStamped_T_wb);
                transformStamped_T_wa.header.stamp = pose_Ptr->header.stamp;
                transformStamped_T_wa.transform.translation = transformStamped_T_wb.transform.translation;
                br.sendTransform(transformStamped_T_wa);
            }
            if (publish_T_bs)
            {
                transformStamped_T_bs.header.stamp = pose_Ptr->header.stamp;
                br.sendTransform(transformStamped_T_bs);
            }
            auto t10 = std::chrono::system_clock::now();
            PointCloudP_ptr cloud(new PointCloudP);
            pcl::fromROSMsg(*pc_Ptr, *cloud);
            
            if (pc_Ptr->is_dense)
            {
            }
            else
            { // remove invalid pts
                vector<int> index;
                pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
            }

            int pcsize = static_cast<int>(cloud->size());
            vector<Vec3> pc_eigen;
            if (pcsize > pc_sample_cnt)
            {
                for (int i = 0; i < pc_sample_cnt; i++)
                {
                    size_t rand_idx = static_cast<size_t>(rand() % pcsize);
                    PointP pt = cloud->at(rand_idx);
                    pc_eigen.push_back(Vec3(static_cast<double>(pt.x),
                                            static_cast<double>(pt.y),
                                            static_cast<double>(pt.z)));
                }
            }
            else
            {
                for (int i = 0; i < pcsize; i++)
                {
                    PointP pt = cloud->at(i);
                    pc_eigen.push_back(Vec3(static_cast<double>(pt.x),
                                            static_cast<double>(pt.y),
                                            static_cast<double>(pt.z)));
                }
            }
            auto t11 = std::chrono::system_clock::now();
            awareness_map->input_pc_pose(pc_eigen, T_wb);
            awarenessmap_pub->pub(awareness_map, pose_Ptr->header.stamp);

            a2w_pub->pub(awareness_map->T_wa,
                         awareness_map->l2g_msg_hit_pts_l,
                         awareness_map->l2g_msg_miss_pts_l,
                         pose_Ptr->header.stamp);
            auto t2 = std::chrono::system_clock::now();
            std::chrono::duration<double> diff = t2 - t1;
            // std::chrono::duration<double> diff = t11 - t10;
            time_total += diff.count() * 1000;
            printf("Awareness map single time: %.8f ms, ave-time cost: %.8f ms\n", diff.count() * 1000, time_total / (++count));

            //        sum_t+=tt.dT_ms();
            //        count++;
            //    cout << awareness_map->l2g_msg_hit_pts_l.size()<<" "<<awareness_map->l2g_msg_miss_pts_l.size() << endl;
        }

        void pc_pose_input_callback(const sensor_msgs::PointCloud2::ConstPtr &pc_Ptr,
                                    const geometry_msgs::PoseStamped::ConstPtr &pose_Ptr)
        {
            //        tic_toc_ros tt;
            //        static double sum_t = 0;
            //        static int count = 0;

            SE3 T_wb(SO3(Quaterniond(pose_Ptr->pose.orientation.w,
                                     pose_Ptr->pose.orientation.x,
                                     pose_Ptr->pose.orientation.y,
                                     pose_Ptr->pose.orientation.z)),
                     Vec3(pose_Ptr->pose.position.x,
                          pose_Ptr->pose.position.y,
                          pose_Ptr->pose.position.z));
            if (publish_T_wb)
            {
                transformStamped_T_wb.header.stamp = pose_Ptr->header.stamp;
                transformStamped_T_wb.transform.translation.x = T_wb.translation().x();
                transformStamped_T_wb.transform.translation.y = T_wb.translation().y();
                transformStamped_T_wb.transform.translation.z = T_wb.translation().z();
                transformStamped_T_wb.transform.rotation.x = T_wb.so3().unit_quaternion().x();
                transformStamped_T_wb.transform.rotation.y = T_wb.so3().unit_quaternion().y();
                transformStamped_T_wb.transform.rotation.z = T_wb.so3().unit_quaternion().z();
                transformStamped_T_wb.transform.rotation.w = T_wb.so3().unit_quaternion().w();
                br.sendTransform(transformStamped_T_wb);

                transformStamped_T_wa.header.stamp = pose_Ptr->header.stamp;
                transformStamped_T_wa.transform.translation = transformStamped_T_wb.transform.translation;
                br.sendTransform(transformStamped_T_wa);
            }
            if (publish_T_bs)
            {
                transformStamped_T_bs.header.stamp = pose_Ptr->header.stamp;
                br.sendTransform(transformStamped_T_bs);
            }

            PointCloudP_ptr cloud(new PointCloudP);
            pcl::fromROSMsg(*pc_Ptr, *cloud);
            if (pc_Ptr->is_dense)
            {
            }
            else
            { // remove invalid pts
                vector<int> index;
                pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
            }

            int pcsize = static_cast<int>(cloud->size());
            vector<Vec3> pc_eigen;
            if (pcsize > pc_sample_cnt)
            {
                for (int i = 0; i < pc_sample_cnt; i++)
                {
                    size_t rand_idx = static_cast<size_t>(rand() % pcsize);
                    PointP pt = cloud->at(rand_idx);
                    pc_eigen.push_back(Vec3(static_cast<double>(pt.x),
                                            static_cast<double>(pt.y),
                                            static_cast<double>(pt.z)));
                }
            }
            else
            {
                for (int i = 0; i < pcsize; i++)
                {
                    PointP pt = cloud->at(i);
                    pc_eigen.push_back(Vec3(static_cast<double>(pt.x),
                                            static_cast<double>(pt.y),
                                            static_cast<double>(pt.z)));
                }
            }

            awareness_map->input_pc_pose(pc_eigen, T_wb);
            awarenessmap_pub->pub(awareness_map, pose_Ptr->header.stamp);

            a2w_pub->pub(awareness_map->T_wa,
                         awareness_map->l2g_msg_hit_pts_l,
                         awareness_map->l2g_msg_miss_pts_l,
                         pose_Ptr->header.stamp);

            //        sum_t+=tt.dT_ms();
            //        count++;
            //        cout << (sum_t/count) << endl;
        }

        void timerCb(const ros::TimerEvent &event)
        {
            transformStamped_T_wa.header.stamp = ros::Time::now();
            br.sendTransform(transformStamped_T_wa);
            if (publish_T_wb)
            {
                transformStamped_T_wb.header.stamp = ros::Time::now();
                br.sendTransform(transformStamped_T_wb);
            }
            if (publish_T_bs)
            {
                transformStamped_T_bs.header.stamp = ros::Time::now();
                br.sendTransform(transformStamped_T_bs);
            }
        }

        virtual void onInit()
        {
            cout << "awareness map node:" << endl;
            ros::NodeHandle &nh = getMTPrivateNodeHandle();
            string configFilePath;
            nh.getParam("/mlmapping_configfile", configFilePath);
            cout << "config file path: " << configFilePath << endl;

            // init map
            pc_sample_cnt = getIntVariableFromYaml(configFilePath, "mlmapping_sample_cnt");
            awareness_map = new awareness_map_cylindrical();

            Mat4x4 T_bs_mat = Mat44FromYaml(configFilePath, "T_B_S");
            SE3 T_bs = SE3(T_bs_mat.topLeftCorner(3, 3),
                           T_bs_mat.topRightCorner(3, 1));
            awareness_map->setTbs(T_bs);
            awareness_map->init_map(getDoubleVariableFromYaml(configFilePath, "mlmapping_am_d_Rho"),
                                    getDoubleVariableFromYaml(configFilePath, "mlmapping_am_d_Phi_deg"),
                                    getDoubleVariableFromYaml(configFilePath, "mlmapping_am_d_Z"),
                                    getIntVariableFromYaml(configFilePath, "mlmapping_am_n_Rho"),
                                    getIntVariableFromYaml(configFilePath, "mlmapping_am_n_Z_below"),
                                    getIntVariableFromYaml(configFilePath, "mlmapping_am_n_Z_over"),
                                    getBoolVariableFromYaml(configFilePath, "mlmapping_use_raycasting"));

            // init transform
            // transformStamped_T_wb
            publish_T_wb = getBoolVariableFromYaml(configFilePath, "publish_T_wb");
            transformStamped_T_wb.header.frame_id = getStringFromYaml(configFilePath, "world_frame_id");
            transformStamped_T_wb.child_frame_id = getStringFromYaml(configFilePath, "body_frame_id");
            transformStamped_T_wb.transform.translation.x = 0;
            transformStamped_T_wb.transform.translation.y = 0;
            transformStamped_T_wb.transform.translation.z = 0;
            transformStamped_T_wb.transform.rotation.x = 0;
            transformStamped_T_wb.transform.rotation.y = 0;
            transformStamped_T_wb.transform.rotation.z = 0;
            transformStamped_T_wb.transform.rotation.w = 1;
            // transformStamped_T_wa
            transformStamped_T_wa = transformStamped_T_wb;
            transformStamped_T_wa.child_frame_id = getStringFromYaml(configFilePath, "awareness_frame_id");
            // transformStamped_T_bs
            publish_T_bs = getBoolVariableFromYaml(configFilePath, "publish_T_bs");
            transformStamped_T_bs.header.frame_id = getStringFromYaml(configFilePath, "body_frame_id");
            transformStamped_T_bs.child_frame_id = getStringFromYaml(configFilePath, "sensor_frame_id");
            transformStamped_T_bs.transform.translation.x = T_bs.translation().x();
            transformStamped_T_bs.transform.translation.y = T_bs.translation().y();
            transformStamped_T_bs.transform.translation.z = T_bs.translation().z();
            transformStamped_T_bs.transform.rotation.x = T_bs.so3().unit_quaternion().x();
            transformStamped_T_bs.transform.rotation.y = T_bs.so3().unit_quaternion().y();
            transformStamped_T_bs.transform.rotation.z = T_bs.so3().unit_quaternion().z();
            transformStamped_T_bs.transform.rotation.w = T_bs.so3().unit_quaternion().w();

            // init publisher
            awarenessmap_pub = new msg_awareness(nh, "/mlmapping_awareness");
            a2w_pub = new msg_awareness2local(nh, "/awareness2local", 2);
            bool use_exactsync = getBoolVariableFromYaml(configFilePath, "use_exactsync");
            bool use_odom = getBoolVariableFromYaml(configFilePath, "use_odom");

            if (use_exactsync)
            {
                pc_sub.subscribe(nh, "/mlmapping/pc", 10);
                if (use_odom)
                {
                    odom_sub.subscribe(nh, "/mlmapping/odom", 10);
                    OdomexactSync_ = new message_filters::Synchronizer<ExactSyncPolicyOdom>(ExactSyncPolicyOdom(5), pc_sub, odom_sub);
                    OdomexactSync_->registerCallback(boost::bind(&AwarenessMapNodeletClass::pc_odom_input_callback, this, _1, _2));
                }
                else

                {
                    pose_sub.subscribe(nh, "/mlmapping/pose", 10);
                    exactSync_ = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(5), pc_sub, pose_sub);
                    exactSync_->registerCallback(boost::bind(&AwarenessMapNodeletClass::pc_pose_input_callback, this, _1, _2));
                }
                cout << "ExactSyncPolicy" << endl;
            }
            else
            {
                pc_sub.subscribe(nh, "/mlmapping/pc", 1);
                if (use_odom)
                {
                    odom_sub.subscribe(nh, "/mlmapping/odom", 1);
                    OdomapproxSync_ = new message_filters::Synchronizer<ApproxSyncPolicyOdom>(ApproxSyncPolicyOdom(100), pc_sub, odom_sub);
                    OdomapproxSync_->registerCallback(boost::bind(&AwarenessMapNodeletClass::pc_odom_input_callback, this, _1, _2));
                }
                else

                {
                    pose_sub.subscribe(nh, "/mlmapping/pose", 1);
                    approxSync_ = new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(100), pc_sub, pose_sub);
                    approxSync_->registerCallback(boost::bind(&AwarenessMapNodeletClass::pc_pose_input_callback, this, _1, _2));
                }
                cout << "ApproxSyncPolicy" << endl;
            }
            timer_ = nh.createTimer(ros::Duration(0.2), boost::bind(&AwarenessMapNodeletClass::timerCb, this, _1));
        }
    }; // class AwarenessMapNodeletClass
} // namespace mlmapping_ns

PLUGINLIB_EXPORT_CLASS(mlmapping_ns::AwarenessMapNodeletClass, nodelet::Nodelet)
