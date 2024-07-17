/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * BSD 3-Clause License
 * @author Qingwen Zhang (https://kin-zhang.github.io/)
 * @date: 2023-04-07 13:29
 * @details: No ROS version, speed up the process
 * 
 * This file is part of DynamicMap Benchmark work (https://github.com/KTH-RPL/DynamicMap_Benchmark).
 * If you find this repo helpful, please cite the respective publication as 
 * listed on the above website.
 * 
 * Input: PCD files + Prior raw global map , check our benchmark in dufomap
 * Output: Cleaned global map
 */

#include <filesystem>
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include "octomapper.h"

namespace octomap {

    MapUpdater::MapUpdater(const std::string &config_file_path ){
        if (std::filesystem::exists(config_file_path))
            LOG(INFO) << "Config file loaded from: " << config_file_path;
        else{
            LOG(ERROR) << "Config file not found: " << config_file_path;
            exit(1);
        }
        toml::table tbl = toml::parse_file(config_file_path);
        cfg_.readtoml(tbl);

        // initialize octomap object & params
        m_octree = new OcTreeT(cfg_.m_res);
        m_octree->setProbHit(cfg_.probHit);
        m_octree->setProbMiss(cfg_.probMiss);
        m_octree->setClampingThresMin(cfg_.thresMin);
        m_octree->setClampingThresMax(cfg_.thresMax);
        m_maxTreeDepth = m_treeDepth;

        ground_pts.reset(new pcl::PointCloud<PointType>());
        raw_map_ptr_.reset(new pcl::PointCloud<PointType>());
        LOG(INFO) << "resolution: " << cfg_.m_res << ". Ground filter: " 
        << cfg_.filterGroundPlane << ", Noise filter: " << cfg_.filterNoise;
    }

    void MapUpdater::run(pcl::PointCloud<PointType>::Ptr const& single_pc){
        // read pose in VIEWPOINT Field in pcd
        float x_curr = single_pc->sensor_origin_[0];
        float y_curr = single_pc->sensor_origin_[1];
        float z_curr = single_pc->sensor_origin_[2];

        octomap::point3d sensorOrigin(x_curr, y_curr, z_curr);
        if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
            || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
        {
            LOG(WARNING) << "Could not generate Key for origin:" << sensorOrigin;
            LOG(WARNING) << "Skipping this frame. Please check your pose correctly and small enough. Otherwise Octomap index will be out of bound.";
            return;
        }

        LOG_IF(INFO, cfg_.verbose_) << "x_curr: " << x_curr << ", y_curr: " << y_curr;

        if (cfg_.need_transform){
            // NOTE(Qingwen): Since sometime the VIEWPOINT/pose is on map frame but the point cloud is not in the map frame, 
            LOG_IF(INFO, cfg_.verbose_) << "Transforming the point cloud to the map frame";
            float qx = single_pc->sensor_orientation_.x();
            float qy = single_pc->sensor_orientation_.y();
            float qz = single_pc->sensor_orientation_.z();
            float qw = single_pc->sensor_orientation_.w();

            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform.block<3,3>(0,0) = Eigen::Quaternionf(qw, qx, qy, qz).toRotationMatrix();
            transform.block<3,1>(0,3) = Eigen::Vector3f(x_curr, y_curr, z_curr);
            pcl::transformPointCloud(*single_pc, *single_pc, transform);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        if(cfg_.filterNoise){
            pcl::StatisticalOutlierRemoval<PointType> sor(true);
            sor.setInputCloud(single_pc);
            sor.setMeanK (cfg_.filterMeanK);
            sor.setStddevMulThresh (cfg_.StddevMulThresh);
            sor.filter (*cloud_filtered);
        }
        else{
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*single_pc, *cloud_filtered, indices);
        }

        *raw_map_ptr_ += *cloud_filtered;
        
        pcl::PointCloud<PointType>::Ptr pc_nonground(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr pc_ground(new pcl::PointCloud<PointType>);

        timing[1].start("Ground Segmentation");
        if(cfg_.filterGroundPlane)
            filterGroundPlane(cloud_filtered, pc_ground, pc_nonground);
        else
            pc_nonground = cloud_filtered;

        // instead of direct scan insertion, compute update to filter ground:
        octomap::KeySet free_cells, occupied_cells;
        // step A: insert ground points only as free so that we will not get false obstacles in ground pts
        for (pcl::PointCloud<PointType>::const_iterator it = pc_ground->begin(); it != pc_ground->end(); ++it){
            point3d point(it->x, it->y, it->z);

            if ((cfg_.m_minRange > 0) && (point - sensorOrigin).norm() < cfg_.m_minRange) continue;

            // maxrange check
            if ((cfg_.m_maxRange > 0.0) && ((point - sensorOrigin).norm() > cfg_.m_maxRange) ) {
                point = sensorOrigin + (point - sensorOrigin).normalized() * cfg_.m_maxRange;
            }

            // only clear space (ground points)
            if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }

            octomap::OcTreeKey endKey;
            if (m_octree->coordToKeyChecked(point, endKey)){
                updateMinKey(endKey, m_updateBBXMin);
                updateMaxKey(endKey, m_updateBBXMax);
            } else{
                LOG_IF(WARNING,cfg_.verbose_) << "Could not generate Key for endpoint "<<point;
            }
        }
        if(pc_ground->size() > 0){
            LOG_IF(INFO,cfg_.verbose_) << "Ground points: " << pc_ground->size();
            *ground_pts += *pc_ground;
        }
        timing[1].stop();
        
        timing[2].start("Ray SetFreeOc");
        // step B: free on ray, occupied on endpoint
        for (pcl::PointCloud<PointType>::const_iterator it = pc_nonground->begin(); it != pc_nonground->end(); ++it){
            octomap::point3d point(it->x, it->y, it->z);
            double pts_distance = (point - sensorOrigin).norm();
            // range filtering
            if ((cfg_.m_minRange > 0) && (pts_distance < cfg_.m_minRange)) continue;

            if ((cfg_.m_maxRange < 0.0) || (pts_distance <= cfg_.m_maxRange) ) {
                // free cells
                if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());

                // occupied endpoint
                octomap::OcTreeKey key;
                if (m_octree->coordToKeyChecked(point, key)){
                    occupied_cells.insert(key);
                    updateMinKey(key, m_updateBBXMin);
                    updateMaxKey(key, m_updateBBXMax);
                }
            }
            else{
                octomap::point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * cfg_.m_maxRange;
                if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());

                    octomap::OcTreeKey endKey;
                    if (m_octree->coordToKeyChecked(new_end, endKey)){
                        free_cells.insert(endKey);
                        updateMinKey(endKey, m_updateBBXMin);
                        updateMaxKey(endKey, m_updateBBXMax);
                    }
                    else
                        LOG(WARNING) << "Could not generate Key for endpoint " << new_end;
                }
            }
        }
        timing[2].stop();
        timing[3].start("Update Octree");
        for (octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it)
            m_octree->updateNode(*it, false);

        for (octomap::KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; ++it)
            m_octree->updateNode(*it, true);

        timing[3].stop();
        
        timing[4].start("Prune Tree");
        // step D: prune tree if config as true
        if(cfg_.m_prune){
            m_octree->prune();
        }
        timing[4].stop();
    }

    void MapUpdater::filterGroundPlane(pcl::PointCloud<PointType>::Ptr const& pc, 
                           pcl::PointCloud<PointType>::Ptr &ground, 
                           pcl::PointCloud<PointType>::Ptr &nonground){
        if (pc->size() < 50){
            LOG(WARNING) << "Pointcloud in OctomapServer too small, skipping ground plane extraction";
            nonground = pc;
            return;
        }

        // plane detection for ground plane removal:
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        // Create the segmentation object and set up:
        pcl::SACSegmentation<PointType> seg;
        seg.setOptimizeCoefficients (true);

        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold (cfg_.m_groundFilterDistance);
        seg.setAxis(Eigen::Vector3f(0,0,1));
        seg.setEpsAngle(cfg_.m_groundFilterAngle);

        // Create the filtering object
        seg.setInputCloud(pc);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0){
            LOG_IF(WARNING, cfg_.verbose_) << "PCL segmentation did not find any plane.";
            nonground = pc;
            return;
        }
        pcl::ExtractIndices<PointType> extract;
        bool groundPlaneFound = false;
        extract.setInputCloud(pc);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*ground);
        if(inliers->indices.size() != pc->size()){
            extract.setNegative(true);
            pcl::PointCloud<PointType> cloud_out;
            extract.filter(cloud_out);
            *nonground += cloud_out;
        }
    }
    void VoxelPointCloud(const pcl::PointCloud<PointType>::Ptr& cloud, pcl::PointCloud<PointType>::Ptr& cloud_voxelized, const double voxel_size) {
        if(voxel_size < 0.05) {
            *cloud_voxelized = *cloud;
            LOG_IF(WARNING, false) << "Voxel size is too small, no need to voxel grid filter!";
            return;
        }
        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel_grid.filter(*cloud_voxelized);
    }

    std::string MapUpdater::saveMap(std::string const& folder_path) {
        pcl::PointCloud<PointType>::Ptr octomap_map_(new pcl::PointCloud<PointType>);
        LOG(INFO) << "Saving cleaned map to pcd file, we will save " << ANSI_MAGENTA << ((!cfg_.output_downsampled_map) ? "point-level" : "voxel-level") << ANSI_RESET << " map.";

        if(cfg_.output_downsampled_map){
            cfg_.filename += "_voxel";
            // traverse the octree and save the points, traverse all leafs in the tree:
            for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth), end = m_octree->end(); it != end; ++it){
                if (m_octree->isNodeOccupied(*it)){
                    // get the center of the voxel
                    double z = it.getZ();
                    double x = it.getX();
                    double y = it.getY();
                    PointType p(x, y, z);
                    octomap_map_->push_back(p);
                }
            }
            pcl::VoxelGrid<PointType> voxel_grid;
            voxel_grid.setInputCloud(ground_pts);
            voxel_grid.setLeafSize(cfg_.m_res, cfg_.m_res, cfg_.m_res);
            voxel_grid.filter(*ground_pts);
        }
        else{
            
            for(auto &pt: raw_map_ptr_->points) {
                octomap::point3d point(pt.x, pt.y, pt.z);
                octomap::OcTreeNode* node = m_octree->search(point);
                if (node == nullptr){
                    LOG_IF(WARNING, cfg_.verbose_) << "Cannot find the Key in octomap at: " << point;
                    continue;
                }
                if (m_octree->isNodeOccupied(node))
                    octomap_map_->push_back(pt);
            }
        }
        // NOTE(Qingwen): since we have already filtered the ground points at the beginning.
        *octomap_map_ += *ground_pts;

        if (octomap_map_->size() == 0) {
            LOG(WARNING) << "\noctomap_map_ is empty, no map is saved";
            return "[Error]: Empty map!";
        }
        std::string filename = cfg_.filename + "_output.pcd";
        pcl::io::savePCDFileBinary(folder_path + "/" + filename, *octomap_map_);
        return filename;
    }
}  // namespace octomap