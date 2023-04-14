/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Kin ZHANG (https://kin-zhang.github.io/)
 * @date: 2023-04-07 13:29
 * @details: No ROS version, speed up the process
 *
 * Input: PCD files + Prior raw global map , check our benchmark in dufomap
 * Output: Cleaned global map
 */

#include <filesystem>
#include <glog/logging.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "octomapper.h"

namespace octomap {

    MapUpdater::MapUpdater(const std::string &config_file_path ){
        yconfig = YAML::LoadFile(config_file_path);
        MapUpdater::setConfig();
        // LOG_IF(INFO, cfg_.verbose_) << "Config file loaded: " << std::filesystem::canonical(config_file_path);

        // initialize octomap object & params
        m_octree = new OcTreeT(cfg_.m_res);
        m_octree->setProbHit(cfg_.probHit);
        m_octree->setProbMiss(cfg_.probMiss);
        m_octree->setClampingThresMin(cfg_.thresMin);
        m_octree->setClampingThresMax(cfg_.thresMax);
        m_maxTreeDepth = m_treeDepth;

        ground_pts.reset(new pcl::PointCloud<PointType>());
    }
    
    void MapUpdater::setConfig(){
        cfg_.m_res = yconfig["resolution"].as<float>();
        cfg_.m_maxRange = yconfig["maxRange"].as<float>();
        cfg_.m_minRange = yconfig["minRange"].as<float>();

        cfg_.probHit = yconfig["probHit"].as<float>();
        cfg_.probMiss = yconfig["probMiss"].as<float>();
        cfg_.thresMin = yconfig["thresMin"].as<float>();
        cfg_.thresMax = yconfig["thresMax"].as<float>();

        cfg_.m_prune = yconfig["prune_tree"].as<bool>();
        cfg_.verbose_ = yconfig["verbose"].as<bool>();
        cfg_.replace_intensity = yconfig["replace_intensity"].as<bool>();

        cfg_.filterGroundPlane = yconfig["filterGroundPlane"].as<bool>();
        if(cfg_.filterGroundPlane){
            cfg_.m_groundFilterDistance = yconfig["m_groundFilterDistance"].as<float>();
            cfg_.m_groundFilterAngle = yconfig["m_groundFilterAngle"].as<float>();
            cfg_.m_groundFilterPlaneDistance = yconfig["m_groundFilterPlaneDistance"].as<float>();
        }
        // if (yconfig["m_groundFilterPlaneDistance"].IsDefined()) {
        //     cfg_.m_groundFilterPlaneDistance = yconfig["m_groundFilterPlaneDistance"].as<float>();
        // }
    }

    void MapUpdater::run(pcl::PointCloud<PointType>::Ptr const& single_pc){
        // read pose in VIEWPOINT Field in pcd
        float x_curr = single_pc->sensor_origin_[0];
        float y_curr = single_pc->sensor_origin_[1];
        float z_curr = single_pc->sensor_origin_[2];
        LOG_IF(INFO, cfg_.verbose_) << "x_curr: " << x_curr << ", y_curr: " << y_curr;

        octomap::point3d sensorOrigin(x_curr, y_curr, z_curr);
        if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
            || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
        {
            LOG(WARNING) << "Could not generate Key for origin:" << sensorOrigin;
        }
        
        pcl::PointCloud<PointType>::Ptr pc_nonground(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr pc_ground(new pcl::PointCloud<PointType>);

        // for filter NaN pts
        pcl::PointCloud<PointType>::Ptr noNan_single_pc(new pcl::PointCloud<PointType>);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*single_pc, *noNan_single_pc, indices);

        timing.start("0. Fit ground   ");
        if(cfg_.filterGroundPlane){
            filterGroundPlane(noNan_single_pc, pc_ground, pc_nonground);
        }
        else{
            pc_nonground = noNan_single_pc;
        }
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
        timing.stop("0. Fit ground   ");
        timing.start("1. Ray SetFreeOc");
        // step B: free on ray, occupied on endpoint
        for (pcl::PointCloud<PointType>::const_iterator it = pc_nonground->begin(); it != pc_nonground->end(); ++it){
            octomap::point3d point(it->x, it->y, it->z);

            // range filtering
            if ((cfg_.m_minRange > 0) && ((point - sensorOrigin).norm() < cfg_.m_minRange)) continue;

            if ((cfg_.m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= cfg_.m_maxRange) ) {
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
                    else{
                        LOG(WARNING) << "Could not generate Key for endpoint " << new_end;
                    }
                }
            }
        }
        timing.stop("1. Ray SetFreeOc");
        timing.start("2. Update Octree");
        // step C: update octree
        for (octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
            m_octree->updateNode(*it, false);
        }
        for (octomap::KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; ++it){
            m_octree->updateNode(*it, true);
        }
        timing.stop("2. Update Octree");
        timing.start("3. Prune Tree   ");
        // step D: prune tree if config as true
        if(cfg_.m_prune){
            m_octree->prune();
        }
        timing.stop("3. Prune Tree   ");
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

    void MapUpdater::saveMap(std::string const& folder_path) {
        if(cfg_.replace_intensity){
            // load raw map
            using PointT = pcl::PointXYZI;

            pcl::PointCloud<PointT>::Ptr rawmap(new pcl::PointCloud<PointT>);
            std::string rawmap_path = folder_path + "/raw_map.pcd";
            pcl::io::loadPCDFile<PointT>(rawmap_path, *rawmap);

            pcl::PointCloud<PointT>::Ptr octomap_map_(new pcl::PointCloud<PointT>);
            octomap::OcTreeKey key;
            for(auto &pt: rawmap->points){
                octomap::point3d point(pt.x, pt.y, pt.z);
                octomap::OcTreeNode* node = m_octree->search(point);
                if (node == nullptr){
                    LOG_IF(WARNING, cfg_.verbose_) << "Cannot find the Key in octomap at: " << point;
                    continue;
                }
                if (m_octree->isNodeOccupied(node)){
                    pt.intensity = 0;
                    octomap_map_->push_back(pt);
                }
                // else{
                //     pt.intensity = 1;
                //     octomap_map_->push_back(pt);
                // }
            }
            LOG(INFO) << "\nground pts size" << ground_pts->size();
            // insert all ground pts which in octomap there are free
            for(auto &pt: ground_pts->points){
                pcl::PointXYZI pti;
                pti.x = pt.x;
                pti.y = pt.y;
                pti.z = pt.z;
                pti.intensity = 0;
                octomap_map_->push_back(pti);
            }
            if (octomap_map_->size() == 0) {
                LOG(WARNING) << "\noctomap_map_ is empty, no map is saved";
                return;
            }
            pcl::io::savePCDFileBinary(folder_path + "/octomap_output.pcd", *octomap_map_);
        }
        else{
            pcl::PointCloud<PointType>::Ptr octomap_map_(new pcl::PointCloud<PointType>);
            LOG(INFO) << "\nSaving octomap from octree to pcd file...";
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
            if (octomap_map_->size() == 0) {
                LOG(WARNING) << "\noctomap_map_ is empty, no map is saved";
                return;
            }
            pcl::io::savePCDFileBinary(folder_path + "/octomap_output.pcd", *octomap_map_);
        }
    }
}  // namespace octomap