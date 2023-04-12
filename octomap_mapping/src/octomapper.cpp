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

        // TODO in octomap: they fit a ground and nonground pointcloud
        // TODO only use nonground for // step B: free on ray, occupied on endpoint
        // TODO only use ground for // step A: free for ray and also endpoint
        timing.start("1. Ray SetFreeOc");

        // instead of direct scan insertion, compute update to filter ground:
        octomap::KeySet free_cells, occupied_cells;
        // step B: free on ray, occupied on endpoint
        for (pcl::PointCloud<PointType>::const_iterator it = single_pc->begin(); it != single_pc->end(); ++it){
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
                    LOG(WARNING) << "Cannot find the Key in octomap at: " << point;
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
            if (octomap_map_->size() == 0) {
                LOG(WARNING) << "\noctomap_map_ is empty, no map is saved";
                return;
            }
            pcl::io::savePCDFileBinary(folder_path + "/octomap_output_clean.pcd", *octomap_map_);
        }
        else{
            pcl::PointCloud<PointType>::Ptr octomap_map_(new pcl::PointCloud<PointType>);
            LOG(INFO) << "Saving octomap from octree to pcd file...";
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