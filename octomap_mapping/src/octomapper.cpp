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

        // init the map
        // map_arranged_.reset(new pcl::PointCloud<PointType>());
        // map_cleaned_.reset(new pcl::PointCloud<PointType>());
        // map_dynamic_.reset(new pcl::PointCloud<PointType>());
    }
    void MapUpdater::setConfig(){
        // TODO
    }
    void MapUpdater::setRawMap(const pcl::PointCloud<PointType>::Ptr &rawmap){
        // TODO
    }


    void MapUpdater::run(){
        // TODO
    }
}  // namespace octomap