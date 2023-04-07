/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Kin ZHANG (https://kin-zhang.github.io/)
 * @date: 2023-04-07 13:30
 * @details: No ROS version, speed up the process, check our benchmark in dufomap
 */
#pragma once

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "utils.h"

namespace octomap {
class MapUpdater {
  public:
    MapUpdater(const std::string &config_file_path);
    virtual ~MapUpdater() = default;
    void setRawMap(const pcl::PointCloud<PointType>::Ptr &rawmap);
    void setConfig();
    void run();
private:
    YAML::Node yconfig;
    };

}  // namespace octomap