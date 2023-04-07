/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Kin ZHANG (https://kin-zhang.github.io/)
 * @date: 2023-04-05 23:28
 * @details: No ROS version, speed up the process
 *
 * Input: PCD files + Prior raw global map , check our benchmark in dufomap
 * Output: Cleaned global map
 */

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "octomapper.h"
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_colorlogtostderr = true;
    google::SetStderrLogging(google::INFO);

    if (argc < 3) {
        LOG(ERROR) << "Usage: ./octomap_run [pcd_folder] [config_file]";
        return 0;
    }
    std::string pcd_parent = argv[1]; // we assume that rawmap is in pcd_parent;
    std::string config_file = argv[2];
    int cnt = 1, run_max = 1;
    // check if the config_file exists
    if (!std::filesystem::exists(config_file)) {
        LOG(ERROR) << "Config file does not exist: " << config_file;
        return 0;
    }

    octomap::MapUpdater map_updater(config_file);

    // load raw map
    std::string rawmap_path = pcd_parent + "/raw_map.pcd";
    pcl::PointCloud<PointType>::Ptr rawmap(new pcl::PointCloud<PointType>);
    pcl::io::loadPCDFile<PointType>(rawmap_path, *rawmap);
    LOG(INFO) << "Raw map loaded, size: " << rawmap->size();

    map_updater.setRawMap(rawmap);

    std::vector<std::string> filenames;
    for (const auto &entry : std::filesystem::directory_iterator(std::filesystem::path(pcd_parent) / "pcd")) {
        filenames.push_back(entry.path().string());
    }

    // sort the filenames
    std::sort(filenames.begin(), filenames.end());
    int total = filenames.size();
    if (argc > 3) {
        run_max = std::stoi(argv[3]);
        if (run_max == -1) {
            LOG(INFO) << "We will run all the frame in sequence, the total "
                         "number is: "
                      << total;
            run_max = total + 1;
        }
    }


    return 0;
}