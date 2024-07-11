/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * BSD 3-Clause License
 * @author Qingwen Zhang (https://kin-zhang.github.io/)
 * @date: 2023-04-07 13:30
 * @details: header file for octomap updater
 * 
 * This file is part of DynamicMap Benchmark work (https://github.com/KTH-RPL/DynamicMap_Benchmark).
 * If you find this repo helpful, please cite the respective publication as 
 * listed on the above website.
 * 
 */
#pragma once

#include <iostream>
// #include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "octomap/octomap.h"
#include "toml.hpp"
#include "timing.hpp"


// CHANGE Point Type Here!!! If you want to use XYZI, change to pcl::PointXYZI
// typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZ PointType;
typedef octomap::OcTree OcTreeT;

// typedef pcl::PointXYZRGB PointType;
// typedef octomap::ColorOcTree OcTreeT;

#define ANSI_RED "\033[1m\x1b[31m"
#define ANSI_GREEN "\033[1m\x1b[32m"
#define ANSI_YELLOW "\033[1m\x1b[33m"
#define ANSI_BLUE "\033[1m\x1b[34m"
#define ANSI_MAGENTA "\033[1m\x1b[35m"
#define ANSI_CYAN "\033[1m\x1b[36m"
#define ANSI_RESET "\x1b[0m"
#define ANSI_BOLD "\033[1m"

// since C++17 we use, C++20 we can use std::remove_cvref_t
template <typename T>
using remove_cvref_t = typename std::remove_cv<typename std::remove_reference<T>::type>::type;

namespace common {
struct Config {

  // octomap params, default from octomap_mapping repo
  float m_res = 0.1;  // resolution of the map
  float probHit = 0.7;
  float probMiss = 0.4;
  float thresMin = 0.12;
  float thresMax = 0.97;
  bool m_prune = true;  // prune the tree after insertion of new data

  // range, // -1 means no range limit
  float m_maxRange = -1.0;
  float m_minRange = -1.0;


  bool verbose_ = false;  // print out logs
  bool replace_intensity = false;

  // SAC segmentation params ground plane
  bool filterGroundPlane = false;
  float m_groundFilterDistance = 0.04;
  float m_groundFilterAngle = 0.15;
  float m_groundFilterPlaneDistance = 0.07;

  // filter noise
  bool filterNoise = false;
  float StddevMulThresh = 1.0;
  int filterMeanK = 50;

  bool output_downsampled_map = false;
  std::string filename = "octomapfg";

	void readtoml(toml::table tbl){
    m_res = read(tbl["octomap"]["resolution"], m_res);
    m_maxRange = read(tbl["octomap"]["maxRange"], m_maxRange);
    m_minRange = read(tbl["octomap"]["minRange"], m_minRange);

    probHit = read(tbl["octomap"]["probHit"], probHit);
    probMiss = read(tbl["octomap"]["probMiss"], probMiss);
    thresMin = read(tbl["octomap"]["thresMin"], thresMin);
    thresMax = read(tbl["octomap"]["thresMax"], thresMax);
    m_prune = read(tbl["octomap"]["pruneTree"], m_prune);

    filterGroundPlane = read(tbl["ground"]["enable"], filterGroundPlane);
    m_groundFilterDistance = read(tbl["ground"]["distance"], m_groundFilterDistance);
    m_groundFilterAngle = read(tbl["ground"]["angle"], m_groundFilterAngle);
    m_groundFilterPlaneDistance = read(tbl["ground"]["planeDistance"], m_groundFilterPlaneDistance);

    filterNoise = read(tbl["filter"]["enable"], filterNoise);
    StddevMulThresh = read(tbl["filter"]["filterMeanK"], StddevMulThresh);
    filterMeanK = read(tbl["filter"]["StddevMulThresh"], filterMeanK);

    output_downsampled_map = read(tbl["output"]["downsampled"], output_downsampled_map);
    filename = read(tbl["output"]["filename"], filename);
	}


 private:
	template <typename T>
	remove_cvref_t<T> read(toml::node_view<toml::node> node, T&& default_value){
		return node.value_or(default_value);
	}
};

}  // namespace common

namespace octomap {
class MapUpdater {
public:
  MapUpdater(const std::string &config_file_path);
  virtual ~MapUpdater() = default;
  void run(pcl::PointCloud<PointType>::Ptr const& single_pc);
  std::string saveMap(std::string const& folder_path);
  const common::Config getCfg() { return cfg_; }
  Timing timing;

private:
    
    common::Config cfg_;
    OcTreeT* m_octree;
    octomap::KeyRay m_keyRay;  // temp storage for ray casting
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;
    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;
    pcl::PointCloud<PointType>::Ptr ground_pts;
    pcl::PointCloud<PointType>::Ptr raw_map_ptr_;
    void filterGroundPlane(pcl::PointCloud<PointType>::Ptr const& pc, 
                           pcl::PointCloud<PointType>::Ptr &ground, 
                           pcl::PointCloud<PointType>::Ptr &nonground);
    pcl::PointCloud<PointType>::Ptr noise_cloud;

protected:
    inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
      for (unsigned i = 0; i < 3; ++i)
        min[i] = std::min(in[i], min[i]);
    };

    inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
      for (unsigned i = 0; i < 3; ++i)
        max[i] = std::max(in[i], max[i]);
    };

};

}  // namespace octomap