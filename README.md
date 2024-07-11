OctoMap for Dynamic Points Removal in Maps
---

Author & Maintainer: [Qingwen Zhang](https://kin-zhang.github.io/). Please give us a star if you like this repo! 🌟 and don't forget reference our [DynamicMap benchmark](https://github.com/KTH-RPL/DynamicMap_Benchmark) for more detail on datasets, evaluation and comparison with other methods.

📜 ChangeLog:
- 2024/07/11: Remove as much as possible dependencies in this repo and update voxel-level clean map saved.
- 2023/07/03: Update all config including octomap, octomap w G, octomap w GF.
- 2023/04/05: Add dynamic points removal in octomap, and update the README.

## 0. Setup

System I tested: Ubuntu 18.04, 20.04.

Dependencies: PCL [Read and write Point Cloud], glog & gflag [Debug Printing]. Normally you may have PCL installed because of others like ROS/ROS2 etc.

```bash
sudo apt update && sudo apt install -y libpcl-dev 
sudo apt install -y libgoogle-glog-dev libgflags-dev
```

git clone this repo:
```bash
git clone https://github.com/Kin-Zhang/octomap
```

## 1. Build & RUN

Build Command:
```bash
cd octomap
cmake -B build && cmake --build build
```

Prepare demo data [KITTI sequence 00 only 384.8MB], or create your own data following [these steps on Benchmark instruction](https://github.com/KTH-RPL/DynamicMap_Benchmark/blob/main/scripts/README.md#data-creation):
```bash
wget https://zenodo.org/records/10886629/files/00.zip
unzip 00.zip -d ${data_path, e.g. /home/kin/data}
```

Run command:
```bash
./build/octomap_run /home/kin/data/00 assets/config_fg.toml -1
```

- `-1` means all frames in the pcd folder, default is only 10 frame.

Demo result visualization in [CloudCompare]() on demo data (KITTI sequence 00):
- left is raw map, and red points are dynamic points labeled by SemanticKITTI (ground truth).
- medium is octomap_fg point-level output.
- right is octomap_gf voxel-level output. Since some mapper may directly want downsampled map. Voxel resolution is based on the config you set in `assets/config_fg.toml` like: `octomap/resolution=0.1` (10cm) and `output/downsampled=true` (voxel-level map).

![](assets/imgs/demo.png)


## 2. Evaluation

Please reference to [DynamicMap_Benchmark](https://github.com/KTH-RPL/DynamicMap_Benchmark) for the evaluation of this method and other dynamic removal methods.

[Evaluation Section link](https://github.com/KTH-RPL/DynamicMap_Benchmark/blob/master/scripts/README.md#evaluation)

## Acknowledgements & Cite

This implementation is based on codes from several repositories: [octomapping](https://github.com/OctoMap/octomap_mapping), [octomap](https://github.com/OctoMap/octomap). Thanks for their great work! ❤️

This work and the improved version is done during our DynamicMap benchmark. Please cite in following works if you use this code:

```
@inproceedings{zhang2023benchmark,
  author={Zhang, Qingwen and Duberg, Daniel and Geng, Ruoyu and Jia, Mingkai and Wang, Lujia and Jensfelt, Patric},
  booktitle={IEEE 26th International Conference on Intelligent Transportation Systems (ITSC)}, 
  title={A Dynamic Points Removal Benchmark in Point Cloud Maps}, 
  year={2023},
  pages={608-614},
  doi={10.1109/ITSC57777.2023.10422094}
}

@article{hornung13auro,
  author = {Armin Hornung and Kai M. Wurm and Maren Bennewitz and Cyrill Stachniss and Wolfram Burgard},
  title = {{OctoMap}: An Efficient Probabilistic {3D} Mapping Framework Based on Octrees},
  journal = {Autonomous Robots},
  year = 2013,
  volume = 34,
  pages = {189-206},
  doi = {10.1007/s10514-012-9321-0},
}
```