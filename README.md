# l-vio

一个视觉SLAM算法，主要特点如下：

- Front-End: ORB & Optical flow
- Back-End: g2o

Front-end of this vio algorithm is based on ORB-feature and optical flow, but IMU preintegration is not integrated yet. The backend is based on g2o, and most of the code is based on ORB-SLAM3 implementation. I added Pangolin visualization code afterward, but it still cannot build successfully. There is not much information about this topic, so I will consider rewriting it with ROS.
基于ORB和光流法的vio-slam算法，但目前还没有整合IMU预积分。后端用的是g2o，大部分都参考了ORB-SLAM3的实现。添加Pangolin可视化代码后，还没有build成功，这方面的资料太少了，后面还是考虑用ROS进行重写。

## history

- 2024.02.27 项目重构

[![Top Langs](https://github-readme-stats.vercel.app/api/top-langs/?username=anuraghazra&hide=javascript,html)](https://github.com/Nothand0212/lvio)
