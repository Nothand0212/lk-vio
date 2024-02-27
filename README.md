# lvio

一个视觉SLAM算法，主要特点如下：

- 1. 特征点：SuperPoint
- 2. 匹配方法：LightGlue
- 3. 优化方法：GTSAM

## history

- 2024.02.27 项目重构

## 依赖

- gtest
- opencv
- onnxruntime-gpu 1.16.0
- spdlog

## 参考

- [SuperPoint/LightGlue](https://github.com/cvg/LightGlue)
- [LightGlue-ONNX](https://github.com/fabio-sim/LightGlue-ONNX)
- [LightGlue-Onnx-cpp](https://github.com/Nothand0212/LightGlue-OnnxRunner-cpp): C++版本的LightGlue-ONNX运行器，支持Ubuntu
- [GTSAM](https://github.com/borglab/gtsam)