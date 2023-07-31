<div align="center">   
# Autonomous Manipulation
</div>


<h3 align="center">
  <a href="#">arXiv</a> |
  <a href="#">Video</a> |
  <a href="#">Slides</a>
</h3>


<br><br>

## Table of Contents:
1. [Highlights](#high)
2. [Getting Started](#start)
   - [Installation](docs/INSTALL.md)
   - [Starting UR5 Simulation](docs/UR5Sim.md)
   - [Starting UR5 Real Robot](docs/UR5Real.md)
   - [Callibration](docs/Callibration.md)
   - [Running](docs/Running.md)
4. [Results and Models](#models)
5. [TODO List](#todos)
6. [License](#license)


## Highlights <a name="high"></a>

- :oncoming_automobile: **Autonomous Manipulation**: Our work focuses on TBD ... 
- :trophy: **SOTA performance**: All tasks achieve SOTA performance .... 



## Getting Started <a name="start"></a>
   - [Installation](docs/INSTALL.md)
   - [Starting UR5 Simulation](docs/UR5Sim.md)
   - [Starting UR5 Real Robot](docs/UR5Real.md)
   - [Callibration](docs/Callibration.md)
   - [Running](docs/Running.md)

## Results and Pre-trained Models <a name="models"></a>
**To fix later**

### Stage1: Perception training
> We first train the perception modules (i.e., track and map) to obtain a stable weight initlization for the next stage. BEV features are aggregated with 5 frames (queue_length = 5).

| Method | Encoder | Tracking<br>AMOTA | Mapping<br>IoU-lane | config | Download |
| :---: | :---: | :---: | :---: | :---:|:---:| 
| UniAD-B | R101 | 0.390 | 0.297 |  [base-stage1](projects/configs/stage1_track_map/base_track_map.py) | [base-stage1](https://github.com/OpenDriveLab/UniAD/releases/download/v1.0/uniad_base_track_map.pth) |



### Stage2: End-to-end training
> We optimize all task modules together, including track, map, motion, occupancy and planning. BEV features are aggregated with 3 frames (queue_length = 3).

<!-- 
Pre-trained models and results under main metrics are provided below. We refer you to the [paper](https://arxiv.org/abs/2212.10156) for more details. -->

| Method | Encoder | Tracking<br>AMOTA | Mapping<br>IoU-lane | Motion<br>minADE |Occupancy<br>IoU-n. | Planning<br>avg.Col. | config | Download |
| :---: | :---: | :---: | :---: | :---:|:---:| :---: | :---: | :---: |
| UniAD-B | R101 | 0.358 | 0.317 | 0.709 | 64.1 | 0.25 |  [base-stage2](projects/configs/stage2_e2e/base_e2e.py) | [base-stage2](https://github.com/OpenDriveLab/UniAD/releases/download/v1.0/uniad_base_e2e.pth) |

### Checkpoint Usage
* Download the checkpoints you need into `UniAD/ckpts/` directory.
* You can evaluate these checkpoints to reproduce the results, following the `evaluation` section in [TRAIN_EVAL.md](docs/TRAIN_EVAL.md).
* You can also initialize your own model with the provided weights. Change the `load_from` field to `path/of/ckpt` in the config and follow the `train` section in [TRAIN_EVAL.md](docs/TRAIN_EVAL.md) to start training.


### Model Structure
The overall pipeline of UniAD is controlled by [uniad_e2e.py](projects/mmdet3d_plugin/uniad/detectors/uniad_e2e.py) which coordinates all the task modules in `UniAD/projects/mmdet3d_plugin/uniad/dense_heads`. If you are interested in the implementation of a specific task module, please refer to its corresponding file, e.g., [motion_head](projects/mmdet3d_plugin/uniad/dense_heads/motion_head.py).

## TODO List <a name="todos"></a>
**To be updated**
- [ ] All configs & checkpoints
- [ ] Upgrade the implementation of MapFormer from Panoptic SegFormer to [TopoNet](https://github.com/OpenDriveLab/TopoNet), which features the vectorized map representations and topology reasoning.
- [ ] Support larger batch size
- [ ] [Long-term] Improve flexibility for future extensions
- [x] Fix bug: Unable to reproduce the results of stage1 track-map model when training from scratch. [Ref: https://github.com/OpenDriveLab/UniAD/issues/21]
- [x] Visualization codes 
- [x] Separating BEV encoder and tracking module
- [x] Base-model configs & checkpoints
- [x] Code initialization


## License <a name="license"></a>

All assets and code are under the [Apache 2.0 license](./LICENSE) unless specified otherwise.

