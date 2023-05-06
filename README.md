# WIP! klipper-fit-bed-mesh

## Installation

```
cd ~
git clone https://github.com/vertexbz/klipper-fit-bed-mesh.git
cd ~/klipper/klippy/extras/
ln -s ~/klipper-fit-bed-mesh/fit_bed_mesh.py .
```

### Moonraker
To add the extension to the update manager you can use following config
```
[update_manager fit_bed_mesh]
type: git_repo
path: ~/klipper-fit-bed-mesh
origin: https://github.com/vertexbz/klipper-fit-bed-mesh.git
primary_branch: master
is_system_service: False
```

## Setup
There are few prerequisites to have this extension working
- Your slicer has to emit object definitions

  For **SuperSlicer** go to `Print Settings` > `Output options`, section `Output file` and ensure `Label objects` checkbox is ticked. If you plan to use `FIT_BED_MESH` macro as part of `PRINT_START` macro - ensure `PRINT_START` is _second_ g-code of custom g-code block, this is due limitations of preprocessor.

- You need [bed_mesh](https://www.klipper3d.org/Config_Reference.html#bed_mesh) and [exclude_object](https://www.klipper3d.org/Config_Reference.html#exclude_object) set up in Klipper


## Configuration
Most of the configuration is inherited from `bed_mesh` section. To enable extension define `[fit_bed_mesh]` section.

```
[fit_bed_mesh]
#offset:
#   Offset defines how far from the bounding box of an object probing should take place
#   Default: 5.0
#density:
#   Desired probing point density, i.e. minimal distance between probing points
#   Can be affected by min_resolution. Default: 20.0
#min_resolution:
#   Minimal number of probing points in X / Y axis for an object. Default: 3
```


## Inspiration
This project is heavily inpsired by [Klipper Adaptive Meshing and Purging](https://github.com/kyleisah/Klipper-Adaptive-Meshing-Purging)

