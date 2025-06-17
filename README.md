# RoboStack (for ROS humble)

[![Conda](https://img.shields.io/conda/dn/robostack-humble/ros-humble-desktop?style=flat-square)](https://anaconda.org/robostack/)
[![Gitter](https://img.shields.io/gitter/room/RoboStack/Lobby?style=flat-square)](https://gitter.im/RoboStack/Lobby)
[![GitHub Repo stars](https://img.shields.io/github/stars/robostack/ros-humble?style=flat-square)](https://github.com/RoboStack/ros-humble/)
[![QUT Centre for Robotics](https://img.shields.io/badge/collection-QUT%20Robotics-%23043d71?style=flat-square)](https://qcr.github.io/)

[![Platforms](https://img.shields.io/badge/platforms-linux%20%7C%20win%20%7C%20macos%20%7C%20macos_arm64%20%7C%20linux_aarch64-green.svg?style=flat-square)](https://github.com/RoboStack/ros-humble)
[![Azure DevOps builds (branch)](https://img.shields.io/github/actions/workflow/status/robostack/ros-humble/linux.yml?branch=buildbranch_linux&label=build%20linux&style=flat-square)](https://github.com/RoboStack/ros-humble/actions/workflows/linux.yml)
[![Azure DevOps builds (branch)](https://img.shields.io/github/actions/workflow/status/robostack/ros-humble/win.yml?branch=buildbranch_win&label=build%20win&style=flat-square)](https://github.com/RoboStack/ros-humble/actions/workflows/win.yml)
[![Azure DevOps builds (branch)](https://img.shields.io/github/actions/workflow/status/robostack/ros-humble/osx.yml?branch=buildbranch_osx&label=build%20osx&style=flat-square)](https://github.com/RoboStack/ros-humble/actions/workflows/osx.yml)
[![Azure DevOps builds (branch)](https://img.shields.io/github/actions/workflow/status/robostack/ros-humble/osx_arm64.yml?branch=buildbranch_osx_arm64&label=build%20osx-arm64&style=flat-square)](https://github.com/RoboStack/ros-humble/actions/workflows/osx_arm64.yml)
[![Azure DevOps builds (branch)](https://img.shields.io/github/actions/workflow/status/robostack/ros-humble/build_linux_aarch64.yml?branch=buildbranch_linux_aarch64&label=build%20aarch64&style=flat-square)](https://github.com/RoboStack/ros-humble/actions/workflows/build_linux_aarch64.yml)

[![GitHub issues](https://img.shields.io/github/issues-raw/robostack/ros-humble?style=flat-square)](https://github.com/RoboStack/ros-humble/issues)
[![GitHub closed issues](https://img.shields.io/github/issues-closed-raw/robostack/ros-humble?style=flat-square)](https://github.com/RoboStack/ros-humble/issues?q=is%3Aissue+is%3Aclosed)
[![GitHub pull requests](https://img.shields.io/github/issues-pr-raw/robostack/ros-humble?style=flat-square)](https://github.com/RoboStack/ros-humble/pulls)
[![GitHub closed pull requests](https://img.shields.io/github/issues-pr-closed-raw/robostack/ros-humble?style=flat-square)](https://github.com/RoboStack/ros-humble/pulls?q=is%3Apr+is%3Aclosed)

[__Table with all available packages & architectures__](https://robostack.github.io/humble.html)

## Why ROS and Conda?
Welcome to RoboStack, which tightly couples ROS with Conda, a cross-platform, language-agnostic package manager. We provide ROS binaries for Linux, macOS, Windows and ARM (Linux). Installing other recent packages via conda-forge side-by-side works easily, e.g. you can install TensorFlow/PyTorch in the same environment as ROS humble without any issues. As no system libraries are used, you can also easily install ROS humble on any recent Linux Distribution - including older versions of Ubuntu. As the packages are pre-built, it saves you from compiling from source, which is especially helpful on macOS and Windows. No root access is required, all packages live in your home directory. We have recently written up a [paper](https://arxiv.org/abs/2104.12910) and [blog post](https://medium.com/robostack/cross-platform-conda-packages-for-ros-fa1974fd1de3) with more information.

## Attribution
If you use RoboStack in your academic work, please refer to the following paper:
```bibtex
@article{FischerRAM2021,
    title={A RoboStack Tutorial: Using the Robot Operating System Alongside the Conda and Jupyter Data Science Ecosystems},
    author={Tobias Fischer and Wolf Vollprecht and Silvio Traversaro and Sean Yen and Carlos Herrero and Michael Milford},
    journal={IEEE Robotics and Automation Magazine},
    year={2021},
    doi={10.1109/MRA.2021.3128367},
}
```

## Building for Robostack

Documentation for this procedure is very lacky, so the following is the best efforts I tried to pull together:

Generally, to convert a ROS package to Conda package, you need to 1. compile all necessary code to correct binaries for specific platforms, 2. link aginst correct libraries or package libraries together into the conda package, and 3. setup conda required metadata or structures. To do this process, instead of using `colcon build` like ROS2, the procedure works around `rattler-build`, which is a tool that compiles source code in many languages to conda packages. 

- **Metadata resolution**: building package like this requires knowing the dependencies of this package in both compile time and runtime. It also requires metadata such as maintainer, package name, etc. Additionally, the builder needs to know any specifics of how this package should be compiled. This is known as a **conda recipe**. This procedure is more or less the same for ROS packages, which is why the RoboStack team wrote [vinca](https://github.com/RoboStack/vinca.git) tool for converting the `package.xml` file of a ROS package into a 

**NOTE**: `vinca` is buggy and changes frequently, according to the RoboStack team, so developements of this repo should strictly follow official documentation. It is recommended to iteratively run the building procedure and debug any issues.

- **Package compilation**: `rattler-build` handles building conda packages from recipes, which is pretty stable during testing. 
- **Installing/Distributing**: Built packages are stored in output/{platform}/{package}.conda. Now you may install this `.conda` file in your conda environment or distribute to your team. 

## Docker-based Building (Interactive Development)

This repository includes Docker Compose configuration for interactive development of RoboStack packages using the prebuilt pixi container. This approach provides a persistent development environment where you can run commands manually.

### Prerequisites

- Docker and Docker Compose installed on your system
- Git (to clone this repository)

### Quick Start

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd RoboStack-humble-ARM
   ```

2. **Start the development container:**
   ```bash
   docker compose up -d robostack-dev
   ```

3. **Enter the container:**
   ```bash
   docker compose exec robostack-dev bash
   # or
   docker exec -it robostack-dev bash
   ```

4. **Inside the container, you can now run pixi commands:**

   - **Set up custom packages (if needed):**
     Place your custom ROS packages in the `custom_packages/` directory on the host. They will be automatically available in the container.

   - **Generate recipes for all custom packages:**
     ```bash
     pixi run -e beta generate-recipes
     ```

   - **Generate a recipe for a single custom package:**
     ```bash
     PACKAGE_XML_PATH=custom_packages/your_package/package.xml 
     pixi run -e beta generate-custom-recipe
     ```

   - **Build all additional recipes:**
     ```bash
     pixi run build_additional_recipes
     ```

### Container Management

- **Start the container:** `docker compose up -d robostack-dev`
- **Enter the container:** `docker compose exec robostack-dev bash`
- **Stop the container:** `docker compose down`
- **View logs:** `docker compose logs robostack-dev`

### Output

Built packages will be available in the `output/linux-64/` directory on your host machine.

### Available Commands (inside the container)

- **`pixi run build_additional_recipes`**: Build only additional recipes
- **`pixi run -e beta generate-recipes`**: Generate recipes from all custom packages  
- **`pixi run -e beta generate-custom-recipe`**: Generate a recipe for a single custom package (set `PACKAGE_XML_PATH`)
- **`pixi run -e beta build`**: Build all packages including generated recipes

### Development Workflow

1. **Start the development environment:**
   ```bash
   docker compose up -d robostack-dev
   docker compose exec robostack-dev bash
   ```

2. **Add your custom ROS packages to `custom_packages/`** (on the host machine)

3. **Inside the container, generate recipes:**
   ```bash
   # For all packages
   pixi run -e beta generate-recipes
   
   # For a single package
   PACKAGE_XML_PATH=custom_packages/your_package/package.xml
   pixi run -e beta generate-custom-recipe
   ```

4. **Move files**
    
    All custom packages should have a dedicated folder in additional_recipes like the example. The previous command should have generated a set of `.sh, .bat, .pz1` files. For linux-64, you only need the `.sh` files. Copy those files to the folder, and copy over the generated recipe.yaml as well. 
    ```bash
    cp *.sh additional_recipes/<package_name>/
    cp recipe.yaml additional_recipes/<package_name>/
    ```

4. **Iteratively Build Packages:**
   ```bash
   pixi run build_additional_recipes
   # or for full build
   pixi run -e beta build
   ```
   Debug any issues from the output, and adjust your recipe accordingly. 

5. **Find built packages in `output/linux-64/`** (on the host machine)

## Configure Your Build

**Project Structure**
- `conda_build_config.yaml` hosts conda environment config that packages will be built in, such as numpy version, python version, etc. 
- `rosdistro_snapshot.yaml` hosts metadata of packages in the ROS release channel. This is used for 
- `packages-ignore.yaml`, `pkg_additional_info.yaml`: unknown, not really configurable.
- `robostack.yaml` maps ROS dependencies to system dependency names. If your package uses a system dependency, you should configure it here.
- `vinca_linux_64.yml` specifies the vinca information for linux_64 platform. We don't need to support platforms other than linux x64, so that is the only one. The official repository contains multiple versions for different OS platforms. Usually this is where which packages gets built are specified, but since we are not building robostack from scratch, we don't need to configure this. 


**Configurable Fields**
1. **Python vesrion** is specified in `conda_build_config.yaml`
2. **numpy version** is specified in `conda_build_config.yaml`
3. 

## Troubleshooting

**General Tips**
- **Container logs**: View logs with `docker compose logs robostack-dev`
- **Restart container**: `docker compose restart robostack-dev`
- **Clean restart**: `docker compose down && docker compose up -d robostack-dev`


### Known Issues

1. Vinca is not very good at detecting if pacakge version is for ROS1 or ROS2, this decides whether `build_catkin` or `build_ament*` is used. If the generated `recipe.yaml` says using `build_catkin.sh`, replace those with `build_ament_cmake.sh` (for cpp packages) or `build_ament_python.sh` (for python-only pakcages) if your package is ROS2. To be 100% sure, delete the scripts that use catkin. 

2. The source path in a `vinca` generated recipe is relative to the generated `recipe.yaml` file. Since we move that file to a dedicated folder, this relative path changes. Therefore, for this data structure, you should change the path to include `../../`

   **Example folder structure:**
   ```
   RoboStack-humble-ARM/
   ├── custom_packages/
   │   └── your_package/
   │       └── package.xml
   ├── additional_recipes/
   │   └── your_package/
   │       ├── recipe.yaml (moved here)
   │       └── build_ament_cmake.sh
   └── recipe.yaml (generated here by vinca)
   ```
   
   **Path correction needed:**
   ```yaml
   # Generated by vinca (incorrect after moving):
   source:
     - path: custom_packages/your_package
   
   # Corrected path in additional_recipes/your_package/recipe.yaml:
   source:
     - path: ../../custom_packages/your_package
   ```

3. It is unknown how rattler-build handles dependencies in the same directory. Building is generally okay, but you may want to build things in a particular order manually.