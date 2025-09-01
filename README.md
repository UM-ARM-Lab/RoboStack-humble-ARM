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

## Building for RoboStack

To convert a ROS package to a Conda package, you need to:
1. Compile all necessary code to correct binaries for specific platforms
2. Link against correct libraries or package libraries together into the conda package  
3. Setup conda required metadata and structures

Instead of using `colcon build` like ROS2, this process uses `rattler-build`, which compiles source code in many languages to conda packages.

### Key Components

- **Metadata resolution**: Building packages requires knowing dependencies at both compile time and runtime, plus metadata like maintainer, package name, etc. This information is stored in a **conda recipe**. The RoboStack team created [vinca](https://github.com/RoboStack/vinca.git) to convert ROS `package.xml` files into conda recipes.

**NOTE**: `vinca` can be buggy and changes frequently. It's recommended to iteratively run the building procedure and debug any issues that arise.

- **Package compilation**: `rattler-build` handles building conda packages from recipes
- **Installing/Distributing**: Built packages are stored in `output/{platform}/{package}.conda` and can be installed in conda environments or distributed 

## Local Development with Pixi

This repository uses [pixi](https://pixi.sh/) for package management and building. Pixi provides a consistent development environment and is the official way to work with RoboStack packages.

### Prerequisites

- [Pixi](https://pixi.sh/) installed on your system
- Git (to clone this repository)

### Quick Start

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd RoboStack-humble-ARM
   ```

2. **Set up your custom packages:**
   Place your custom ROS packages in the `custom_packages/` directory.

3. **Generate recipes:**

   - **For all custom packages:**
     ```bash
     pixi run generate-recipes
     ```

   - **For a single custom package:**
     ```bash
     pixi run generate-recipes -p ./custom_packages/your_package_path
     ```

4. **Move generated files:**
   After generating recipes, you'll need to organize the files:
   ```bash
   # Create directory for your package in additional_recipes/
   mkdir -p additional_recipes/your-package-name/
   
   # Copy the generated build scripts (.sh files for Linux)
   cp *.sh additional_recipes/your-package-name/
   
   # Copy the generated recipe
   cp recipe.yaml additional_recipes/your-package-name/
   ```

5. **Build packages:**
   ```bash
   pixi run build
   ```

### Available Commands

- **`pixi run generate-recipes`**: Generate recipes from custom packages
- **`pixi run generate-recipes -p <path>`**: Generate recipe for a specific package
- **`pixi run build`**: Build all packages including generated recipes
- **`pixi run build-additional`**: Build only additional recipes (if available)

### Development Workflow

1. **Add your custom ROS packages** to the `custom_packages/` directory

2. **Generate recipes:**
   ```bash
   pixi run generate-recipes -p ./custom_packages/your_package
   ```

3. **Organize files:**
   Move the generated `recipe.yaml` and build scripts to `additional_recipes/your-package-name/`

4. **Fix the source path in the recipe:**
   Since you moved the `recipe.yaml` to a subdirectory, update the source path:
   ```yaml
   # Change from:
   source:
     path: custom_packages/your_package
   
   # To:
   source:
     path: ../../custom_packages/your_package
   ```

5. **Iteratively build and debug:**
   ```bash
   pixi run build
   ```
   Debug any issues from the output and adjust your recipe accordingly.

6. **Find built packages** in `output/linux-64/`

## Configuration

### Project Structure
- **`conda_build_config.yaml`**: Conda environment config (Python version, numpy version, etc.)
- **`rosdistro_snapshot.yaml`**: Metadata of packages in the ROS release channel
- **`packages-ignore.yaml`**, **`pkg_additional_info.yaml`**: Package filtering and metadata
- **`robostack.yaml`**: Maps ROS dependencies to system dependency names
- **`vinca_linux_64.yml`**: Vinca configuration for linux_64 platform

### Key Configuration Options
1. **Python version**: Specified in `conda_build_config.yaml`
2. **NumPy version**: Specified in `conda_build_config.yaml`
3. **System dependencies**: Configure ROS-to-system dependency mapping in `robostack.yaml`

## Troubleshooting

### Common Issues and Solutions

1. **Wrong build system detection**:
   Vinca may incorrectly detect whether a package is for ROS1 or ROS2, affecting whether `build_catkin` or `build_ament*` is used. If the generated `recipe.yaml` uses `build_catkin.sh`, replace it with:
   - `build_ament_cmake.sh` (for C++ packages)
   - `build_ament_python.sh` (for Python-only packages)

2. **Incorrect source paths**:
   When moving `recipe.yaml` to a dedicated folder in `additional_recipes/`, update the source path:
   
   **Example:**
   ```yaml
   # Generated by vinca (incorrect after moving):
   source:
     path: custom_packages/your_package
   
   # Corrected path in additional_recipes/your_package/recipe.yaml:
   source:
     path: ../../custom_packages/your_package
   ```

3. **Duplicate dependencies**:
   Remove duplicate entries in the `host` and `run` sections of `recipe.yaml`

4. **Missing dependencies**:
   Add required dependencies like `pip` to the `build` and `host` sections for Python packages

### Debugging Tips
- Run builds iteratively and examine error messages carefully
- Check that all required build scripts have execute permissions
- Verify that source paths are correct relative to the recipe location
- Ensure Python packages have the correct build system (`ament_python` vs `ament_cmake`)

### Build Order
Dependencies within the same directory may need to be built in a specific order. Consider building packages manually in the correct sequence if automatic dependency resolution fails.