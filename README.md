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

## Installation, FAQ, and Contributing Instructions
Please see our instructions [here](https://robostack.github.io/GettingStarted.html).

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
     PACKAGE_XML_PATH=custom_packages/your_package/package.xml pixi run -e beta generate-custom-recipe
     ```

   - **Build additional recipes:**
     ```bash
     pixi run build_additional_recipes
     ```

   - **Build all packages:**
     ```bash
     pixi run -e beta build
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
   PACKAGE_XML_PATH=custom_packages/your_package/package.xml pixi run -e beta generate-custom-recipe
   ```

4. **Build packages:**
   ```bash
   pixi run build_additional_recipes
   # or for full build
   pixi run -e beta build
   ```

5. **Find built packages in `output/linux-64/`** (on the host machine)

### Troubleshooting

- **Container logs**: View logs with `docker compose logs robostack-dev`
- **Restart container**: `docker compose restart robostack-dev`
- **Clean restart**: `docker compose down && docker compose up -d robostack-dev`
