# quad_ws Development Environment

## 1. Prerequisites
Install the following on your local machine:
* [VS Code](https://code.visualstudio.com/)
* [Docker Desktop](https://www.docker.com/products/docker-desktop/) (or Docker Engine on Linux)
* **VS Code Extension:** [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

## 2. Linux Host Configuration
**Note:** These steps are required only on the **Host Machine** (your laptop/robot), not inside the container.

### Fix Docker Permissions
If you encounter permission errors accessing the Docker daemon:
```
sudo usermod -aG docker $USER
newgrp docker
```

### Setup NVIDIA Container Toolkit (x86 Linux)
Required for GPU acceleration (`--runtime=nvidia`).

**1. Add the package repositories:**
```
curl -fsSL [https://nvidia.github.io/libnvidia-container/gpgkey](https://nvidia.github.io/libnvidia-container/gpgkey) | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L [https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list](https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list) | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

**2. Install and configure:**
```
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

## 3. Launching the Environment
1.  Open the `quad_ws` folder in VS Code.
2.  Press `F1` (or `Ctrl+Shift+P`).
3.  Select **Dev Containers: Reopen in Container**.

## 4. Building the Project
Once inside the Dev Container terminal (`/workspace/quad_ws`):

```
# Build the workspace
colcon build --symlink-install

# Source the environment
source install/setup.bash
```

***Useful Commands and Urls***
```
docker system prune -a

cd src
git clone --recursive https://github.com/chvmp/champ -b ros2
git clone https://github.com/chvmp/champ_teleop -b ros2
cd ..
rosdep install --from-paths src --ignore-src -r -y

xacro src/robot_description/urdf/robot.xacro > src/robot_description/urdf/robot.urdf


```
https://github.com/chvmp/champ/tree/ros2
