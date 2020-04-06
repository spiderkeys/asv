# Preparing your Jetson Nano image

- Create a Jetson Nano SD card that has the Jetson SDK installed:
  - https://docs.nvidia.com/jetson/jetpack/install-jetpack/index.html

- Install the ZED SDK
  - Download the SDK release that matches the Jetpack version you installed on your Nano:
    - https://www.stereolabs.com/developers/release/
  - Either directly download or copy the file to your Jetson Nano and install by running:
    ```bash
    # Filename may be different, depending on Jetpack version
    chmod +x ZED_SDK_Tegra_JP43_v3.1.1.run
    ./ZED_SDK_Tegra_JP43_v3.1.1.run
    ```

- Install the following general purpose dependencies on the Nano:
    ```bash
    # Note: Some of these are not strictly necessary, but match what was used in the Docker image for the first Cybie prototype
    sudo apt-get update -y
    sudo apt-get install -y --no-install-recommends \
        lsb-release \
        wget \
        less \
        udev \
        sudo \
        nano \
        vim \
        apt-transport-https \
        build-essential \
        cmake \
        curl \
        gnupg2 \
        apt-transport-https \
        apt-utils \
        supervisor \
        openssh-server \
        git \
        libxml2-dev \
        libxslt1-dev \
        libv4l-dev \
        python3-pip \
        g++ \
        gcc
    ```

- Install ROS2 Dashing:
    ```bash
    # Set up apt sources list
    curl http://repo.ros2.org/repos.key | sudo apt-key add - && \
    sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

    # Install
    sudo apt-get update -y
    sudo apt-get install -y --no-install-recommends \
        ros-dashing-ros-base \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool 
    ```
- Install and configure Conan (C/C++ package manager)
    ```bash
    # Install
    pip3 install conan --user

    # Configure
    conan remote add mr https://api.bintray.com/conan/missionrobotics/missionrobotics
    echo "[settings]\n\
    os=Linux\n\
    os_build=Linux\n\
    arch=armv8\n\
    arch_build=armv8\n\
    compiler=gcc\n\
    compiler.version=7\n\
    compiler.libcxx=libstdc++11\n\
    build_type=Release\n\
    [options]\n\
    [build_requires]\n\
    [env]" > ~/.conan/profiles/default
    ```

- Install Python packages
    ```bash
    pip3 install -U argcomplete mavproxy Jetson.GPIO bluerobotics-ping
    ```

- Create MR directory
    ```bash
    sudo mkdir /opt/mr
    sudo chown -R ${USER}:${USER} /opt/mr
    ```
# Cloning and building the software

- Clone the ASV repo and copy the files to their designated locations
    ```bash
    git clone https://gitlab.com/missionrobotics/customer/ocean-systems/asv-nano-software /tmp/
    mv /tmp/asv-nano-software/conan-packages /opt/mr/
    mv /tmp/asv-nano-software/ros2_ws /opt/mr/
    mv /tmp/asv-nano-software/supervisord.conf /opt/mr/
    rm -rf /tmp/asv-nano-software
    ```

- Build the conan packages:
    ```bash
    cd /opt/mr/conan-packages
    ./build.sh
    ```

- Build the ROS2 Workspace:
    ```bash
    cd /opt/mr/ros2_ws
    ./build.sh
    ```
# Summary of libraries and applications
- Conan library packages (/opt/mr/conan-packages)
  - `bzip2`: requirement for the Boost library (used by C++ apps)
  - `concurrentqueue`: threadsafe lockfree queue used by mavchannel
  - `readerwriterqueue`: threadsafe lockfree queue used by mavchannel
  - `serial`: serial port library used by mavchannel
  - `mavchannel`: small library that combines serial, mavlink, and the threadsafe queues to provide a serial interface for devices that speak mavlink
  - `mavlink2`: packaged version of the official mavlink repository. Mavlink is a communication protocol traditionally used within the Ardupilot/Ardusub space.

- ROS2 applications (/opt/mr/ros2_ws)
  - C++
    - `asv_bridge`: application that bridges communications between Ardusub (running on the Pixhawk and speaking mavlink) and the ROS2 databus
    - `recorder`: application that subscribes to telemetry and video data and writes them to disk. Also publishes compressed versions of the left camera and depth image from the ZED2 camera for other applications to subscribe to.
  - Python
    - `mockbot`: application that can be used to (poorly) simulate some of the data produced by a pixhawk connected through the asv_bridge. Useful for testing the recorder node without actually having a pixhawk connected.
    - `pinger`: application that connects to the Blue Robotics Ping1D sensor over serial to get depth soundings
    - `status`: application that subscribes to the state of the recording node and lights an LED on pin 7 of the nano if it is active

# Configuration:
- In supervisord.conf, look for the line that says:
    ```
    environment=RECORDER_DATA_DIR="/tmp"
    ```
  - RECORDER_DATA_DIR is an environment variable used by `recorder` as the destination for recorded image/data files. The default value is /tmp. You will want to change the path specified here to wherever you would like recorded data to be written to. 
  - It is likely that you will want to write data to an external hard drive or flash drive, so insert that drive, find out it's path, and set that value for the RECORDER_DATA_DIR variable.

# I/O Connections
 - The Pixhawk should be connected via USB to the Jetson Nano
 - The Pixhawk should also be connected to the Nano via the Pixhawk's telem2 channel to the Nano's primary serial header pins (RXD/TXD, pins 10 and 8 respectively). Also be sure to connect to ground on the Nano.
 - The status LED GPIO should be connected to pin 7 and a ground connection on the Nano.
 - The Blue Robotics Ping1D should be connected to the Nano via USB
 
# Running the vehicle software
 - Run the vehicle software with the command:
    ```bash
    # Start the daemon
    supervisord -c /opt/mr/supervisord.conf
    ```
 - You can create an init script (using any init system of your choosing, but most likely systemd) to automatically start the supervisord daemon on boot.
 - You can start/stop/restart applications using commands like:
    ```bash
    # Start all apps
    supervisorctl -c /opt/mr/supervisord.conf start
    
    # Stop all apps
    supervisorctl -c /opt/mr/supervisord.conf start

    # Restart a specific app (like recorder)
    supervisorctl -c /opt/mr/supervisord.conf restart recorder
    ```
# Using ROS2 tools
 - You can use the ROS2 command line utilities to interact with the applications in this project:
    ```bash
    # Source the ROS2 Underlay
    source /opt/ros/dashing/setup.bash

    # Subscribe and print the GPS messages coming from asv_bridge
    ros2 topic echo "gps"

    # Trigger and end a recording session
    # Either from your laptop within the ros2 container, or after ssh'ing into the vehicle:
    ros2 topic pub "enable" std_msgs/msg/Bool "data: True"
    # Kill the above with ctrl-c

    # Stop the recording session:
    ros2 topic pub "enable" std_msgs/msg/Bool "data: False"
    # Kill the above with ctrl-c
    ```
 - See the official ROS2 docs for more info about what tools are available

# General workflow/usage

NOTE: The following assume you have started the software stack on the vehicle, as described above

On the Host Laptop, check to make sure that the system is generally operational:
 1. Connect via WiFi to the vehicle's access point
 2. Open QGroundControl and check for connectivity, sensor data, ability to control, etc
 3. SSH into vehicle
 4. `ls /media/ext/` should show at least "lost+found" if the USB drive is mounted. There may also be previous recordings in here.
 5. `supervisorctl -c /opt/mr/supervisord.conf status` should show all applications running
 6. Check for data coming from the pixhawk:
    ```bash
    source /opt/ros/dashing/setup.bash
    ros2 topic echo attitude
    ```
 When ready to record:
 1. SSH into vehicle
 2. Check for GPS
    ```bash
    source /opt/ros/dashing/setup.bash
    ros2 topic echo gps
    # GPS is good when "status" is 0 or greater. -1 means no fix
    ```
 3. Start recording
    ```bash
    source /opt/ros/dashing/setup.bash
    ros2 topic pub enable std_msgs/msg/Bool "data: True"
    ```
 4. Check that files are being recorded:
    ```bash
    ls -al /media/ext/
    # Recordings should start populating the directory if recording is active
    ```
 5. Once you confirm files are recording, you can run the `showimage` program on your laptop (from the asv-desktop-software project) to see the stream
 6. Now, send the vehicle on its mission with QGC
 7. Once the mission is complete, stop recording:
    ```bash
    source /opt/ros/dashing/setup.bash
    ros2 topic pub enable std_msgs/msg/Bool "data: False"
    ```

# Displaying live streams of camera topics:
From host laptop inside ros2-playground in the ROS2 container:

```bash
# Build and source the workspace:
cd ros2_ws
./build.sh
source ./install/setup.bash

# Run the viewer application (takes a few seconds to bring the window up)
ros2 run showimage showimage --topic left
#or
ros2 run showimage showimage --topic depth
```

