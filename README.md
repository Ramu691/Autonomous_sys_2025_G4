# Sub-Terrain Challenge: Autonomous Systems

## Installation

1. Start with a fresh Ubuntu 20.04 installation.
2. Install ROS Noetic: http://wiki.ros.org/noetic/Installation/Ubuntu, up to and including number 1.6:
    * Choose Desktop-Full Install in section 1.4
3. Install the following dependencies:
    ```bash
    sudo apt-get install ros-noetic-octomap-ros ros-noetic-octomap-server ros-noetic-octomap-rviz-plugins python3-catkin-tools git
    ```
    ```bash
    sudo apt-get install ros-noetic-ompl
    ```
4. Set up your SSH keys set up for Github. If you don't have them set up, follow these instructions:
    * To create an SSH
      keypair: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent
    * To add the SSH key to your Github
      account: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account
5. Clone this repository into a folder, we recommend using the `~/catkin_ws` folder as the root this repository.
    ```bash
    git clone git@github.com:Ramu691/Autonomous_sys_2025_G4.git ~/catkin_ws
    ```
   If you decide to clone the repository into a different folder, make sure to replace `~/catkin_ws` with the path to
   the folder you want to clone the repository into.
6. Navigate to the catkin workspace folder and compile the codebase:
    ```bash
   catkin build
     ```
7. Copy the two files and one folder for the simulation to `devel/lib/simulation` relative to the workspace root. It
   should then contain the following
   files:
    * `Simulation.x86_64`
    * `Simulation_Data`: a folder
    * `UnityPlayer.so`
    * `state_estimate_corruptor_node`: A symbolic link
    * `unity_ros`: A symbolic link
    * `w_to_unity`: A symbolic link
8. Make sure the `Car_build.x86_64` file is executable by running the following command in the workspace root:
    ```bash
    chmod +x ./devel/lib/simulation/Car_build.x86_64
    ```
9. Source the workspace:
    ```bash
    source devel/setup.bash
    ```
10. Run the following command to launch the entire simulation, including the autonomy pipeline:
    ```bash
    roslaunch simulation all.launch
    ```

# Additional packages installed

- ros-noetic-octomap-ros
- ros-noetic-octomap-server
- ros-noetic-octomap-rviz-plugins
- python3-catkin-tools
- git
