# **DVL RViz Plugins**

A ROS 2 RViz2 plugin designed to visualize Doppler Velocity Log (DVL) data. This plugin specifically supports the [marine\_acoustic\_msgs/msg/Dvl](https://github.com/apl-ocean-engineering/marine_msgs/blob/ros2/marine_acoustic_msgs/msg/Dvl.msg) format, providing an intuitive way to monitor underwater vehicle velocity and beam data in a 3D environment.

## **Features**

* **Velocity Vector Visualization**: Renders the vehicle's velocity relative to the seafloor or water column.  
* **Beam Visualization**: Displays individual DVL beams (transducer directions) as lines or cones.  
* **Real-time Data Updates**: Subscribes to marine\_acoustic\_msgs/msg/Dvl topics and updates the display dynamically.  
* **Customizable Aesthetics**: Adjust colors, scale, and line thickness directly from the RViz display panel.

## **Prerequisites**

This package requires a ROS 2 installation (Humble, Iron, or Jazzy recommended) and the marine\_acoustic\_msgs package.

Install dependencies (replace \<ros-distro\> with your version, e.g., humble)  

```bash
sudo apt update  
sudo apt install ros-\<ros-distro\>-marine-acoustic-msgs
```

## **Installation**

1. **Clone the repository** into your ROS 2 workspace:  
   ```
   cd \~/ros2\_ws/src  
   git clone \[https://github.com/arihantb2/dvl\_rviz\_plugins.git\](https://github.com/arihantb2/dvl\_rviz\_plugins.git)
   ```

2. **Install dependencies** using rosdep:  
   ```
   cd \~/ros2\_ws  
   rosdep install \--from-paths src \--ignore-src \-r \-y
   ```

3. **Build the package**:  
   ```
   colcon build \--packages-select dvl\_rviz\_plugins
   ```

4. **Source the workspace**:  
   ```
   source install/setup.bash
   ```

## **Usage**

1. Launch RViz2:  
   ```
   rviz2
   ```

2. In the **Displays** panel, click **Add**.  
3. Under the **By display type** tab, look for dvl\_rviz\_plugins.  
4. Select the **DvlDisplay** (or similar) and click **OK**.  
5. Set the **Topic** to your DVL message topic (e.g., /dvl/data).  
6. Ensure your **Fixed Frame** is set correctly (e.g., base\_link or dvl\_link) to see the visualization.

## **Message Format**

The plugin subscribes to:

* **Type**: marine\_acoustic\_msgs/msg/Dvl  
* **Description**: This message includes velocity covariance, individual beam ranges, and velocities.

## **Contributing**

Contributions are welcome\! If you encounter bugs or have feature requests, please open an issue or submit a pull request.

1. Fork the Project  
2. Create your Feature Branch (`git checkout \-b feature/AmazingFeature`)  
3. Commit your Changes (`git commit \-m 'Add some AmazingFeature'`)  
4. Push to the Branch (`git push origin feature/AmazingFeature`)  
5. Open a Pull Request

## **License**

Distributed under the MIT License. See LICENSE for more information.

## **Author**

**Arihant Lunawat** \- [GitHub Profile](https://github.com/arihantb2)
