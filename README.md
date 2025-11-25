Some common fixes and bugs when working with this code: 

Here is a summary of the common issues and fixes we encountered, formatted for your `README.md`.

### Troubleshooting

#### 1\. Simulation Hanging or "Entity Already Exists"

  * **Symptom:** Gazebo fails to launch, hangs indefinitely, or crashes with errors like `Entity [burger] already exists` or `gzserver process died`.
  * **Cause:** "Zombie" processes from a previous run are still active in the background, blocking new ones.
  * **Fix:** Force-kill all simulation and ROS processes.
    ```bash
    # Kill Simulator
    killall -9 gzserver gzclient

    # Kill all ROS nodes
    pkill -9 -f ros

    # Reset ROS Network Discovery
    ros2 daemon stop
    ros2 daemon start

    If this dosent work, straight up restart your PC
    ```

#### 2\. Corrupt Workspace (Old Code Running)

  * **Symptom:** You updated a Python script or launch file, but `ros2 run` still executes the old version, or you get `No executable found` errors.
  * **Cause:** The `install/` folder is out of sync with your `src/` folder.
  * **Fix:** Perform a "Deep Clean" and rebuild.
    ```bash
    cd ~/turtlebot3_ws

    # 1. Delete generated files
    rm -rf build/ install/ log/

    # 2. Rebuild from scratch
    colcon build --packages-select grazen_system --symlink-install

    # 3. Refresh your terminal
    source install/setup.bash
    ```

#### 3\. Network / DDS Errors ("Network Unreachable")

  * **Symptom:** Errors like `selected interface "lo" is not multicast-capable` or `Failed to find a free participant index`.
  * **Cause:** Conflict between Real Robot settings (Tailscale/VPN) and Simulation settings (Localhost).
  * **Fix:** Ensure you are in the correct mode.
      * **For Sim:** Unset custom DDS configs (`unset CYCLONEDDS_URI`) and ensure `ROS_LOCALHOST_ONLY=0` if using WiFi.
      * **For Real Robot:** Ensure you are connected to the robot's network/VPN.
      * **Universal Fix:** `ros2 daemon stop && ros2 daemon start`

#### 4\. TF / Map "Time Travel" Errors

  * **Symptom:** Logs spanning `Transform data too old` or `Lookup would require extrapolation`.
  * **Cause:** Nodes are mixing **System Time** (Real World) with **Simulation Time** (Gazebo).
  * **Fix:** Ensure `use_sim_time:=True` is passed to **every** node in the launch file (Nav2, SLAM, Robot State Publisher).
