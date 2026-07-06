#!/bin/bash
# =============================================================================
#  DRONE SIMULATION LAUNCHER
#  File: launch_sim.sh
# =============================================================================
#
#  WHAT THIS DOES:
#    Opens 3 separate terminal windows and runs the full drone simulation
#    stack automatically — no more copy-pasting commands one by one!
#
#  USAGE:
#    chmod +x ~/Desktop/Drone_IP/launch_sim.sh   (only needed once)
#    ~/Desktop/Drone_IP/launch_sim.sh             (run it!)
#
#  OPTIONAL ARGUMENTS:
#    --auto     Launches the sensor-based autonomous navigator instead
#               of the keyboard controller.
#    --algo X   Choose the path planning algorithm (default: astar)
#               Options: astar, prm, rrt, theta, dstar
#
#  WHAT GETS LAUNCHED:
#    Terminal 1: Micro-XRCE-DDS Agent   (bridge between PX4 ↔ ROS 2)
#    Terminal 2: PX4 SITL + Gazebo      (flight controller + 3D simulator)
#    Terminal 3: Keyboard Controller    (your WASD drone controls)
#               OR Waypoint Navigation  (if --auto flag is used)
#
# =============================================================================

# --- Cleanup: Kill leftover processes from previous runs ---------------------
# This fixes the "Gazebo won't open" issue caused by stale processes.
echo "🧹 Cleaning up any leftover processes from previous runs..."
killall -q MicroXRCEAgent 2>/dev/null || true
killall -q px4 2>/dev/null || true
killall -q ruby 2>/dev/null || true          # PX4 SITL uses ruby internally
pkill -f "gz sim" 2>/dev/null || true        # Gazebo simulator
pkill -f "parameter_bridge" 2>/dev/null || true  # ros_gz_bridge if running
sleep 2  # Give processes time to fully die
echo "   ✅ Cleanup done"
echo ""

# --- Configuration -----------------------------------------------------------
# Change these if your installation paths are different.

DDS_AGENT_DIR="$HOME/Micro-XRCE-DDS-Agent/build"
PX4_DIR="$HOME/PX4-Autopilot"
ROS2_WS="$HOME/px4_ros_ws"
WORLD_NAME="house_3room"
SDF_SOURCE="$HOME/Desktop/Drone_IP/worlds/house_3room.sdf"
SDF_DEST="$PX4_DIR/Tools/simulation/gz/worlds/house_3room.sdf"

# Scripts
KEYBOARD_SCRIPT="$HOME/Desktop/Drone_IP/core/keyboard_control.py"

# --- Parse arguments ---------------------------------------------------------
AUTO_MODE=false
ALGO="astar"

for arg in "$@"; do
    case "$arg" in
        --auto) AUTO_MODE=true ;;
        --algo)
            # Next arg will be the algorithm name
            ;;
        astar|prm|rrt|theta|dstar|dijkstra|bellman|benchmark)
            ALGO="$arg" ;;
    esac
done

# Handle --algo <name> (two-word argument)
for i in $(seq 1 $#); do
    arg="${!i}"
    if [[ "$arg" == "--algo" ]]; then
        next=$((i + 1))
        ALGO="${!next}"
    fi
done

# --- Determine which controller to use ---------------------------------------
CONTROLLER_SCRIPT="$KEYBOARD_SCRIPT"
CONTROLLER_NAME="Keyboard Controller"

if [[ "$AUTO_MODE" == "true" ]]; then
    case "$ALGO" in
        astar)
            CONTROLLER_SCRIPT="$HOME/Desktop/Drone_IP/planners/navigator_astar.py"
            CONTROLLER_NAME="Autonomous Navigator (A* Path Planner)"
            ;;
        prm)
            CONTROLLER_SCRIPT="$HOME/Desktop/Drone_IP/planners/navigator_prm.py"
            CONTROLLER_NAME="Autonomous Navigator (PRM Planner)"
            ;;
        rrt)
            CONTROLLER_SCRIPT="$HOME/Desktop/Drone_IP/planners/navigator_rrt.py"
            CONTROLLER_NAME="Autonomous Navigator (RRT Planner)"
            ;;
        theta)
            CONTROLLER_SCRIPT="$HOME/Desktop/Drone_IP/planners/navigator_theta_star.py"
            CONTROLLER_NAME="Autonomous Navigator (Theta* Planner)"
            ;;
        dstar)
            CONTROLLER_SCRIPT="$HOME/Desktop/Drone_IP/planners/navigator_dstar_lite.py"
            CONTROLLER_NAME="Autonomous Navigator (D* Lite Planner)"
            ;;
        dijkstra)
            CONTROLLER_SCRIPT="$HOME/Desktop/Drone_IP/planners/navigator_dijkstra.py"
            CONTROLLER_NAME="Autonomous Navigator (Dijkstra Planner)"
            ;;
        bellman)
            CONTROLLER_SCRIPT="$HOME/Desktop/Drone_IP/planners/navigator_bellman_ford.py"
            CONTROLLER_NAME="Autonomous Navigator (Bellman Ford Planner)"
            ;;
        benchmark)
            CONTROLLER_SCRIPT="$HOME/Desktop/Drone_IP/planners/navigator_benchmark.py"
            CONTROLLER_NAME="Visual Benchmark (All 6 Algorithms)"
            ;;
        *)
            echo "Unknown algorithm: $ALGO"
            echo "Options: astar, prm, rrt, theta, dijkstra, bellman, benchmark"
            exit 1
            ;;
    esac
fi

# --- Detect terminal emulator ------------------------------------------------
# Try common terminals in order of popularity
detect_terminal() {
    if command -v gnome-terminal &> /dev/null; then
        echo "gnome-terminal"
    elif command -v xfce4-terminal &> /dev/null; then
        echo "xfce4-terminal"
    elif command -v konsole &> /dev/null; then
        echo "konsole"
    elif command -v xterm &> /dev/null; then
        echo "xterm"
    else
        echo ""
    fi
}

TERMINAL=$(detect_terminal)

if [[ -z "$TERMINAL" ]]; then
    echo "❌ ERROR: No supported terminal emulator found!"
    echo "   Install one of: gnome-terminal, xfce4-terminal, konsole, xterm"
    exit 1
fi

echo "=============================================="
echo "  🚁 Drone Simulation Launcher"
echo "=============================================="
echo ""
echo "  Terminal emulator : $TERMINAL"
echo "  World             : $WORLD_NAME"
echo "  Controller        : $CONTROLLER_NAME"
echo ""

# --- Step 0: Copy updated SDF to PX4 worlds ----------------------------------
echo "📋 Copying latest SDF to PX4 worlds directory..."
cp "$SDF_SOURCE" "$SDF_DEST"
echo "   ✅ Copied $SDF_SOURCE → $SDF_DEST"
echo ""

# --- Helper: open a terminal with a title and command -------------------------
open_terminal() {
    local title="$1"
    local cmd="$2"

    case "$TERMINAL" in
        gnome-terminal)
            gnome-terminal --title="$title" -- bash -c "$cmd; exec bash" &
            ;;
        xfce4-terminal)
            xfce4-terminal --title="$title" -e "bash -c '$cmd; exec bash'" &
            ;;
        konsole)
            konsole --new-tab -p tabtitle="$title" -e bash -c "$cmd; exec bash" &
            ;;
        xterm)
            xterm -T "$title" -e bash -c "$cmd; exec bash" &
            ;;
    esac
}

# --- Terminal 1: DDS Bridge ---------------------------------------------------
echo "🔌 [Terminal 1] Starting Micro-XRCE-DDS Agent..."
open_terminal "T1 — DDS Bridge" \
    "cd $DDS_AGENT_DIR && echo '🔌 Starting DDS Agent on UDP port 8888...' && ./MicroXRCEAgent udp4 -p 8888"

sleep 2  # Give DDS a moment to start

# --- Terminal 2: PX4 + Gazebo -------------------------------------------------
echo "🛩️  [Terminal 2] Starting PX4 SITL + Gazebo ($WORLD_NAME)..."
open_terminal "T2 — PX4 + Gazebo" \
    "cd $PX4_DIR && echo '🛩️  Building PX4 and launching Gazebo...' && PX4_GZ_WORLD=$WORLD_NAME make px4_sitl gz_x500_depth"

echo ""
echo "⏳ Waiting 30 seconds for PX4 + Gazebo to initialize..."
echo "   (Gazebo takes a while to load the 3D world)"
sleep 30

# --- Terminal 3: Controller ---------------------------------------------------
echo "🎮 [Terminal 3] Starting $CONTROLLER_NAME..."
open_terminal "T3 — $CONTROLLER_NAME" \
    "cd $ROS2_WS && source install/setup.bash && echo '🎮 Launching $CONTROLLER_NAME...' && python3 $CONTROLLER_SCRIPT"

sleep 3  # Let controller initialize before starting bridge

# --- Terminal 4: Gazebo-to-ROS2 Bridge ----------------------------------------
# Bridges the depth camera topics from Gazebo's internal messaging to ROS 2.
# Without this, your ROS 2 nodes can't see the depth camera data.
echo "🌉 [Terminal 4] Starting Gazebo → ROS 2 Bridge..."
open_terminal "T4 — GZ-ROS2 Bridge" \
    "source /opt/ros/humble/setup.bash && echo '🌉 Bridging Gazebo depth topics to ROS 2...' && ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock /depth_camera@sensor_msgs/msg/Image[gz.msgs.Image /depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked /camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo /world/house_3room/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"

sleep 3  # Let bridge initialize before starting filter

# --- Terminal 5: Point Cloud Filter -------------------------------------------
# Subscribes to raw point cloud, applies voxel grid + outlier removal,
# and publishes cleaned data for path planning.
FILTER_SCRIPT="$HOME/Desktop/Drone_IP/core/pointcloud_filter.py"
echo "🔬 [Terminal 5] Starting Point Cloud Filter..."
open_terminal "T5 — PointCloud Filter" \
    "cd $ROS2_WS && source install/setup.bash && echo '🔬 Starting Point Cloud Filter...' && python3 $FILTER_SCRIPT --ros-args -p use_sim_time:=true"

sleep 3  # Let filter start before launching RViz2

# --- Terminal 6: RViz2 Visualization ------------------------------------------
# 3D visualization of the filtered point cloud — you can SEE what the drone sees.
RVIZ_CONFIG="$HOME/Desktop/Drone_IP/config/drone_rviz.rviz"
echo "👁️  [Terminal 6] Starting RViz2 Visualization..."
open_terminal "T6 — RViz2" \
    "source /opt/ros/humble/setup.bash && echo '👁️  Launching RViz2...' && rviz2 -d $RVIZ_CONFIG --ros-args -p use_sim_time:=true"

sleep 2

# --- Terminal 7: TF Broadcaster -----------------------------------------------
# Publishes drone position as TF transforms so OctoMap knows where
# the camera is in the world.
TF_SCRIPT="$HOME/Desktop/Drone_IP/core/tf_broadcaster.py"
echo "📍 [Terminal 7] Starting TF Broadcaster..."
open_terminal "T7 — TF Broadcaster" \
    "cd $ROS2_WS && source install/setup.bash && echo '📍 Starting TF Broadcaster...' && python3 $TF_SCRIPT --ros-args -p use_sim_time:=true"

sleep 3  # TF must be running before OctoMap starts

# --- Terminal 8: OctoMap Server -----------------------------------------------
# Builds a persistent 3D occupancy map from the RAW point cloud.
# OctoMap handles its own spatial resolution — no pre-filtering needed.
echo "🗺️  [Terminal 8] Starting OctoMap Server..."
open_terminal "T8 — OctoMap Server" \
    "source /opt/ros/humble/setup.bash && echo '🗺️  Starting OctoMap Server...' && ros2 run octomap_server octomap_server_node --ros-args -p use_sim_time:=true -r cloud_in:=/depth_camera/points -p resolution:=0.10 -p frame_id:=map -p sensor_model.max_range:=5.0 -p sensor_model.min_range:=0.3 -p sensor_model.hit:=0.7 -p sensor_model.miss:=0.4 -p occupancy_min_z:=0.15 -p occupancy_max_z:=3.0 -p height_map:=false -p color.r:=0.6 -p color.g:=0.6 -p color.b:=0.6 -p color.a:=1.0"

echo ""
echo "=============================================="
echo "  ✅ All 8 terminals launched!"
echo "=============================================="
echo ""
echo "  T1: DDS Bridge        — should show 'Agent running'"
echo "  T2: PX4 + Gazebo      — wait for 'Ready for takeoff!'"
echo "  T3: Keyboard Control  — manual override if needed"
echo "  T4: GZ-ROS2 Bridge    — bridges depth camera + pose to ROS 2"
echo "  T5: PointCloud Filter — cleans raw point cloud data"
echo "  T6: RViz2             — 3D point cloud + OctoMap visualization"
echo "  T7: TF Broadcaster    — Gazebo ground-truth TF"
echo "  T8: OctoMap Server    — builds 3D occupancy map"
echo ""
if [[ "$1" != "--auto" ]]; then
    echo "  Controls (Focus on Terminal 3):"
    echo "    T = Takeoff    L = Land"
    echo "    W = Forward    S = Backward"
    echo "    A = Left       D = Right"
    echo "    R = Up         F = Down"
    echo "    Q = Quit"
fi
echo ""
echo "  To stop everything: close the terminal windows or Ctrl+C in each"
echo "=============================================="
