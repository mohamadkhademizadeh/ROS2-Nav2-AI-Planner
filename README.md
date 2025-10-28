# ROS2 Nav2 + AI Local Planner (Gazebo Sim)

A Dockerized **ROS 2 (Humble)** simulation that showcases:
- **Gazebo** world with a simple differential-drive robot
- **SLAM Toolbox** for online mapping
- **Nav2** for global path planning
- A Python **AI local planner** node that modulates `cmd_vel` to avoid dynamic obstacles (learned policy or rule-based fallback)
- **RViz** visualization
- One-command start via **Docker Compose**

> Built for portfolio: clean structure, launch files, and a lightweight AI planner that you can later swap for a trained PPO policy.

---

## Quickstart (Docker)

```bash
# 1) Build image
docker compose -f docker/docker-compose.yml build

# 2) Launch Gazebo + Nav2 + SLAM + RViz (GUI requires X/Wayland forwarding)
docker compose -f docker/docker-compose.yml up
```

> On Linux with X11 you may need `xhost +local:` to allow GUI. On macOS/Windows use a VNC/X server (Doc notes inline).

---

## Quickstart (Local ROS2, optional)

```bash
# Install ROS 2 Humble and dependencies (Ubuntu 22.04 recommended)
# Then:
colcon build --symlink-install
source install/setup.bash

# Launch Nav2 + SLAM + Gazebo world + AI planner
ros2 launch nav_launch bringup.launch.py
```

---

## Repo Layout

```
ROS2-Nav2-AI-Planner/
├── src/
│   ├── nav_ai/                # AI planner package (Python)
│   │   ├── nav_ai/ai_planner_node.py
│   │   ├── nav_ai/__init__.py
│   │   ├── package.xml
│   │   ├── setup.cfg
│   │   ├── setup.py
│   │   ├── launch/ai_planner.launch.py
│   │   └── policy/model.pt                # (placeholder)
│   ├── nav_launch/
│   │   └── launch/bringup.launch.py       # Gazebo + Nav2 + SLAM + AI planner
│   └── slam_launch/
│       └── launch/slam_toolbox.launch.py
├── worlds/factory.world
├── rviz/nav_view.rviz
├── docker/Dockerfile.ros2
├── docker/docker-compose.yml
└── README.md
```

---

## Notes

- This scaffold uses a **topic-injection** approach for the AI planner: it subscribes to the **Nav2** `cmd_vel` and **LaserScan** and gently modifies the velocity to avoid imminent obstacles or follow a learned policy. This avoids needing to compile a C++ Nav2 controller plugin in this portfolio project.
- The Gazebo world is a simple maze-like factory floor. You can add dynamic obstacles by spawning moving models.
- To replace the fallback heuristic with a **trained policy**, drop your Torch `model.pt` into `src/nav_ai/policy/` and set `use_policy:=true` in the AI planner launch.
