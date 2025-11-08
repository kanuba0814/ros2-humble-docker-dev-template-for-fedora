## ROS2 Humble - Fedora 43 Development Template

This template provides a complete ROS2 Humble development environment optimized for **Fedora 43**, using **Podman** (instead of Docker) and **Wayland** (instead of X11). Perfect for robotics development with modern Linux infrastructure!

### 🎯 What's Different for Fedora 43?

- ✅ **Podman** - Red Hat's daemonless container engine (Docker alternative)
- ✅ **Wayland** - Modern display protocol (default in Fedora 43)
- ✅ **SELinux** - Proper volume mount labels for Fedora's security
- ✅ **Beginner-friendly** - Extensive documentation and tmux guides

---

## 📋 Prerequisites (First-Time Setup)

If you're new to Fedora 43 or ROS2, follow these steps:

### 1. Install Podman (if not already installed)

```bash
sudo dnf install -y podman podman-compose
```

### 2. Configure Podman for Rootless Operation

```bash
# Check if subuid/subgid are configured
cat /etc/subuid
cat /etc/subgid

# If empty, run:
sudo usermod --add-subuids 100000-165535 --add-subgids 100000-165535 $USER
podman system migrate
```

### 3. (Optional) Enable GPU Access for 3D Visualization

For Intel/AMD GPUs (Mesa):
```bash
# Already configured in the container! Just verify your user can access /dev/dri
ls -l /dev/dri
```

For NVIDIA GPUs:
```bash
# Install NVIDIA container toolkit for Podman
sudo dnf install -y nvidia-container-toolkit
```

---

## 🚀 Quick Start

### Clone and Setup

```bash
git clone https://github.com/YOUR-USERNAME/ros2-humble-docker-dev-template-for-fedora.git
cd ros2-humble-docker-dev-template-for-fedora
./run.sh -h
```

### Building the Container Image

Build your ROS2 development environment (this takes 10-15 minutes the first time):

```bash
./run.sh -w dev_ws -i ros2-humble-fedora43 -b
```

**What happens during build:**
- Downloads ROS2 Humble base image
- Installs Gazebo, RViz, and robotics tools
- Configures Wayland support
- Sets up ZSH with helpful plugins
- Installs tmux/tmuxinator for session management

### Running the Container

Start your development environment:

```bash
./run.sh -w dev_ws -i ros2-humble-fedora43 -r
```

**On Wayland (default):** Automatically detects and uses Wayland socket
**On X11 (fallback):** Falls back to X11 if Wayland isn't available

---

## 🖥️ Tmux Basics for Beginners

When you run the container, you'll enter a **tmux session** managed by **tmuxinator**. Think of it as a powerful terminal multiplexer that lets you run multiple programs in organized windows and panes.

### What is Tmux?

Tmux lets you:
- Run multiple terminals in one window
- Split your screen horizontally and vertically
- Switch between different "windows" (like browser tabs)
- Keep programs running even if you disconnect

### Essential Tmux Commands

**All tmux commands start with the prefix `Ctrl+a`** (customized from the default `Ctrl+b`):

| Action | Shortcut | What it does |
|--------|----------|--------------|
| Split pane horizontally | `Ctrl+a` then `"` | Splits current pane top/bottom |
| Split pane vertically | `Ctrl+a` then `%` | Splits current pane left/right |
| Switch between panes | `Ctrl+a` then arrow keys | Move focus to different pane |
| Next window | `PageDown` or `Ctrl+a` then `n` | Switch to next window (tab) |
| Previous window | `PageUp` or `Ctrl+a` then `p` | Switch to previous window (tab) |
| Close current pane | `Ctrl+d` or type `exit` | Closes the active pane |
| Close session | `Ctrl+a` then `x` then `y` | Exits tmux and container |
| Scroll/copy mode | `Ctrl+a` then `[` | Scroll up in terminal (q to quit) |
| Select text with mouse | Hold `Shift` then drag | Copy text with mouse |

### Your Tmux Layout

The `.session.yml` file defines your layout:
- **Window 1 (workspace)**: Main development area - build and run code here
- **Window 2 (monitoring)**: Check topics, logs, and system resources

Edit `.session.yml` to customize your layout!

---

## 🛠️ Development Workflow

### Typical ROS2 Workflow in the Container

1. **Create a package** (in workspace window):
```bash
cd ~/dev_ws/src
ros2 pkg create --build-type ament_python my_robot_package
```

2. **Install dependencies**:
```bash
cd ~/dev_ws
rosdi  # Alias for: rosdep install --from-paths src --ignore-src -y
```

3. **Build your workspace**:
```bash
cbuild  # Alias for: colcon build --symlink-install
```

4. **Source your workspace**:
```bash
ssetup  # Alias for: source ./install/local_setup.zsh
```

5. **Run your nodes** (split panes with `Ctrl+a` then `"`):
```bash
# In pane 1:
ros2 run my_robot_package my_node

# In pane 2 (after splitting):
ros2 topic echo /my_topic
```

### Helpful Aliases (Pre-configured)

| Alias | Full Command | Purpose |
|-------|--------------|---------|
| `rosdi` | `rosdep install --from-paths src --ignore-src -y` | Install package dependencies |
| `cbuild` | `colcon build --symlink-install` | Build ROS2 workspace |
| `ssetup` | `source ./install/local_setup.zsh` | Source workspace |
| `cyclone` | `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | Use CycloneDDS (default) |
| `fastdds` | `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` | Use FastDDS |

---

## 🎨 Customization

### Customize Tmux Behavior

Edit `.tmux.conf` to add more features:
```bash
# Enable status bar at bottom
set -g status on

# Change prefix to something else
set-option -g prefix ^b
```

### Customize Session Layout

Edit `.session.yml` to change windows and panes:
```yaml
windows:
  - my_window:
      layout: even-horizontal
      panes:
        - cd ~/dev_ws && cbuild
        - ros2 topic list
```

---

## 🔧 Troubleshooting

### Container won't start / Permission denied

**SELinux issue** - Add `:z` labels (already done in run.sh):
```bash
# Volumes should have :z flag for SELinux
-v $HOME/dev_ws:/root/dev_ws:z
```

### GUI apps won't display (Wayland)

Check Wayland socket:
```bash
echo $WAYLAND_DISPLAY  # Should show something like "wayland-0"
ls $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY  # Should exist
```

### Podman command not found

Install Podman:
```bash
sudo dnf install -y podman
```

### GPU not working in Gazebo/RViz

Verify device access:
```bash
ls -l /dev/dri  # Should show render devices
```

For NVIDIA, ensure nvidia-container-toolkit is installed.

### Performance is slow

Try adjusting resources:
```bash
# Check Podman resources
podman info

# Consider using --cpus and --memory flags in run.sh
```

---

## 📚 Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Tmux Cheat Sheet](https://tmuxcheatsheet.com/)
- [Podman Documentation](https://docs.podman.io/)
- [Fedora Container Guide](https://docs.fedoraproject.org/en-US/fedora/latest/system-administrators-guide/virtualization/Containers/)

---

## 💡 Tips for Beginners

1. **Start small**: Run the demo talker/listener first (uncomment in `.session.yml`)
2. **Learn tmux gradually**: Master 3-4 commands, then expand
3. **Use aliases**: They save tons of typing (`cbuild` vs `colcon build --symlink-install`)
4. **Keep it running**: Tmux sessions persist until you explicitly close them
5. **Experiment safely**: Containers are isolated - break things and rebuild!

---

## 🤝 Contributing

Found a bug or have a suggestion? Open an issue or submit a pull request!

---

## 📄 License

See [LICENSE](LICENSE) file for details.
