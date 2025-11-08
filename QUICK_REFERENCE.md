# Quick Reference Card - ROS2 on Fedora 43

## 🚀 Starting Your Environment

```bash
# Build the image (first time only)
./run.sh -w dev_ws -i ros2-humble-fedora43 -b

# Run the container
./run.sh -w dev_ws -i ros2-humble-fedora43 -r
```

---

## ⌨️ Tmux Commands (Prefix = Ctrl+a)

### Windows (Tabs)
| Key | Action |
|-----|--------|
| `PageDown` | Next window |
| `PageUp` | Previous window |
| `Ctrl+a c` | Create new window |
| `Ctrl+a ,` | Rename window |
| `Ctrl+a w` | List all windows |

### Panes (Splits)
| Key | Action |
|-----|--------|
| `Ctrl+a "` | Split horizontal (top/bottom) |
| `Ctrl+a %` | Split vertical (left/right) |
| `Ctrl+a |` | Split vertical (easier!) |
| `Ctrl+a -` | Split horizontal (easier!) |
| `Ctrl+a ←↑→↓` | Move between panes |
| `Ctrl+a z` | Zoom/unzoom pane |
| `Ctrl+a x` | Close current pane |

### Scrolling
| Key | Action |
|-----|--------|
| `Ctrl+a [` | Enter scroll mode |
| `q` | Exit scroll mode |
| `Shift+Select` | Select text with mouse |

### Session
| Key | Action |
|-----|--------|
| `Ctrl+a d` | Detach (keep running) |
| `Ctrl+a x` then `y` | Exit session |

---

## 🤖 ROS2 Workflow Inside Container

### Initial Setup for New Package
```bash
# 1. Create package
cd ~/dev_ws/src
ros2 pkg create --build-type ament_python my_package

# 2. Install dependencies
cd ~/dev_ws
rosdi

# 3. Build
cbuild

# 4. Source workspace
ssetup
```

### Build-Test Cycle
```bash
# Build only modified packages
cbuild --packages-select my_package

# Source and run
ssetup
ros2 run my_package my_node
```

---

## 🔧 Pre-configured Aliases

| Alias | Command | Use |
|-------|---------|-----|
| `rosdi` | `rosdep install --from-paths src --ignore-src -y` | Install dependencies |
| `cbuild` | `colcon build --symlink-install` | Build workspace |
| `ssetup` | `source ./install/local_setup.zsh` | Source workspace |
| `cyclone` | Switch to CycloneDDS | Better performance (default) |
| `fastdds` | Switch to FastDDS | Alternative DDS |

---

## 🐛 Common ROS2 Debugging Commands

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Echo topic data
ros2 topic echo /topic_name

# Show topic info
ros2 topic info /topic_name

# Check message type
ros2 interface show std_msgs/msg/String

# Monitor topic frequency
ros2 topic hz /topic_name

# Check system
ros2 wtf
```

---

## 📊 Typical Window Layout

### Window 1: Build & Development
```
┌─────────────────────────┐
│   Build commands        │
│   cbuild, rosdi         │
├─────────────────────────┤
│   Run your nodes        │
│   ros2 run ...          │
└─────────────────────────┘
```

### Window 2: Monitoring
```
┌──────────┬──────────┐
│ Topics   │ Nodes    │
│ list     │ list     │
├──────────┼──────────┤
│ Echo     │ System   │
│ /cmd_vel │ htop     │
└──────────┴──────────┘
```

---

## 🎨 Customization Tips

### Edit Tmux Layout
```bash
# Edit session configuration
nano .session.yml
```

### Edit Tmux Behavior
```bash
# Edit tmux config
nano .tmux.conf

# Reload config (inside tmux)
Ctrl+a r
```

---

## 🆘 Quick Troubleshooting

### Container won't start
```bash
# Check Podman is running
podman info

# Check permissions
ls -l /dev/dri
```

### Can't see GUI apps
```bash
# Check Wayland
echo $WAYLAND_DISPLAY

# Check X11 fallback
echo $DISPLAY
```

### Build errors
```bash
# Clean build
rm -rf build install log
cbuild

# Update dependencies
rosdi
```

---

## 📚 Full Documentation

- **README.md** - Complete setup guide
- **TMUX_GUIDE.md** - Detailed tmux tutorial
- **setup-fedora43.sh** - Automated setup script

---

## 💡 Pro Tips

1. **Use tab completion** - ZSH autocompletes ROS2 commands
2. **Split for monitoring** - Run node in one pane, echo topic in another
3. **Name your windows** - `Ctrl+a ,` to rename (e.g., "Gazebo", "Build")
4. **Save your layout** - Edit `.session.yml` to save your perfect setup
5. **Learn 3 tmux commands** - Split (`|` or `-`), move (`arrows`), new window (`c`)

---

**Print this page and keep it handy!** 🖨️

After a week, these commands will be muscle memory. 💪
