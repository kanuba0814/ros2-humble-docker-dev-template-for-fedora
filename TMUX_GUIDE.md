# Tmux Beginner's Guide for ROS2 Development

Welcome! This guide will help you master tmux, a powerful terminal multiplexer that's essential for efficient ROS2 development.

## 🤔 What is Tmux?

Tmux (Terminal Multiplexer) lets you:
- Run multiple terminal sessions in one window
- Split your screen into panes (like tiling windows)
- Switch between different workspaces (windows)
- Keep processes running even if you disconnect
- Save and restore your entire terminal layout

Think of it as having multiple terminal tabs AND split views, all in one place!

---

## 🎮 The Prefix Key

**IMPORTANT**: All tmux commands start with a "prefix" key combination.

- **Default prefix**: `Ctrl+b`
- **Our custom prefix**: `Ctrl+a` (easier to reach!)

When you see `prefix`, it means press `Ctrl+a`, release, then press the next key.

---

## 🪟 Understanding Windows vs Panes

### Windows
- Like browser tabs
- Each window is a separate workspace
- Switch between them with PageUp/PageDown
- Shown in the status bar at the bottom

### Panes
- Splits within a window
- Multiple terminal views side-by-side
- Perfect for running a node and monitoring it simultaneously

---

## ⌨️ Essential Commands

### Window Management

| Command | What it does | Example use case |
|---------|--------------|------------------|
| `PageDown` | Next window | Switch from build window to monitoring |
| `PageUp` | Previous window | Go back to previous task |
| `prefix c` | Create new window | Start a new task in a fresh window |
| `prefix ,` | Rename window | Name it "Build" or "Gazebo" |
| `prefix w` | List all windows | See all your workspaces and jump to one |
| `prefix &` | Kill window | Close entire window (prompts for confirmation) |

### Pane Management

| Command | What it does | Example use case |
|---------|--------------|------------------|
| `prefix "` | Split horizontally (top/bottom) | Run node on top, monitor topics below |
| `prefix %` | Split vertically (left/right) | Code on left, test on right |
| `prefix ←↑→↓` | Move between panes | Navigate your split views |
| `prefix o` | Go to next pane | Quick switch to other pane |
| `prefix x` | Kill current pane | Close just this pane (not the whole window) |
| `prefix z` | Zoom/unzoom pane | Make pane fullscreen (toggle) |
| `prefix Space` | Cycle through layouts | Try different split arrangements |

### Copy Mode (Scrolling)

| Command | What it does |
|---------|--------------|
| `prefix [` | Enter copy mode (scroll with arrows/PgUp/PgDn) |
| `q` | Exit copy mode |
| `Shift + mouse select` | Select text to copy (with mouse) |

### Session Management

| Command | What it does |
|---------|--------------|
| `prefix d` | Detach from session (keeps running!) |
| `tmux attach` | Re-attach to detached session |
| `prefix x` then `y` | Kill session (in our config, kills container too) |

---

## 🎯 Common ROS2 Workflows with Tmux

### Workflow 1: Build and Test

1. Start in the workspace window
2. Split horizontally: `prefix "`
   - **Top pane**: Run build command
   ```bash
   cd ~/dev_ws
   cbuild
   ```
   - **Bottom pane**: Move with `prefix ↓`, run your node
   ```bash
   ssetup
   ros2 run my_package my_node
   ```

### Workflow 2: Monitor Multiple Topics

1. Create new window: `prefix c`
2. Rename it: `prefix ,` type "Monitoring"
3. Split into 4 panes:
   - `prefix "` (split horizontal)
   - `prefix %` (split first pane vertical)
   - Move to bottom: `prefix ↓`
   - `prefix %` (split second pane vertical)
4. Now you have 4 panes for:
   ```bash
   # Pane 1
   ros2 topic list

   # Pane 2
   ros2 topic echo /camera/image

   # Pane 3
   ros2 node list

   # Pane 4
   ros2 topic hz /lidar
   ```

### Workflow 3: Run Gazebo and Control Robot

1. **Window 1**: Gazebo simulation
   ```bash
   ros2 launch my_robot_pkg gazebo.launch.py
   ```

2. **Window 2** (create with `prefix c`): Split for control and monitoring
   - Top pane: Control
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   - Bottom pane: Monitor velocity
   ```bash
   ros2 topic echo /cmd_vel
   ```

---

## 💡 Pro Tips

### 1. Mouse Support is Enabled!
- Click on panes to switch focus
- Click on window names in status bar
- **Hold Shift to select text** with mouse

### 2. Resize Panes
- `prefix Ctrl+←↑→↓` - Resize current pane (hold Ctrl and press arrows)

### 3. Swap Panes
- `prefix {` - Swap with previous pane
- `prefix }` - Swap with next pane

### 4. Synchronize Panes
Want to type the same command in all panes?
- `prefix :` (enter command mode)
- Type: `setw synchronize-panes on`
- Now typing affects all panes!
- Turn off: `setw synchronize-panes off`

### 5. Create Your Own Layouts
Edit `.session.yml` to define your perfect setup:

```yaml
windows:
  - build:
      layout: main-vertical
      panes:
        - cbuild
        - ssetup && ros2 launch my_pkg my_launch.py
  - monitor:
      layout: tiled
      panes:
        - ros2 topic list
        - ros2 node list
        - htop
        - ros2 topic echo /diagnostics
```

---

## 🐛 Common Mistakes

### "I pressed Ctrl+b but nothing happened!"
- Remember, our prefix is `Ctrl+a`, not `Ctrl+b`

### "I closed my terminal and lost everything!"
- tmux runs inside the container. If you exit the container, the session ends
- Use `prefix d` to detach without closing (then reconnect with `tmux attach`)

### "I can't scroll up!"
- Enter copy mode first: `prefix [`
- Scroll with arrow keys or PageUp/PageDown
- Press `q` to exit

### "Text selection doesn't work!"
- Hold `Shift` while selecting with mouse

---

## 📚 Learning Path

### Week 1: Master the Basics
- [ ] Split panes horizontally and vertically
- [ ] Switch between panes with arrow keys
- [ ] Create new windows and switch between them
- [ ] Close panes and windows

### Week 2: Intermediate Skills
- [ ] Rename windows meaningfully
- [ ] Resize panes
- [ ] Use copy mode to scroll
- [ ] Customize your `.session.yml`

### Week 3: Advanced Usage
- [ ] Create custom layouts for specific tasks
- [ ] Use synchronize-panes for multi-node commands
- [ ] Add your own key bindings in `.tmux.conf`

---

## 🎓 Practice Exercises

### Exercise 1: Create a 4-pane Monitoring Dashboard
1. Start with one pane
2. Split it into 4 equal panes
3. In each pane, run:
   - Pane 1: `ros2 topic list`
   - Pane 2: `ros2 node list`
   - Pane 3: `top` (system resources)
   - Pane 4: `ros2 wtf` (diagnostics)

### Exercise 2: Build and Test Workflow
1. Create a window named "Build"
2. Split it into 2 vertical panes
3. Left pane: Build command
4. Right pane: Run tests
5. Practice switching between them quickly

---

## 🔗 More Resources

- [Tmux Cheat Sheet](https://tmuxcheatsheet.com/)
- [Tmux Documentation](https://github.com/tmux/tmux/wiki)
- [Tmuxinator GitHub](https://github.com/tmuxinator/tmuxinator)

---

## Quick Reference Card

```
PREFIX = Ctrl+a

Windows:
  PageDown/PageUp     → Next/Previous window
  prefix c            → Create window
  prefix ,            → Rename window
  prefix w            → List windows

Panes:
  prefix "            → Split horizontal
  prefix %            → Split vertical
  prefix ←↑→↓         → Move between panes
  prefix z            → Zoom/unzoom pane
  prefix x            → Kill pane

Copy Mode:
  prefix [            → Enter copy mode (scroll)
  q                   → Exit copy mode

Session:
  prefix d            → Detach session
  prefix x, y         → Kill session
```

Print this and keep it handy until muscle memory kicks in!

---

Happy tmux-ing! 🚀
