# Changelog - Fedora 43 Adaptation

## Version 2.0 - Fedora 43 Compatibility (2025-11-08)

This release transforms the original Docker-based template into a **Fedora 43 optimized** development environment with Podman and Wayland support.

### 🎯 Major Changes

#### Container Runtime
- **Replaced Docker with Podman** - Daemonless, rootless container engine
- **Added SELinux support** - Proper `:z` volume mount labels for Fedora security
- **Removed Docker-specific flags** - Replaced `--gpus` with `--device /dev/dri` for Mesa/Intel/AMD GPUs
- **Added security optimizations** - `--security-opt label=disable` for better compatibility

#### Display Protocol
- **Added Wayland support** - Native support for Fedora 43's default display server
- **Auto-detection** - Automatically detects Wayland or X11 and configures accordingly
- **Wayland environment variables** - Added `QT_QPA_PLATFORM`, `GDK_BACKEND`, `CLUTTER_BACKEND`, `SDL_VIDEODRIVER`
- **X11 fallback** - Gracefully falls back to X11 if Wayland is not available
- **Wayland socket mounting** - Properly mounts `$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY`

#### Container Image
- **Added Wayland libraries** - `libwayland-dev`, `libxkbcommon-dev`, `wayland-protocols`, `qtwayland5`
- **Added Mesa utilities** - Better GPU support and diagnostics
- **Enhanced comments** - Extensive inline documentation for beginners
- **Improved environment variables** - Better defaults for Wayland-first usage

### 📚 Documentation Improvements

#### New Documentation Files
1. **README.md** (completely rewritten)
   - Fedora 43 specific prerequisites
   - Podman installation and configuration
   - Wayland detection and setup
   - GPU support guide (Intel/AMD/NVIDIA)
   - Comprehensive troubleshooting section
   - Beginner-friendly workflow examples

2. **TMUX_GUIDE.md** (new)
   - Complete tmux tutorial for beginners
   - What is tmux and why use it
   - Essential commands with examples
   - ROS2-specific workflows
   - Practice exercises
   - Common mistakes and solutions
   - Learning path for progressive mastery

3. **QUICK_REFERENCE.md** (new)
   - One-page cheat sheet
   - Tmux commands
   - ROS2 workflow
   - Pre-configured aliases
   - Common debugging commands
   - Printable format

4. **setup-fedora43.sh** (new)
   - Automated first-time setup script
   - Checks Fedora installation
   - Installs and configures Podman
   - Verifies Wayland and GPU support
   - Creates workspace
   - Optionally builds container
   - Colorful output with progress indicators

5. **CHANGELOG.md** (this file)
   - Documents all changes
   - Migration notes
   - Breaking changes

### 🔧 Configuration Improvements

#### .tmux.conf
- **Enhanced for beginners** - Extensive comments explaining each option
- **Better keybindings** - Added `|` and `-` for intuitive splits
- **Vim-style navigation** - `hjkl` for pane movement
- **Shift+Arrow resizing** - Easier pane resizing
- **Visual improvements** - Better colors and status bar
- **Increased history** - 10,000 line scrollback buffer
- **Quick reload** - `Ctrl+a r` to reload config
- **Activity monitoring** - Visual alerts for background windows

#### .session.yml
- **Beginner-friendly layout** - Default workspace and monitoring windows
- **Helpful comments** - Explains windows, panes, and layouts
- **Example nodes commented out** - Easy to enable for demos
- **Welcome messages** - Tips and alias reminders on startup

### 🐛 Bug Fixes

- Fixed volume mount permissions for SELinux
- Improved GPU device access for Podman
- Better error handling in run.sh
- Removed Docker-specific NVIDIA GPU flags

### 🔄 Breaking Changes

#### For Users Migrating from Original Template

1. **Docker → Podman**
   - Install Podman: `sudo dnf install -y podman`
   - Configure rootless: See README.md Prerequisites section
   - Existing Docker images won't work - rebuild with `./run.sh -b`

2. **Command Changes**
   - All `docker` commands are now `podman` commands
   - No changes to `run.sh` interface (still `-b` and `-r`)

3. **Volume Mounts**
   - Added `:z` SELinux labels - shouldn't affect functionality
   - If SELinux issues occur, see Troubleshooting in README.md

4. **X11 → Wayland**
   - Automatically detected, no action needed
   - If using X11, add `--device /dev/dri` manually for GPU access (already in run.sh)

### 📊 Testing

Tested on:
- ✅ Fedora 43 Workstation (Wayland)
- ✅ Fedora 43 Workstation (X11 fallback)
- ✅ Intel integrated GPU
- ✅ AMD GPU (Mesa drivers)

Not yet tested:
- ⚠️ NVIDIA GPU with proprietary drivers (should work with nvidia-container-toolkit)

### 🎓 Beginner Friendliness Improvements

- **Progressive disclosure** - Start simple, advanced options available
- **Extensive examples** - Every feature has examples
- **Troubleshooting guides** - Common issues documented
- **Visual aids** - ASCII diagrams for layouts
- **Practice exercises** - Learn by doing
- **Quick wins** - Can get started in minutes

### 🚀 Performance Improvements

- **Rootless Podman** - Better security, no daemon overhead
- **CycloneDDS default** - Better ROS2 performance
- **Symlink install** - Faster rebuilds (`cbuild` alias)
- **Efficient caching** - Better container layer caching

### 🔮 Future Enhancements

Potential improvements for future versions:
- [ ] Podman Quadlet support (systemd integration)
- [ ] Dev container configuration for VSCode
- [ ] Automated tests for container
- [ ] Multi-architecture support (ARM64)
- [ ] NVIDIA GPU testing and documentation
- [ ] Integration with Fedora Toolbox
- [ ] Pre-built images on container registries

### 🙏 Acknowledgments

Based on the original work by [sskorol/ros2-humble-docker-dev-template](https://github.com/sskorol/ros2-humble-docker-dev-template).

Adapted for Fedora 43 with focus on:
- Modern Fedora tooling (Podman, Wayland)
- Beginner accessibility
- Security best practices (SELinux, rootless containers)

---

## How to Upgrade

If you're using the original template:

```bash
# Backup your workspace
cp -r ~/dev_ws ~/dev_ws.backup

# Pull latest changes
git pull origin main

# Rebuild container with Podman
./run.sh -w dev_ws -i ros2-humble-fedora43 -b

# Run new environment
./run.sh -w dev_ws -i ros2-humble-fedora43 -r
```

Your workspace files are mounted as volumes, so they remain unchanged!

---

For questions or issues, please open a GitHub issue.
