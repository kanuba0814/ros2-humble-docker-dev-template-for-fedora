#!/bin/bash
# ============================================================================
# Fedora 43 Setup Script for ROS2 Humble Development Environment
# ============================================================================
# This script helps first-time users set up their Fedora 43 system for
# ROS2 development with Podman and Wayland support.
#
# Usage: ./setup-fedora43.sh
# ============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
print_header() {
    echo -e "\n${BLUE}===================================================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}===================================================================${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# Check if running on Fedora
check_fedora() {
    if [[ ! -f /etc/fedora-release ]]; then
        print_error "This script is designed for Fedora. Your system may not be compatible."
        read -p "Continue anyway? (y/N) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    else
        print_success "Running on Fedora $(cat /etc/fedora-release)"
    fi
}

# Check if Podman is installed
check_podman() {
    print_header "Checking Podman Installation"

    if command -v podman &> /dev/null; then
        print_success "Podman is already installed ($(podman --version))"
    else
        print_warning "Podman not found. Installing..."
        sudo dnf install -y podman podman-compose
        print_success "Podman installed successfully"
    fi
}

# Configure Podman for rootless operation
configure_podman() {
    print_header "Configuring Podman for Rootless Operation"

    # Check if subuid/subgid are configured
    if grep -q "^${USER}:" /etc/subuid && grep -q "^${USER}:" /etc/subgid; then
        print_success "User namespaces already configured"
    else
        print_warning "Configuring user namespaces..."
        sudo usermod --add-subuids 100000-165535 --add-subgids 100000-165535 $USER
        podman system migrate
        print_success "User namespaces configured"
        print_warning "You may need to log out and back in for changes to take effect"
    fi

    # Test Podman
    if podman run --rm hello-world &> /dev/null; then
        print_success "Podman is working correctly"
    else
        print_warning "Podman test failed - you may need to log out and back in"
    fi
}

# Check Wayland
check_wayland() {
    print_header "Checking Wayland Support"

    if [[ -n "$WAYLAND_DISPLAY" ]]; then
        print_success "Running on Wayland (WAYLAND_DISPLAY=$WAYLAND_DISPLAY)"
    else
        print_warning "Not running on Wayland (X11 fallback will be used)"
        print_info "To use Wayland, log out and select 'GNOME on Wayland' at login"
    fi
}

# Check GPU support
check_gpu() {
    print_header "Checking GPU Support"

    if [[ -d /dev/dri ]]; then
        print_success "GPU devices found:"
        ls -l /dev/dri/ | grep -E "card|render" | awk '{print "  " $0}'
    else
        print_warning "No GPU devices found - 3D acceleration may not work"
    fi

    # Check for NVIDIA
    if lspci | grep -i nvidia &> /dev/null; then
        print_info "NVIDIA GPU detected"
        if rpm -q nvidia-container-toolkit &> /dev/null; then
            print_success "NVIDIA container toolkit is installed"
        else
            print_warning "NVIDIA container toolkit not installed"
            print_info "Install with: sudo dnf install -y nvidia-container-toolkit"
        fi
    fi
}

# Check required tools
check_tools() {
    print_header "Checking Development Tools"

    local tools=("git" "curl" "wget")
    local missing_tools=()

    for tool in "${tools[@]}"; do
        if command -v $tool &> /dev/null; then
            print_success "$tool is installed"
        else
            missing_tools+=($tool)
        fi
    done

    if [[ ${#missing_tools[@]} -gt 0 ]]; then
        print_warning "Installing missing tools: ${missing_tools[*]}"
        sudo dnf install -y "${missing_tools[@]}"
    fi
}

# Create workspace directory
create_workspace() {
    print_header "Setting Up Workspace"

    read -p "Enter workspace directory name (default: dev_ws): " workspace_name
    workspace_name=${workspace_name:-dev_ws}

    if [[ -d "$HOME/$workspace_name" ]]; then
        print_info "Workspace $HOME/$workspace_name already exists"
    else
        mkdir -p "$HOME/$workspace_name/src"
        print_success "Created workspace at $HOME/$workspace_name"
    fi

    echo "$workspace_name" > .workspace_name
}

# Build the container image
build_container() {
    print_header "Building ROS2 Container Image"

    read -p "Build container image now? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        workspace_name=$(cat .workspace_name)
        print_info "Building image... This will take 10-15 minutes."
        print_info "Command: ./run.sh -w $workspace_name -i ros2-humble-fedora43 -b"

        if ./run.sh -w "$workspace_name" -i ros2-humble-fedora43 -b; then
            print_success "Container image built successfully!"
        else
            print_error "Container build failed. Check the output above for errors."
            return 1
        fi
    else
        print_info "Skipping build. Run this later: ./run.sh -w WORKSPACE_NAME -i ros2-humble-fedora43 -b"
    fi
}

# Show next steps
show_next_steps() {
    workspace_name=$(cat .workspace_name 2>/dev/null || echo "dev_ws")

    print_header "Setup Complete! 🎉"

    echo -e "${GREEN}Next Steps:${NC}\n"

    echo -e "${YELLOW}1. Build the container (if not done):${NC}"
    echo -e "   ./run.sh -w $workspace_name -i ros2-humble-fedora43 -b\n"

    echo -e "${YELLOW}2. Run the development environment:${NC}"
    echo -e "   ./run.sh -w $workspace_name -i ros2-humble-fedora43 -r\n"

    echo -e "${YELLOW}3. Learn tmux basics:${NC}"
    echo -e "   Read TMUX_GUIDE.md for a complete guide\n"

    echo -e "${YELLOW}4. Start developing:${NC}"
    echo -e "   - Use PageUp/PageDown to switch between windows"
    echo -e "   - Split panes with Ctrl+a then | (vertical) or - (horizontal)"
    echo -e "   - See README.md for detailed workflows\n"

    echo -e "${GREEN}Useful aliases inside the container:${NC}"
    echo -e "   ${BLUE}rosdi${NC}  - Install ROS dependencies"
    echo -e "   ${BLUE}cbuild${NC} - Build your workspace"
    echo -e "   ${BLUE}ssetup${NC} - Source your workspace\n"

    print_info "Documentation:"
    echo -e "   📖 README.md - Complete guide for Fedora 43"
    echo -e "   🖥️  TMUX_GUIDE.md - Tmux tutorial for beginners"
}

# Main setup flow
main() {
    clear
    print_header "ROS2 Humble on Fedora 43 - First-Time Setup"

    echo "This script will:"
    echo "  1. Check your Fedora installation"
    echo "  2. Install and configure Podman"
    echo "  3. Verify Wayland and GPU support"
    echo "  4. Set up your workspace"
    echo "  5. (Optional) Build the container image"
    echo ""

    read -p "Continue with setup? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Setup cancelled."
        exit 0
    fi

    check_fedora
    check_podman
    configure_podman
    check_wayland
    check_gpu
    check_tools
    create_workspace
    build_container
    show_next_steps

    # Cleanup
    rm -f .workspace_name
}

# Run main function
main
