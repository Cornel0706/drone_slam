#!/bin/bash
# ============================================================
#  install.sh — Automated installation script for Raspberry Pi 5
# ============================================================
#  Run once after cloning the repository on the RPi5:
#    git clone https://github.com/USER/drone_slam.git ~/drone_ws/src/drone_slam
#    cd ~/drone_ws/src/drone_slam
#    bash install.sh
#
#  This script installs:
#    1. System dependencies (udev rules, swap)
#    2. ROS2 Jazzy (if not already installed)
#    3. All required ROS2 packages
#    4. OAK-S2 camera driver (depthai-ros)
#    5. Builds the workspace
# ============================================================

set -e  # Exit immediately on any error

# Terminal colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
success() { echo -e "${GREEN}[OK]${NC} $1"; }
warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
error()   { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║         DRONE SLAM — RPi5 Installation Script            ║"
echo "║         HolyBro S500 + Pixhawk 6C Mini + OAK-S2          ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# ── System check ─────────────────────────────────────────────────────
info "Checking system..."
ARCH=$(uname -m)
if [ "$ARCH" != "aarch64" ]; then
    warn "Detected architecture: $ARCH (expected: aarch64 for RPi5)"
    warn "Continuing at your own risk..."
fi

OS=$(lsb_release -rs 2>/dev/null || echo "unknown")
if [ "$OS" != "24.04" ]; then
    warn "Detected OS: Ubuntu $OS (recommended: 24.04)"
fi

# ── Swap configuration (4GB required for compilation) ────────────────
info "Checking swap space (4GB recommended for colcon build on RPi5)..."
SWAP_SIZE=$(free -m | awk '/^Swap:/{print $2}')
if [ -z "$SWAP_SIZE" ] || [ "$SWAP_SIZE" -lt 3500 ]; then
    info "Setting up 4GB swapfile..."
    sudo swapoff /swapfile 2>/dev/null || true
    sudo fallocate -l 4G /swapfile 2>/dev/null || sudo dd if=/dev/zero of=/swapfile bs=1M count=4096 status=progress
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    if ! grep -q '/swapfile' /etc/fstab; then
        echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    fi
    success "4GB swap configured"
else
    success "Swap is sufficient ($SWAP_SIZE MB)"
fi

# ── System update ─────────────────────────────────────────────────────
info "Updating system packages..."
sudo apt update -q
sudo apt upgrade -y -q
sudo apt install -y -q git curl wget nano build-essential cmake python3-pip
success "System updated"

# ── Install ROS2 Jazzy (if not already installed) ────────────────────
if ! command -v ros2 &> /dev/null; then
    info "ROS2 Jazzy not found. Installing..."

    sudo apt install -y software-properties-common
    sudo add-apt-repository universe -y

    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) \
        signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu \
        $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update -q
    sudo apt install -y ros-jazzy-ros-base ros-dev-tools python3-colcon-common-extensions

    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    success "ROS2 Jazzy installed"
else
    success "ROS2 Jazzy already installed"
fi

source /opt/ros/jazzy/setup.bash

# ── udev rule for OAK-S2 camera ──────────────────────────────────────
info "Configuring udev rule for OAK-S2 camera (USB access)..."
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | \
    sudo tee /etc/udev/rules.d/80-luxonis.rules > /dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -a -G dialout $USER
success "udev rule configured"

# ── Install ROS2 packages ─────────────────────────────────────────────
info "Installing ROS2 packages..."
sudo apt install -y \
    ros-jazzy-rtabmap \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-rtabmap-slam \
    ros-jazzy-rtabmap-odom \
    ros-jazzy-rtabmap-viz \
    ros-jazzy-robot-localization \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-mavros-msgs \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs
success "ROS2 packages installed"

# ── OAK-S2 camera driver (depthai-ros) ───────────────────────────────
info "Installing OAK-S2 driver (depthai-ros)..."
if sudo apt install -y ros-jazzy-depthai-ros ros-jazzy-depthai-ros-driver 2>/dev/null; then
    success "depthai-ros installed from binary packages"
else
    warn "Binary packages not available. Building from source..."
    WORKSPACE_DIR=$(dirname $(dirname $(realpath "$0")))
    cd "$WORKSPACE_DIR"
    if [ ! -d "src/depthai-ros" ]; then
        git clone -b jazzy https://github.com/luxonis/depthai-ros.git src/depthai-ros
    fi
fi

# ── GeographicLib datasets for MAVROS ────────────────────────────────
info "Installing GeographicLib datasets (required by MAVROS)..."
if [ ! -f /usr/share/GeographicLib/geoids/egm96-5.pgm ]; then
    if [ -f /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh ]; then
        sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
    else
        wget -q https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
        sudo bash install_geographiclib_datasets.sh
        rm -f install_geographiclib_datasets.sh
    fi
    success "GeographicLib datasets installed"
else
    success "GeographicLib datasets already installed"
fi

# ── Ensure Python scripts are executable ──────────────────────────────
WORKSPACE_DIR=$(dirname $(dirname $(realpath "$0")))
chmod +x "$WORKSPACE_DIR"/src/drone_slam/scripts/*.py 2>/dev/null || true

# ── Build the workspace ───────────────────────────────────────────────
info "Building ROS2 workspace (may take 10-20 minutes on RPi5)..."
cd "$WORKSPACE_DIR"

rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build with a single parallel worker to conserve RPi5 RAM
colcon build \
    --symlink-install \
    --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# ── Add workspace to .bashrc ──────────────────────────────────────────
WORKSPACE_SETUP="source $WORKSPACE_DIR/install/setup.bash"
if ! grep -q "$WORKSPACE_SETUP" ~/.bashrc; then
    echo "$WORKSPACE_SETUP" >> ~/.bashrc
    success "Workspace sourced in .bashrc"
fi

# ── Enable GPIO UART for Pixhawk connection ───────────────────────────
info "Configuring UART for Pixhawk (TELEM2 → RPi5 GPIO)..."
CONFIG_PATH="/boot/firmware/config.txt"
if [ -f "$CONFIG_PATH" ]; then
    if ! grep -q "dtparam=uart0=on" "$CONFIG_PATH"; then
        echo "" | sudo tee -a "$CONFIG_PATH"
        echo "# UART for Pixhawk 6C Mini" | sudo tee -a "$CONFIG_PATH"
        echo "enable_uart=1" | sudo tee -a "$CONFIG_PATH"
        echo "dtparam=uart0=on" | sudo tee -a "$CONFIG_PATH"
        warn "UART configured in $CONFIG_PATH — REBOOT REQUIRED!"
    fi
fi
# Disable serial console login on ttyAMA0 so it doesn't collide with MAVROS
sudo systemctl stop serial-getty@ttyAMA0.service 2>/dev/null || true
sudo systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║               INSTALLATION COMPLETE!                    ║"
echo "╠══════════════════════════════════════════════════════════╣"
echo "║  Next step: REBOOT to activate GPIO UART                ║"
echo "║    sudo reboot                                           ║"
echo "║                                                          ║"
echo "║  After reboot, start the full system with:              ║"
echo "║    ros2 launch drone_slam slam_full.launch.py           ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""
