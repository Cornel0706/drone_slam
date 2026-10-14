#!/bin/bash
# ============================================================
#  install.sh — Script de instalare automata pe Raspberry Pi 5
# ============================================================
#  Ruleaza o singura data dupa ce clonezi repo-ul pe RPi5:
#    git clone https://github.com/USER/drone_slam.git ~/drone_ws/src/drone_slam
#    cd ~/drone_ws/src/drone_slam
#    bash install.sh
#
#  Scriptul instaleaza:
#    1. Dependente sistem (udev, swap)
#    2. ROS2 Jazzy (daca nu e instalat)
#    3. Toate pachetele ROS2 necesare
#    4. Driver camera OAK-S2
#    5. Compileaza workspace-ul
# ============================================================

set -e  # Opreste la prima eroare

# Culori pentru output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Functii helper
info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
success() { echo -e "${GREEN}[OK]${NC} $1"; }
warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
error()   { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║         DRONE SLAM — Script de instalare RPi5            ║"
echo "║         HolyBro S500 + Pixhawk 6C Mini + OAK-S2          ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# ── Verificare sistem ────────────────────────────────────────────────
info "Verificare sistem..."
ARCH=$(uname -m)
if [ "$ARCH" != "aarch64" ]; then
    warn "Arhitectura detectata: $ARCH (asteptat: aarch64 pentru RPi5)"
    warn "Continui pe raspunderea ta..."
fi

OS=$(lsb_release -rs 2>/dev/null || echo "unknown")
if [ "$OS" != "24.04" ]; then
    warn "OS detectat: Ubuntu $OS (recomandat: 24.04)"
fi

# ── Configurare Swap (4GB pentru compilare) ──────────────────────────
info "Configurare swap 4GB (necesar pentru colcon build)..."
SWAP_SIZE=$(free -m | awk '/^Swap:/{print $2}')
if [ "$SWAP_SIZE" -lt 2048 ]; then
    sudo dphys-swapfile swapoff 2>/dev/null || true
    sudo sed -i 's/CONF_SWAPSIZE=.*/CONF_SWAPSIZE=4096/' /etc/dphys-swapfile
    sudo dphys-swapfile setup
    sudo dphys-swapfile swapon
    success "Swap configurat la 4GB"
else
    success "Swap suficient ($SWAP_SIZE MB)"
fi

# ── Update sistem ─────────────────────────────────────────────────────
info "Update sistem..."
sudo apt update -q
sudo apt upgrade -y -q
sudo apt install -y -q git curl wget nano build-essential cmake python3-pip
success "Sistem actualizat"

# ── Instalare ROS2 Jazzy (daca nu e instalat) ────────────────────────
if ! command -v ros2 &> /dev/null; then
    info "ROS2 Jazzy nu e instalat. Instalez..."

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
    success "ROS2 Jazzy instalat"
else
    success "ROS2 Jazzy deja instalat"
fi

source /opt/ros/jazzy/setup.bash

# ── Regula udev pentru OAK-S2 ────────────────────────────────────────
info "Configurare udev pentru camera OAK-S2..."
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | \
    sudo tee /etc/udev/rules.d/80-luxonis.rules > /dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -a -G dialout $USER
success "udev configurat"

# ── Instalare pachete ROS2 ────────────────────────────────────────────
info "Instalare pachete ROS2..."
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
success "Pachete ROS2 instalate"

# ── Driver camera OAK-S2 (depthai-ros) ───────────────────────────────
info "Instalare driver OAK-S2 (depthai-ros)..."
if sudo apt install -y ros-jazzy-depthai-ros 2>/dev/null; then
    success "depthai-ros instalat din binare"
else
    warn "Binare nu sunt disponibile. Build din sursa..."
    WORKSPACE_DIR=$(dirname $(dirname $(realpath "$0")))
    cd "$WORKSPACE_DIR"
    if [ ! -d "src/depthai-ros" ]; then
        git clone -b jazzy https://github.com/luxonis/depthai-ros.git src/depthai-ros
    fi
fi

# ── Date geografice pentru MAVROS ─────────────────────────────────────
info "Instalare date geografice (MAVROS)..."
if [ ! -f /usr/share/GeographicLib/geoids/egm96-5.pgm ]; then
    wget -q https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
    sudo bash install_geographiclib_datasets.sh
    rm -f install_geographiclib_datasets.sh
    success "Date geografice instalate"
else
    success "Date geografice deja instalate"
fi

# ── Compilare workspace ───────────────────────────────────────────────
info "Compilare workspace ROS2 (poate dura 10-20 minute pe RPi5)..."
WORKSPACE_DIR=$(dirname $(dirname $(realpath "$0")))
cd "$WORKSPACE_DIR"

rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Compilare cu un singur worker paralel (economisire memorie RPi5)
colcon build \
    --symlink-install \
    --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# ── Configurare bashrc ────────────────────────────────────────────────
WORKSPACE_SETUP="source $WORKSPACE_DIR/install/setup.bash"
if ! grep -q "$WORKSPACE_SETUP" ~/.bashrc; then
    echo "$WORKSPACE_SETUP" >> ~/.bashrc
    success "Workspace adaugat in bashrc"
fi

# ── Configurare UART GPIO (Pixhawk) ──────────────────────────────────
info "Configurare UART GPIO pentru Pixhawk..."
if ! grep -q "enable_uart=1" /boot/firmware/config.txt 2>/dev/null; then
    echo "" | sudo tee -a /boot/firmware/config.txt
    echo "# UART pentru Pixhawk 6C Mini" | sudo tee -a /boot/firmware/config.txt
    echo "enable_uart=1" | sudo tee -a /boot/firmware/config.txt
    echo "dtoverlay=disable-bt" | sudo tee -a /boot/firmware/config.txt
    warn "UART configurat — necesita REBOOT!"
fi

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║                  INSTALARE COMPLETA!                    ║"
echo "╠══════════════════════════════════════════════════════════╣"
echo "║  Urmatorul pas: REBOOT pentru activare UART GPIO        ║"
echo "║    sudo reboot                                          ║"
echo "║                                                          ║"
echo "║  Dupa reboot, porneste sistemul:                        ║"
echo "║    ros2 launch drone_slam slam_full.launch.py           ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""
