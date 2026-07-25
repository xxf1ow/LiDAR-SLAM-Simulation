#!/usr/bin/env bash
# =============================================================================
# CAN Driver 8030D — Jetson Setup Script
# =============================================================================
# One-command installation for Jetson Orin/Nano with ROS 2 Humble.
#
# This script will:
#   1. Verify ROS 2 Humble is installed
#   2. Check JetPack / L4T version
#   3. Install system dependencies
#   4. Install libcontrolcan.so to system library path
#   5. Create udev rule for ZLG USBCAN2 device
#   6. Build the ROS 2 package
#   7. Optionally add source line to ~/.bashrc
#
# Usage:
#   chmod +x setup_jetson.sh
#   ./setup_jetson.sh
#
# Or run remotely via SSH:
#   ssh jetson@<ip> 'bash -s' < setup_jetson.sh
# =============================================================================

set -e  # Exit on error

# ---------------------------------------------------------------------------
# Color output helpers
# ---------------------------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

info()    { echo -e "${BLUE}[INFO]${NC}    $*"; }
success() { echo -e "${GREEN}[OK]${NC}      $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}    $*"; }
error()   { echo -e "${RED}[ERROR]${NC}   $*"; }

# ---------------------------------------------------------------------------
# Header
# ---------------------------------------------------------------------------
echo ""
echo "============================================"
echo "  CAN Driver 8030D — Jetson Setup"
echo "  ROS 2 + ZLG USBCAN2 + ZL-8030D"
echo "============================================"
echo ""

# ---------------------------------------------------------------------------
# Step 1: Check ROS 2
# ---------------------------------------------------------------------------
info "Step 1/7: Checking ROS 2 installation..."

ROS_SETUP=""
for dir in /opt/ros/*/setup.bash; do
    if [ -f "$dir" ]; then
        distro=$(echo "$dir" | grep -oP '/opt/ros/\K[^/]+')
        if [ "$distro" != "noetic" ]; then
            source "$dir"
            success "ROS 2 $distro found"
            ROS_SETUP="$dir"
            break
        fi
    fi
done
if [ -z "$ROS_SETUP" ]; then
    error "No ROS 2 installation found in /opt/ros/"
    error "Please install ROS 2 (Galactic/Humble) first."
    exit 1
fi

# ---------------------------------------------------------------------------
# Step 2: Check Jetson platform
# ---------------------------------------------------------------------------
info "Step 2/7: Checking Jetson platform..."

if [ -f /etc/nv_tegra_release ]; then
    L4T_RELEASE=$(head -1 /etc/nv_tegra_release | grep -oP 'R\d+' | head -1)
    success "Jetson platform detected: L4T ${L4T_RELEASE}"

    # Check architecture
    ARCH=$(uname -m)
    if [ "$ARCH" != "aarch64" ]; then
        warn "Architecture is $ARCH, expected aarch64 for Jetson"
    else
        success "Architecture: aarch64 (Jetson compatible)"
    fi
else
    warn "/etc/nv_tegra_release not found. This may not be a Jetson device."
    warn "The SDK is designed for Jetson. PC users: use scripts/install_deps.sh instead."
fi

# ---------------------------------------------------------------------------
# Step 3: Install system dependencies
# ---------------------------------------------------------------------------
info "Step 3/7: Installing system dependencies..."

sudo apt-get update -qq

# Required dependencies
sudo apt-get install -y -qq \
    libusb-1.0-0-dev \
    can-utils \
    2>&1 | tail -1

success "System dependencies installed"

# ---------------------------------------------------------------------------
# Step 4: Install libcontrolcan.so
# ---------------------------------------------------------------------------
info "Step 4/7: Installing libcontrolcan.so..."

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_PATH="${SCRIPT_DIR}/../lib/libcontrolcan.so"

if [ -f "$LIB_PATH" ]; then
    sudo cp "$LIB_PATH" /usr/local/lib/
    sudo ldconfig
    success "libcontrolcan.so installed to /usr/local/lib/"
else
    error "libcontrolcan.so not found at $LIB_PATH"
    exit 1
fi

# ---------------------------------------------------------------------------
# Step 5: Create udev rule for ZLG USBCAN2
# ---------------------------------------------------------------------------
info "Step 5/7: Creating udev rule for ZLG USBCAN2..."

UDEV_RULE_FILE="/etc/udev/rules.d/99-zlg-usbcan2.rules"

# NOTE: Replace VID:PID with your device's actual values.
# Run 'lsusb' to find the ZLG USBCAN2 device and get its vendor:product ID.
# Common ZLG USBCAN2 VID:PID is 04D8:0053 or similar.
if [ ! -f "$UDEV_RULE_FILE" ]; then
    echo '# ZLG USBCAN2 — allow user access' | sudo tee "$UDEV_RULE_FILE" > /dev/null
    echo '# Run "lsusb" to find the actual VID:PID of your device and update below:' | sudo tee -a "$UDEV_RULE_FILE" > /dev/null
    echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="04d8", ATTR{idProduct}=="0053", MODE="0666"' | sudo tee -a "$UDEV_RULE_FILE" > /dev/null
    echo 'SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", MODE="0666"' | sudo tee -a "$UDEV_RULE_FILE" > /dev/null

    sudo udevadm control --reload-rules
    sudo udevadm trigger
    success "udev rule created. Please verify VID/PID with 'lsusb' if device is not recognized."
else
    success "udev rule already exists"
fi

# ---------------------------------------------------------------------------
# Step 6: Build the ROS 2 package
# ---------------------------------------------------------------------------
info "Step 6/7: Building the ROS 2 package..."

WS_DIR="${HOME}/can_ws"
PKG_SRC_DIR="${WS_DIR}/src/can_driver_8030D"

# Create workspace if needed
mkdir -p "${WS_DIR}/src"

# Copy package to workspace (skip if already there)
if [ ! -d "$PKG_SRC_DIR" ]; then
    PKG_ORIGIN="$(cd "${SCRIPT_DIR}/.." && pwd)"
    cp -r "$PKG_ORIGIN" "$PKG_SRC_DIR"
    success "Package copied to ${PKG_SRC_DIR}"
else
    success "Package already exists at ${PKG_SRC_DIR}"
fi

# Build
cd "$WS_DIR"
info "Running colcon build..."
colcon build --packages-select can_driver --symlink-install

success "Package built successfully"

# ---------------------------------------------------------------------------
# Step 7: Final configuration
# ---------------------------------------------------------------------------
info "Step 7/7: Final configuration..."

SETUP_LINE="source ${WS_DIR}/install/setup.bash"

if grep -qF "$SETUP_LINE" ~/.bashrc; then
    success "ROS 2 workspace already sourced in ~/.bashrc"
else
    echo ""
    echo "----------------------------------------"
    read -r -p "Add workspace source to ~/.bashrc? [Y/n] " REPLY
    REPLY=${REPLY:-Y}
    if [[ "$REPLY" =~ ^[Yy]$ ]]; then
        echo "" >> ~/.bashrc
        echo "# CAN Driver 8030D workspace" >> ~/.bashrc
        echo "$SETUP_LINE" >> ~/.bashrc
        success "Added to ~/.bashrc"
    else
        info "Skipped. You can source manually with:"
        info "  source ${WS_DIR}/install/setup.bash"
    fi
fi

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo ""
echo "============================================"
echo "  Setup Complete!"
echo "============================================"
echo ""
echo "  Quick Start:"
echo "    # Start the driver"
echo "    ros2 launch can_driver can_driver_8030.launch.py"
echo ""
echo "    # Send speed command (forward 100 RPM)"
echo "    ros2 topic pub /motor_speed std_msgs/msg/Int16MultiArray \"{data: [100, 100]}\""
echo ""
echo "    # Stop motors"
echo "    ros2 topic pub /motor_speed std_msgs/msg/Int16MultiArray \"{data: [0, 0]}\""
echo ""
echo "    # Emergency stop"
echo "    ros2 topic pub --once /driver std_msgs/msg/Int8 \"{data: 0}\""
echo ""
echo "  Documentation: ${PKG_SRC_DIR}/README.md"
echo ""
