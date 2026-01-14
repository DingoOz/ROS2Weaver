#!/bin/bash
#
# Build ROS Weaver as a .deb package
#
# Usage: ./scripts/build-deb.sh [ROS_DISTRO]
#
# Examples:
#   ./scripts/build-deb.sh          # Build for default distro (jazzy)
#   ./scripts/build-deb.sh humble   # Build for ROS2 Humble
#   ./scripts/build-deb.sh jazzy    # Build for ROS2 Jazzy
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ROS_DISTRO="${1:-jazzy}"

echo "=========================================="
echo "Building ROS Weaver .deb package"
echo "ROS Distribution: $ROS_DISTRO"
echo "=========================================="

# Check for required tools
for cmd in dpkg-buildpackage lintian; do
    if ! command -v $cmd &> /dev/null; then
        echo "Error: $cmd is required but not installed."
        echo "Install with: sudo apt-get install devscripts build-essential lintian"
        exit 1
    fi
done

# Check ROS2 is installed
if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "Error: ROS2 $ROS_DISTRO not found at /opt/ros/$ROS_DISTRO"
    exit 1
fi

# Source ROS2
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Get version from package.xml
VERSION=$(grep -oP '(?<=<version>)[^<]+' "$PROJECT_DIR/src/ros_weaver/package.xml")
echo "Package version: $VERSION"

# Create build directory
BUILD_DIR="$PROJECT_DIR/build-deb"
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

# Copy source to build directory
echo "Preparing source..."
cp -r "$PROJECT_DIR"/* "$BUILD_DIR/" 2>/dev/null || true
cp -r "$PROJECT_DIR"/.git "$BUILD_DIR/" 2>/dev/null || true

# Copy debian directory to root
cp -r "$BUILD_DIR/packaging/debian" "$BUILD_DIR/"

# Update control file with actual ROS distro
sed -i "s/\${ROS_DISTRO}/$ROS_DISTRO/g" "$BUILD_DIR/debian/control"

# Update changelog version if needed
CHANGELOG_VERSION=$(head -1 "$BUILD_DIR/debian/changelog" | grep -oP '\(\K[^)]+')
if [ "$CHANGELOG_VERSION" != "$VERSION-1" ]; then
    echo "Updating changelog version to $VERSION-1"
    sed -i "1s/([^)]*)/($VERSION-1)/" "$BUILD_DIR/debian/changelog"
fi

# Build the package
cd "$BUILD_DIR"
echo "Building package..."
export ROS_DISTRO
dpkg-buildpackage -us -uc -b

# Move the .deb file to output directory
OUTPUT_DIR="$PROJECT_DIR/dist"
mkdir -p "$OUTPUT_DIR"
mv "$PROJECT_DIR"/*.deb "$OUTPUT_DIR/" 2>/dev/null || true
mv "$PROJECT_DIR"/*.changes "$OUTPUT_DIR/" 2>/dev/null || true
mv "$PROJECT_DIR"/*.buildinfo "$OUTPUT_DIR/" 2>/dev/null || true

# Run lintian (optional, don't fail on warnings)
echo ""
echo "Running lintian..."
lintian "$OUTPUT_DIR"/*.deb --suppress-tags embedded-library || true

echo ""
echo "=========================================="
echo "Build complete!"
echo ""
echo "Output files in: $OUTPUT_DIR"
ls -la "$OUTPUT_DIR"/*.deb 2>/dev/null || echo "No .deb files found"
echo ""
echo "Install with: sudo dpkg -i $OUTPUT_DIR/ros-weaver_${VERSION}-1_*.deb"
echo "=========================================="
