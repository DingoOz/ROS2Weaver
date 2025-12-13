#!/bin/bash
#
# ROS2Weaver UI Regression Test Runner
#
# This script sets up the environment and runs the UI regression tests
# in a headless Xvfb environment.
#
# Usage:
#   ./run_tests.sh [pytest-args]
#
# Examples:
#   ./run_tests.sh                    # Run all tests
#   ./run_tests.sh -v                 # Verbose output
#   ./run_tests.sh -k "menu"          # Run only menu tests
#   ./run_tests.sh --collect-only     # List tests without running
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TESTS_DIR="$SCRIPT_DIR"

# Configuration
DISPLAY_NUM=99
XVFB_RESOLUTION="1920x1080x24"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WORKSPACE_PATH="${ROS2WEAVER_WORKSPACE:-$HOME/Programming/ROS2Weaver}"

# Colours for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Colour

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check dependencies
check_dependencies() {
    log_info "Checking dependencies..."
    
    local missing=()
    
    # Check for Xvfb
    if ! command -v Xvfb &> /dev/null; then
        missing+=("xvfb")
    fi
    
    # Check for Python packages
    if ! python3 -c "import pytest" &> /dev/null; then
        missing+=("pytest (pip install pytest)")
    fi
    
    if ! python3 -c "import gi; gi.require_version('Atspi', '2.0')" &> /dev/null; then
        missing+=("python3-pyatspi (apt install python3-pyatspi)")
    fi
    
    if ! python3 -c "import reportlab" &> /dev/null; then
        missing+=("reportlab (pip install reportlab)")
    fi
    
    if ! python3 -c "import PIL" &> /dev/null; then
        missing+=("Pillow (pip install Pillow)")
    fi
    
    # Check for AT-SPI2 registry
    if ! command -v at-spi2-registryd &> /dev/null; then
        missing+=("at-spi2-core (apt install at-spi2-core)")
    fi
    
    # Check for scrot (screenshot fallback)
    if ! command -v scrot &> /dev/null; then
        log_warn "scrot not found - screenshots may use alternative methods"
    fi
    
    if [ ${#missing[@]} -gt 0 ]; then
        log_error "Missing dependencies:"
        for dep in "${missing[@]}"; do
            echo "  - $dep"
        done
        echo ""
        echo "Install system dependencies with:"
        echo "  sudo apt install xvfb python3-pyatspi at-spi2-core scrot"
        echo ""
        echo "Install Python dependencies with:"
        echo "  pip install -r $PROJECT_ROOT/requirements-test.txt"
        exit 1
    fi
    
    log_info "All dependencies satisfied"
}

# Start Xvfb if not running
start_xvfb() {
    if [ -e "/tmp/.X${DISPLAY_NUM}-lock" ]; then
        log_info "Xvfb already running on :$DISPLAY_NUM"
        return 0
    fi
    
    log_info "Starting Xvfb on :$DISPLAY_NUM..."
    Xvfb :$DISPLAY_NUM -screen 0 $XVFB_RESOLUTION &
    XVFB_PID=$!
    
    # Wait for Xvfb to start
    sleep 1
    
    if ! kill -0 $XVFB_PID 2>/dev/null; then
        log_error "Failed to start Xvfb"
        exit 1
    fi
    
    log_info "Xvfb started (PID: $XVFB_PID)"
}

# Stop Xvfb
stop_xvfb() {
    if [ -n "$XVFB_PID" ]; then
        log_info "Stopping Xvfb..."
        kill $XVFB_PID 2>/dev/null || true
        wait $XVFB_PID 2>/dev/null || true
    fi
}

# Start AT-SPI2 registry
start_atspi() {
    log_info "Ensuring AT-SPI2 registry is running..."
    
    # Start the AT-SPI2 bus
    export $(dbus-launch)
    
    # Start the registry if not running
    if ! pgrep -x "at-spi2-registryd" > /dev/null; then
        /usr/libexec/at-spi2-registryd &
        sleep 0.5
    fi
}

# Set up environment
setup_environment() {
    log_info "Setting up environment..."
    
    export DISPLAY=:$DISPLAY_NUM
    export QT_LINUX_ACCESSIBILITY_ALWAYS_ON=1
    export QT_ACCESSIBILITY=1
    export QT_QPA_PLATFORM=xcb
    
    # Disable GPU acceleration for headless
    export QT_XCB_GL_INTEGRATION=none
    export LIBGL_ALWAYS_SOFTWARE=1
    
    # Source ROS2
    if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        log_info "Sourcing ROS2 $ROS_DISTRO..."
        source "/opt/ros/$ROS_DISTRO/setup.bash"
    else
        log_error "ROS2 $ROS_DISTRO not found at /opt/ros/$ROS_DISTRO"
        exit 1
    fi
    
    # Source workspace
    if [ -f "$WORKSPACE_PATH/install/setup.bash" ]; then
        log_info "Sourcing workspace at $WORKSPACE_PATH..."
        source "$WORKSPACE_PATH/install/setup.bash"
    else
        log_warn "Workspace not found at $WORKSPACE_PATH - tests may fail"
        log_warn "Set ROS2WEAVER_WORKSPACE environment variable if workspace is elsewhere"
    fi
    
    # Add tests to Python path
    export PYTHONPATH="$TESTS_DIR:$PYTHONPATH"
}

# Run tests
run_tests() {
    log_info "Running UI regression tests..."
    echo ""
    
    cd "$TESTS_DIR"
    
    # Default pytest arguments
    local pytest_args=(
        "-v"                          # Verbose
        "--tb=short"                  # Short traceback
        "--timeout=60"                # Per-test timeout
        "-p" "no:cacheprovider"       # Disable cache
    )
    
    # Add user-provided arguments
    pytest_args+=("$@")
    
    # Run pytest
    python3 -m pytest "${pytest_args[@]}" || TEST_EXIT_CODE=$?
    
    echo ""
    return ${TEST_EXIT_CODE:-0}
}

# Cleanup function
cleanup() {
    log_info "Cleaning up..."
    stop_xvfb
    
    # Kill any leftover ros_weaver processes
    pkill -f "ros_weaver" 2>/dev/null || true
}

# Set trap for cleanup
trap cleanup EXIT

# Main
main() {
    echo "========================================"
    echo "ROS2Weaver UI Regression Test Suite"
    echo "========================================"
    echo ""
    
    check_dependencies
    start_xvfb
    start_atspi
    setup_environment
    
    echo ""
    echo "Environment:"
    echo "  DISPLAY=$DISPLAY"
    echo "  ROS_DISTRO=$ROS_DISTRO"
    echo "  WORKSPACE=$WORKSPACE_PATH"
    echo ""
    
    run_tests "$@"
    exit_code=$?
    
    echo ""
    if [ $exit_code -eq 0 ]; then
        log_info "All tests passed!"
    else
        log_error "Some tests failed (exit code: $exit_code)"
    fi
    
    echo ""
    log_info "Test reports are in: $PROJECT_ROOT/tests/baselines/"
    
    exit $exit_code
}

main "$@"
