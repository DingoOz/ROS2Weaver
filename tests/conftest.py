"""
Pytest configuration and fixtures for ROS2Weaver UI regression testing.

Provides fixtures for:
- Launching ros_weaver in a headless Xvfb environment
- AT-SPI2 accessibility setup
- Screenshot capture session
- Test result collection and PDF report generation
"""

import os
import sys
import time
import signal
import subprocess
from pathlib import Path
from typing import Generator, Optional
from datetime import datetime

import pytest

from .utils import (
    initialise_atspi,
    find_application,
    close_dialogs,
    wait_for_idle,
    AppNotFoundError,
    ScreenshotSession,
    TestResultCollector,
    get_run_dir
)


# Configuration
APP_NAME = 'weaver'  # Match both 'ros_weaver' and 'ROS Weaver'
APP_LAUNCH_TIMEOUT = 20.0  # seconds to wait for app to start
INTERACTION_TIMEOUT = 5.0  # seconds to wait for each interaction
XVFB_DISPLAY = ':99'
XVFB_RESOLUTION = '1920x1080x24'


class XvfbManager:
    """Manages a virtual X framebuffer for headless testing."""

    def __init__(self, display: str = XVFB_DISPLAY, resolution: str = XVFB_RESOLUTION):
        self.display = display
        self.resolution = resolution
        self.process: Optional[subprocess.Popen] = None
        self._using_existing_display = False

    def start(self) -> bool:
        """Start the Xvfb server or use existing display."""
        if self.is_running():
            return True

        # Check if there's an existing display we can use
        existing_display = os.environ.get('DISPLAY')
        if existing_display:
            try:
                # Verify the display works by checking if X server responds
                check_env = os.environ.copy()
                check_env['DISPLAY'] = existing_display
                result = subprocess.run(
                    ['xdpyinfo'],
                    env=check_env,
                    capture_output=True,
                    timeout=5
                )
                if result.returncode == 0:
                    print(f"Using existing display {existing_display}")
                    self.display = existing_display
                    self._using_existing_display = True
                    return True
            except Exception:
                pass  # Fall through to start Xvfb

        # Try to start Xvfb
        try:
            self.process = subprocess.Popen(
                ['Xvfb', self.display, '-screen', '0', self.resolution],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            time.sleep(0.5)  # Give Xvfb time to start
            return self.is_running()
        except FileNotFoundError:
            print("ERROR: Xvfb not found and no existing display. Install with: sudo apt install xvfb")
            return False
        except Exception as e:
            print(f"ERROR: Failed to start Xvfb: {e}")
            return False

    def stop(self):
        """Stop the Xvfb server."""
        if self.process and not self._using_existing_display:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
            self.process = None

    def is_running(self) -> bool:
        """Check if Xvfb is running or using existing display."""
        if self._using_existing_display:
            return True
        if self.process is None:
            return False
        return self.process.poll() is None

    @property
    def env(self) -> dict:
        """Get environment variables for running apps in Xvfb."""
        env = os.environ.copy()
        env['DISPLAY'] = self.display
        # Enable Qt accessibility
        env['QT_LINUX_ACCESSIBILITY_ALWAYS_ON'] = '1'
        env['QT_ACCESSIBILITY'] = '1'
        # Disable GPU for headless
        env['QT_QPA_PLATFORM'] = 'xcb'
        return env


class AppManager:
    """Manages the ros_weaver application process."""
    
    def __init__(self, env: dict):
        self.env = env
        self.process: Optional[subprocess.Popen] = None
        self.app_accessible = None
    
    def start(self, timeout: float = APP_LAUNCH_TIMEOUT) -> bool:
        """
        Start ros_weaver and wait for it to be accessible.
        
        Returns:
            True if app started and is accessible via AT-SPI2.
        """
        try:
            # Source ROS2 and launch the app
            # Adjust the path based on your workspace
            # Use ROS_DISTRO env var, default to jazzy
            ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
            workspace = os.environ.get('ROS2WEAVER_WORKSPACE',
                os.path.expanduser('~/Programming/ROS2Weaver'))
            cmd = [
                'bash', '-c',
                f'source /opt/ros/{ros_distro}/setup.bash && '
                f'source {workspace}/install/setup.bash && '
                'ros2 run ros_weaver ros_weaver'
            ]
            
            self.process = subprocess.Popen(
                cmd,
                env=self.env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group for clean termination
            )
            
            # Wait for app to appear in accessibility tree
            start_time = time.time()
            while time.time() - start_time < timeout:
                try:
                    self.app_accessible = find_application(APP_NAME, timeout=1.0)
                    if self.app_accessible:
                        # Give it a moment to fully initialise
                        time.sleep(1.0)
                        return True
                except AppNotFoundError:
                    pass
                
                # Check if process died
                if self.process.poll() is not None:
                    stdout, stderr = self.process.communicate()
                    print(f"App exited unexpectedly. stderr: {stderr.decode()}")
                    return False
                
                time.sleep(0.5)
            
            return False
            
        except Exception as e:
            print(f"Failed to start app: {e}")
            return False
    
    def stop(self):
        """Stop the ros_weaver application."""
        if self.process:
            try:
                # Send SIGTERM to process group
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=5)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            self.process = None
            self.app_accessible = None
    
    def is_running(self) -> bool:
        """Check if the app is still running."""
        if self.process is None:
            return False
        return self.process.poll() is None
    
    def check_crash(self) -> tuple[bool, str]:
        """
        Check if the app has crashed.
        
        Returns:
            Tuple of (crashed: bool, error_message: str)
        """
        if self.process is None:
            return True, "Process not started"
        
        exit_code = self.process.poll()
        if exit_code is not None:
            stdout, stderr = self.process.communicate()
            return True, f"Process exited with code {exit_code}: {stderr.decode()}"
        
        return False, ""


# Global state for session-scoped resources
_xvfb: Optional[XvfbManager] = None
_result_collector: Optional[TestResultCollector] = None
_run_dir: Optional[Path] = None


def pytest_configure(config):
    """Configure pytest for UI testing."""
    global _result_collector, _run_dir
    
    # Create run directory for this session
    _run_dir = get_run_dir()
    
    # Initialise result collector
    _result_collector = TestResultCollector('ros2weaver_ui_tests')
    _result_collector.set_run_dir(_run_dir)


def pytest_sessionfinish(session, exitstatus):
    """Generate report at end of session."""
    global _result_collector, _run_dir, _xvfb
    
    # Generate PDF report
    if _result_collector and _result_collector.suites:
        try:
            report_path = _result_collector.generate_report(_run_dir)
            print(f"\n{'='*60}")
            print(f"UI Test Report generated: {report_path}")
            print(f"{'='*60}\n")
        except Exception as e:
            print(f"Failed to generate report: {e}")
    
    # Clean up Xvfb
    if _xvfb:
        _xvfb.stop()


@pytest.fixture(scope='session')
def xvfb() -> Generator[XvfbManager, None, None]:
    """Session-scoped Xvfb fixture."""
    global _xvfb
    
    _xvfb = XvfbManager()
    
    if not _xvfb.start():
        pytest.skip("Failed to start Xvfb - is it installed?")
    
    yield _xvfb
    
    # Cleanup handled in pytest_sessionfinish


@pytest.fixture(scope='session')
def atspi_init():
    """Initialise AT-SPI2 accessibility framework."""
    initialise_atspi()


@pytest.fixture(scope='function')
def app(xvfb: XvfbManager, atspi_init) -> Generator[AppManager, None, None]:
    """
    Function-scoped fixture that launches ros_weaver for each test.
    
    Yields:
        AppManager instance with running app.
    """
    app_manager = AppManager(xvfb.env)
    
    if not app_manager.start():
        pytest.fail("Failed to start ros_weaver")
    
    yield app_manager
    
    app_manager.stop()


@pytest.fixture(scope='function')
def app_accessible(app: AppManager):
    """Get the AT-SPI2 accessible for the app."""
    if app.app_accessible is None:
        pytest.fail("App accessible not available")
    return app.app_accessible


@pytest.fixture(scope='session')
def run_dir() -> Path:
    """Get the directory for this test run."""
    global _run_dir
    return _run_dir


@pytest.fixture(scope='session')
def result_collector() -> TestResultCollector:
    """Get the result collector for this session."""
    global _result_collector
    return _result_collector


@pytest.fixture(scope='function')
def screenshot_session(run_dir: Path, request) -> ScreenshotSession:
    """Create a screenshot session for the current test."""
    test_name = request.node.name
    return ScreenshotSession(test_name, run_dir)


@pytest.fixture(scope='function')
def interaction_timeout() -> float:
    """Get the interaction timeout value."""
    return INTERACTION_TIMEOUT


# Pytest hooks for better test names
def pytest_collection_modifyitems(items):
    """Modify test names for better reporting."""
    for item in items:
        # Add UI test marker
        item.add_marker(pytest.mark.ui_test)


# Custom assertions
class UIAssertions:
    """Helper class for UI-specific assertions."""
    
    @staticmethod
    def assert_app_running(app: AppManager, context: str = ""):
        """Assert that the app is still running (no crash)."""
        crashed, error = app.check_crash()
        if crashed:
            pytest.fail(f"App crashed{' during ' + context if context else ''}: {error}")
    
    @staticmethod
    def assert_responsive(app: AppManager, timeout: float = INTERACTION_TIMEOUT):
        """Assert that the app responds within timeout."""
        if app.app_accessible:
            if not wait_for_idle(app.app_accessible, timeout):
                pytest.fail(f"App did not respond within {timeout}s (possible hang)")


@pytest.fixture
def ui_assert() -> UIAssertions:
    """Get UI assertion helpers."""
    return UIAssertions()
