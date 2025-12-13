"""
Screenshot capture utilities for UI regression testing.

Supports headless capture via Xvfb and stores screenshots in the baselines
directory for inclusion in test reports.
"""

import os
import subprocess
import time
from pathlib import Path
from typing import Optional, Tuple
from datetime import datetime

# Try multiple screenshot backends
try:
    import pyscreenshot as ImageGrab
    SCREENSHOT_BACKEND = 'pyscreenshot'
except ImportError:
    try:
        from PIL import ImageGrab
        SCREENSHOT_BACKEND = 'pillow'
    except ImportError:
        SCREENSHOT_BACKEND = 'scrot'


class ScreenshotError(Exception):
    """Raised when screenshot capture fails."""
    pass


def get_screenshot_dir() -> Path:
    """Get the directory for storing screenshots."""
    # Look for tests/baselines relative to this file (utils is inside tests)
    tests_dir = Path(__file__).parent.parent
    baselines_dir = tests_dir / 'baselines'
    baselines_dir.mkdir(parents=True, exist_ok=True)
    return baselines_dir


def get_run_dir() -> Path:
    """Get a timestamped directory for this test run's screenshots."""
    baselines = get_screenshot_dir()
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = baselines / f'run_{timestamp}'
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def sanitise_filename(name: str) -> str:
    """Convert a test/widget name to a safe filename."""
    # Replace problematic characters
    safe = name.replace(' ', '_')
    safe = safe.replace('/', '_')
    safe = safe.replace('\\', '_')
    safe = safe.replace(':', '_')
    safe = safe.replace('>', '_')
    safe = safe.replace('<', '_')
    safe = safe.replace('|', '_')
    safe = safe.replace('"', '_')
    safe = safe.replace("'", '_')
    safe = safe.replace('?', '_')
    safe = safe.replace('*', '_')
    
    # Remove consecutive underscores
    while '__' in safe:
        safe = safe.replace('__', '_')
    
    # Trim and limit length
    safe = safe.strip('_')[:100]
    
    return safe if safe else 'unnamed'


def capture_screenshot(
    filename: Optional[str] = None,
    output_dir: Optional[Path] = None,
    region: Optional[Tuple[int, int, int, int]] = None
) -> Path:
    """
    Capture a screenshot of the current display.
    
    Args:
        filename: Base filename (without extension). Auto-generated if None.
        output_dir: Directory to save screenshot. Uses run dir if None.
        region: Optional (x, y, width, height) region to capture.
        
    Returns:
        Path to the saved screenshot file.
        
    Raises:
        ScreenshotError: If capture fails.
    """
    if output_dir is None:
        output_dir = get_run_dir()
    
    if filename is None:
        timestamp = datetime.now().strftime('%H%M%S_%f')
        filename = f'screenshot_{timestamp}'
    
    filename = sanitise_filename(filename)
    output_path = output_dir / f'{filename}.png'
    
    try:
        if SCREENSHOT_BACKEND in ('pyscreenshot', 'pillow'):
            if region:
                # Convert (x, y, w, h) to (left, top, right, bottom)
                bbox = (region[0], region[1], region[0] + region[2], region[1] + region[3])
                img = ImageGrab.grab(bbox=bbox)
            else:
                img = ImageGrab.grab()
            img.save(str(output_path))
        else:
            # Fallback to scrot command
            cmd = ['scrot', str(output_path)]
            if region:
                # scrot uses -a for area: x,y,w,h
                cmd.extend(['-a', f'{region[0]},{region[1]},{region[2]},{region[3]}'])
            
            result = subprocess.run(cmd, capture_output=True, timeout=10)
            if result.returncode != 0:
                raise ScreenshotError(f"scrot failed: {result.stderr.decode()}")
        
        if not output_path.exists():
            raise ScreenshotError(f"Screenshot file not created: {output_path}")
            
        return output_path
        
    except subprocess.TimeoutExpired:
        raise ScreenshotError("Screenshot capture timed out")
    except Exception as e:
        raise ScreenshotError(f"Screenshot capture failed: {e}")


def capture_window(
    window_name: str,
    filename: Optional[str] = None,
    output_dir: Optional[Path] = None
) -> Path:
    """
    Capture a screenshot of a specific window by name.
    
    Args:
        window_name: Name/title of the window to capture.
        filename: Base filename. Uses window name if None.
        output_dir: Directory to save screenshot.
        
    Returns:
        Path to the saved screenshot file.
    """
    if filename is None:
        filename = sanitise_filename(window_name)
    
    if output_dir is None:
        output_dir = get_run_dir()
    
    output_path = output_dir / f'{sanitise_filename(filename)}.png'
    
    try:
        # Use import (ImageMagick) with window name
        # First, find the window ID
        result = subprocess.run(
            ['xdotool', 'search', '--name', window_name],
            capture_output=True,
            timeout=5
        )
        
        if result.returncode == 0 and result.stdout.strip():
            window_id = result.stdout.decode().strip().split('\n')[0]
            
            # Use import to capture specific window
            subprocess.run(
                ['import', '-window', window_id, str(output_path)],
                capture_output=True,
                timeout=10
            )
            
            if output_path.exists():
                return output_path
        
        # Fallback to full screen capture
        return capture_screenshot(filename, output_dir)
        
    except Exception:
        # Ultimate fallback
        return capture_screenshot(filename, output_dir)


class ScreenshotSession:
    """
    Manages screenshots for a single test run.
    
    Provides consistent naming and organisation of screenshots taken
    during test execution.
    """
    
    def __init__(self, test_name: str, run_dir: Optional[Path] = None):
        """
        Initialise a screenshot session.
        
        Args:
            test_name: Name of the test or test suite.
            run_dir: Directory for this test run. Auto-created if None.
        """
        self.test_name = sanitise_filename(test_name)
        self.run_dir = run_dir or get_run_dir()
        self.test_dir = self.run_dir / self.test_name
        self.test_dir.mkdir(parents=True, exist_ok=True)
        self.screenshot_count = 0
        self.screenshots = []
    
    def capture(
        self,
        label: str,
        prefix: Optional[str] = None
    ) -> Path:
        """
        Capture a screenshot with a descriptive label.
        
        Args:
            label: Description of what's being captured (e.g., 'before_click_file_menu')
            prefix: Optional prefix for ordering (e.g., '001')
            
        Returns:
            Path to the captured screenshot.
        """
        self.screenshot_count += 1
        
        if prefix is None:
            prefix = f'{self.screenshot_count:03d}'
        
        filename = f'{prefix}_{sanitise_filename(label)}'
        path = capture_screenshot(filename, self.test_dir)
        
        self.screenshots.append({
            'path': path,
            'label': label,
            'index': self.screenshot_count,
            'timestamp': datetime.now().isoformat()
        })
        
        return path
    
    def capture_before_action(self, action_name: str) -> Path:
        """Capture screenshot before performing an action."""
        return self.capture(f'before_{sanitise_filename(action_name)}')
    
    def capture_after_action(self, action_name: str) -> Path:
        """Capture screenshot after performing an action."""
        return self.capture(f'after_{sanitise_filename(action_name)}')
    
    def get_all_screenshots(self) -> list:
        """Get list of all screenshots taken in this session."""
        return self.screenshots.copy()
    
    def get_screenshot_dir(self) -> Path:
        """Get the directory containing screenshots for this session."""
        return self.test_dir
