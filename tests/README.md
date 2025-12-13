# ROS2Weaver UI Regression Tests

Automated UI smoke testing for ROS2Weaver using AT-SPI2 accessibility APIs.

## Overview

This test framework verifies that:
- All menu items can be clicked without crashes or hangs
- All toolbar and panel buttons respond correctly
- Keyboard shortcuts don't cause crashes
- The canvas responds to basic interactions

Tests run headlessly using Xvfb and generate a PDF report with screenshots.

## Requirements

### System Dependencies

```bash
sudo apt install -y \
    xvfb \
    python3-pyatspi \
    at-spi2-core \
    scrot \
    imagemagick
```

### Python Dependencies

```bash
pip install -r requirements-test.txt
```

## Running Tests

### Quick Start

```bash
# Make the runner executable
chmod +x tests/ui/run_tests.sh

# Run all tests
./tests/ui/run_tests.sh

# Run with verbose output
./tests/ui/run_tests.sh -v

# Run only menu tests
./tests/ui/run_tests.sh -k "menu"

# Run only button tests
./tests/ui/run_tests.sh -k "button"
```

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROS_DISTRO` | `humble` | ROS2 distribution name |
| `ROS2WEAVER_WORKSPACE` | `~/ROS2Weaver` | Path to ROS2Weaver workspace |

### Example

```bash
export ROS2WEAVER_WORKSPACE=/home/user/ros2_ws/src/ROS2Weaver
export ROS_DISTRO=humble
./tests/ui/run_tests.sh
```

## Test Structure

```
tests/
├── baselines/           # Test run outputs (screenshots, reports)
│   └── run_YYYYMMDD_HHMMSS/
│       ├── *.png        # Screenshots
│       └── *.pdf        # Test report
├── ui/
│   ├── conftest.py      # Pytest fixtures and configuration
│   ├── run_tests.sh     # Test runner script
│   ├── test_menus.py    # Menu interaction tests
│   ├── test_panels.py   # Panel and button tests
│   ├── test_canvas.py   # Canvas interaction tests
│   └── utils/
│       ├── accessibility.py  # AT-SPI2 helpers
│       ├── screenshot.py     # Screenshot capture
│       └── report.py         # PDF report generation
└── requirements-test.txt
```

## Test Reports

After each test run, a PDF report is generated in `tests/baselines/run_*/`.

The report includes:
- Summary of pass/fail counts
- For each test:
  - Screenshot taken before the interaction
  - Pass/fail status
  - Error details if failed
  - Duration

## How It Works

1. **Xvfb** creates a virtual display for headless operation
2. **AT-SPI2** provides accessibility APIs to discover and interact with Qt widgets
3. Tests enumerate all menus, buttons, and panels
4. Each widget is clicked, and the app is monitored for crashes/hangs
5. Screenshots are captured before each interaction
6. Results are collected and rendered to PDF

## Customising Tests

### Adding New Tests

Create a new test file in `tests/ui/`:

```python
import pytest
from .utils import find_buttons, click_widget

class TestMyFeature:
    @pytest.fixture(autouse=True)
    def setup(self, result_collector):
        result_collector.start_suite('My Feature')
    
    def test_my_button(self, app, app_accessible, screenshot_session, 
                       result_collector, ui_assert):
        screenshot_path = screenshot_session.capture_before_action('my_button')
        
        # Find and click button
        buttons = find_buttons(app_accessible)
        my_btn = next((b for b in buttons if 'my' in b.name.lower()), None)
        
        if my_btn:
            click_widget(my_btn)
            ui_assert.assert_app_running(app, "clicking my button")
        
        result_collector.add_result(
            test_name='my_button',
            widget_name='My Button',
            widget_type='Button',
            passed=True,
            screenshot_path=screenshot_path
        )
```

### Adjusting Timeouts

Edit `conftest.py`:

```python
APP_LAUNCH_TIMEOUT = 15.0  # Time to wait for app to start
INTERACTION_TIMEOUT = 5.0   # Time to wait for each interaction
```

## Troubleshooting

### "Application not found"

- Ensure ros_weaver builds and runs manually first
- Check `ROS2WEAVER_WORKSPACE` points to correct location
- Verify ROS2 is sourced correctly

### "AT-SPI2 not available"

```bash
# Ensure AT-SPI2 is installed and running
sudo apt install at-spi2-core
/usr/libexec/at-spi2-registryd &
```

### No widgets discovered

Qt accessibility may need to be explicitly enabled. The test runner sets:
- `QT_LINUX_ACCESSIBILITY_ALWAYS_ON=1`
- `QT_ACCESSIBILITY=1`

If still not working, try adding to ros_weaver's main():
```cpp
QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
```

### Screenshots are black

Ensure Xvfb is running with sufficient colour depth:
```bash
Xvfb :99 -screen 0 1920x1080x24
```

## Integration with CI

Add to your CI pipeline:

```yaml
test:
  script:
    - apt-get update && apt-get install -y xvfb python3-pyatspi at-spi2-core scrot
    - pip install -r tests/requirements-test.txt
    - ./tests/ui/run_tests.sh
  artifacts:
    paths:
      - tests/baselines/
    when: always
```

## Licence

Same as ROS2Weaver (GPL-3.0).
