# EKF Analyzer CLI Tool

A command-line interface for analyzing robot_localization EKF performance on rosbag files without the GUI.

## Overview

The `ekf_analyzer` tool provides terminal-based EKF analysis capabilities, allowing you to:

- Evaluate EKF configuration against recorded rosbag data
- Compute trajectory metrics (ATE, RPE, yaw drift)
- Run parameter sweeps to find optimal settings
- Output results in text, JSON, or CSV formats
- Integrate with CI/CD pipelines and automation scripts

## Installation

The tool is built automatically with the ros_weaver package:

```bash
cd ~/ros2_ws
colcon build --packages-select ros_weaver
source install/setup.bash
```

## Quick Start

### Basic Analysis

```bash
ros2 run ros_weaver ekf_analyzer /path/to/bag.mcap -c ekf_config.yaml
```

### Output to JSON File

```bash
ros2 run ros_weaver ekf_analyzer bag.mcap -c config.yaml -o results.json -f json
```

### Parameter Sweep

```bash
ros2 run ros_weaver ekf_analyzer bag.mcap -c config.yaml --sweep \
    --sweep-param "processNoiseCovariance.0:0.01:0.5:5" \
    -o sweep_results.csv -f csv
```

## Command-Line Options

| Option | Description |
|--------|-------------|
| `bag` (positional) | Path to input rosbag2 file (.mcap or .db3) |
| `-c, --config FILE` | EKF YAML config file (robot_localization format) - **required** |
| `-o, --output FILE` | Output file path (stdout if omitted) |
| `-f, --format FORMAT` | Output format: `text`, `json`, `csv` (default: text) |
| `-g, --ground-truth TOPIC` | Ground truth topic (default: /ground_truth/odom) |
| `-r, --rate RATE` | Playback rate, 0 = max speed (default: 0) |
| `--sweep` | Enable parameter sweep mode |
| `--sweep-param SPEC` | Sweep parameter specification (can be used multiple times) |
| `--verbose` | Verbose output with progress messages |
| `-h, --help` | Show help message |
| `-v, --version` | Show version information |

## Output Formats

### Text (default)

Human-readable format for terminal viewing:

```
=== EKF Analysis Results ===
Bag: /path/to/bag.mcap

--- Absolute Trajectory Error (ATE) ---
RMSE:    0.0432 m
Mean:    0.0387 m
Median:  0.0356 m
Std:     0.0189 m
Min:     0.0012 m
Max:     0.1234 m

--- Relative Pose Error (RPE) ---
RMSE:    0.0156 m
Mean:    0.0134 m
Median:  0.0121 m
Std:     0.0078 m

--- Additional Metrics ---
Yaw drift:          0.0023 deg/m
Trajectory length:  125.67 m
Pose count:         3782
Ground truth:       yes
```

### JSON

Machine-readable format for programmatic processing:

```json
{
  "ate": {
    "rmse": 0.0432,
    "mean": 0.0387,
    "median": 0.0356,
    "std": 0.0189,
    "min": 0.0012,
    "max": 0.1234
  },
  "rpe": {
    "rmse": 0.0156,
    "mean": 0.0134,
    "median": 0.0121,
    "std": 0.0078
  },
  "yaw_drift": 0.0023,
  "trajectory_length": 125.67,
  "pose_count": 3782,
  "has_ground_truth": true,
  "bag_path": "/path/to/bag.mcap"
}
```

### CSV

Tabular format for parameter sweeps:

```csv
Index,processNoiseCovariance.0,ATE_RMSE,ATE_Mean,RPE_RMSE,RPE_Mean,Yaw_Drift,Trajectory_Length,Pose_Count
0,0.01,0.0432,0.0387,0.0156,0.0134,0.0023,125.67,3782
1,0.1325,0.0398,0.0352,0.0143,0.0125,0.0021,125.67,3782
2,0.255,0.0415,0.0368,0.0151,0.0131,0.0022,125.67,3782
3,0.3775,0.0456,0.0401,0.0168,0.0145,0.0025,125.67,3782
4,0.5,0.0512,0.0445,0.0189,0.0162,0.0028,125.67,3782
```

## Parameter Sweep Mode

The parameter sweep mode automatically tests multiple parameter configurations and identifies the best settings.

### Sweep Parameter Specification

Format: `name:min:max:steps[:log]`

| Component | Description |
|-----------|-------------|
| `name` | Parameter path (e.g., `processNoiseCovariance.0`) |
| `min` | Minimum value to test |
| `max` | Maximum value to test |
| `steps` | Number of values to test |
| `log` | Optional: use logarithmic scale |

### Examples

Linear sweep of process noise for X position:
```bash
--sweep-param "processNoiseCovariance.0:0.01:0.5:5"
```

Logarithmic sweep for better coverage of small values:
```bash
--sweep-param "processNoiseCovariance.6:0.001:0.1:10:log"
```

Multiple parameters (Cartesian product):
```bash
--sweep-param "processNoiseCovariance.0:0.01:0.1:3" \
--sweep-param "processNoiseCovariance.5:0.01:0.1:3"
```

### Available Parameters

Common parameters to sweep:

| Parameter | Description |
|-----------|-------------|
| `processNoiseCovariance.0-14` | Process noise for each state (X, Y, Z, roll, pitch, yaw, VX, VY, VZ, Vroll, Vpitch, Vyaw, AX, AY, AZ) |
| `frequency` | EKF update frequency |

## Exit Codes

| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | Invalid arguments |
| 2 | Config file error |
| 3 | Input bag error |
| 4 | robot_localization not installed |
| 5 | Simulation failed |
| 6 | Output write error |

## Configuration File Format

The tool uses the standard robot_localization YAML format:

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true

    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    odom0: /odom
    odom0_config: [true, true, false,
                   false, false, true,
                   false, false, false,
                   false, false, false,
                   false, false, false]

    imu0: /imu/data
    imu0_config: [false, false, false,
                  true, true, false,
                  false, false, false,
                  false, false, true,
                  false, false, false]

    process_noise_covariance: [0.05, 0.05, 0.06, 0.03, 0.03, 0.06,
                               0.025, 0.025, 0.04, 0.01, 0.01, 0.02,
                               0.01, 0.01, 0.015]
```

## Requirements

- ROS2 (Humble or later)
- robot_localization package installed
- Rosbag with odometry/IMU data and optionally ground truth

Install robot_localization:
```bash
sudo apt install ros-${ROS_DISTRO}-robot-localization
```

## Integration Examples

### CI/CD Pipeline

```yaml
# GitHub Actions example
- name: Run EKF analysis
  run: |
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run ros_weaver ekf_analyzer test_data/robot.mcap \
      -c config/ekf.yaml -o results.json -f json

- name: Check ATE threshold
  run: |
    ATE=$(jq '.ate.rmse' results.json)
    if (( $(echo "$ATE > 0.1" | bc -l) )); then
      echo "ATE RMSE ($ATE) exceeds threshold (0.1)"
      exit 1
    fi
```

### Batch Processing

```bash
#!/bin/bash
# Process multiple bags and aggregate results

for bag in data/*.mcap; do
  name=$(basename "$bag" .mcap)
  ros2 run ros_weaver ekf_analyzer "$bag" \
    -c config/ekf.yaml \
    -o "results/${name}.json" -f json
done

# Combine results
jq -s '.' results/*.json > combined_results.json
```

### Parameter Optimization Script

```bash
#!/bin/bash
# Find optimal process noise settings

ros2 run ros_weaver ekf_analyzer robot_data.mcap \
  -c base_config.yaml \
  --sweep \
  --sweep-param "processNoiseCovariance.0:0.001:0.1:10:log" \
  --sweep-param "processNoiseCovariance.5:0.001:0.1:10:log" \
  -o sweep_results.csv -f csv \
  --verbose

# Find best configuration
best=$(sort -t',' -k3 -n sweep_results.csv | head -2 | tail -1)
echo "Best configuration: $best"
```

## Troubleshooting

### "robot_localization package not found"

Install the package:
```bash
sudo apt install ros-${ROS_DISTRO}-robot-localization
```

### "No ground truth available"

The tool will fall back to relative metrics if no ground truth topic is found. To specify a custom ground truth topic:
```bash
ros2 run ros_weaver ekf_analyzer bag.mcap -c config.yaml -g /mocap/odom
```

### Simulation timeout

For very long bags, the default timeout may not be sufficient. The tool uses a 5-minute watchdog timeout. For longer bags, consider:
- Using a subset of the bag
- Increasing playback rate (`-r 2.0`)

## See Also

- [EKF Tuner Workbench](../README.md) - GUI-based EKF tuning
- [robot_localization documentation](https://docs.ros.org/en/humble/p/robot_localization/)
- [Rosbag Workbench](rosbag_workbench.md) - Recording and playback tools
