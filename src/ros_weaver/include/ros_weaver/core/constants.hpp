#ifndef ROS_WEAVER_CONSTANTS_HPP
#define ROS_WEAVER_CONSTANTS_HPP

#include <QString>
#include <QColor>

namespace ros_weaver {

/**
 * @brief Centralized constants for the application
 *
 * All magic numbers and configurable values should be defined here
 * for consistency and maintainability.
 */
namespace constants {

// =============================================================================
// Canvas Constants
// =============================================================================

namespace canvas {
  // Grid settings
  constexpr int GRID_SIZE = 20;
  constexpr int MAJOR_GRID_INTERVAL = 5;

  // Zoom settings
  constexpr double ZOOM_MIN = 0.1;
  constexpr double ZOOM_MAX = 5.0;
  constexpr double ZOOM_STEP = 0.1;
  constexpr double ZOOM_DEFAULT = 1.0;

  // Scene size
  constexpr int SCENE_SIZE = 10000;
  constexpr int SCENE_HALF = SCENE_SIZE / 2;

  // Colors (dark theme defaults)
  inline QColor backgroundColor() { return QColor(35, 35, 35); }
  inline QColor gridMinorColor() { return QColor(50, 50, 50); }
  inline QColor gridMajorColor() { return QColor(60, 60, 60); }
}

// =============================================================================
// Block Constants
// =============================================================================

namespace block {
  // Dimensions
  constexpr int WIDTH = 180;
  constexpr int MIN_HEIGHT = 60;
  constexpr int HEADER_HEIGHT = 30;
  constexpr int CORNER_RADIUS = 8;

  // Pin settings
  constexpr int PIN_RADIUS = 6;
  constexpr int PIN_HEIGHT = 25;
  constexpr int PIN_HIT_RADIUS = 12;  // Larger for easier clicking

  // Colors
  inline QColor headerGradientStart() { return QColor(42, 130, 218); }
  inline QColor headerGradientEnd() { return QColor(30, 100, 180); }
  inline QColor bodyGradientStart() { return QColor(70, 70, 70); }
  inline QColor bodyGradientEnd() { return QColor(50, 50, 50); }
  inline QColor selectedBorder() { return QColor(42, 130, 218); }
  inline QColor normalBorder() { return QColor(80, 80, 80); }
  inline QColor shadowColor() { return QColor(0, 0, 0, 50); }
}

// =============================================================================
// Pin Type Colors
// =============================================================================

namespace pin_colors {
  inline QColor topic() { return QColor(100, 200, 100); }    // Green
  inline QColor service() { return QColor(100, 150, 255); }  // Blue
  inline QColor action() { return QColor(255, 180, 100); }   // Orange
  inline QColor parameter() { return QColor(200, 100, 200); } // Purple
  inline QColor unknown() { return QColor(180, 180, 180); }   // Gray
}

// =============================================================================
// Connection Line Constants
// =============================================================================

namespace connection {
  constexpr int LINE_WIDTH = 2;
  constexpr int SELECTED_WIDTH = 3;
  constexpr int ANIMATION_PARTICLE_SIZE = 4;
  constexpr int ANIMATION_SPEED_MS = 50;
}

// =============================================================================
// Timing Constants
// =============================================================================

namespace timing {
  // Auto-save
  constexpr int AUTO_SAVE_INTERVAL_MS = 5 * 60 * 1000;  // 5 minutes
  constexpr int AUTO_SAVE_MIN_INTERVAL_MS = 30 * 1000;  // 30 seconds minimum

  // Log processing
  constexpr int LOG_QUEUE_PROCESS_INTERVAL_MS = 50;
  constexpr int LOG_QUEUE_BATCH_SIZE = 100;

  // Status updates
  constexpr int ROS_STATUS_CHECK_INTERVAL_MS = 1500;
  constexpr int OLLAMA_STATUS_CHECK_INTERVAL_MS = 5000;

  // System scan
  constexpr int SYSTEM_SCAN_TIMEOUT_MS = 10000;
  constexpr int AUTO_SCAN_INTERVAL_MS = 30000;

  // UI animations
  constexpr int TOAST_DEFAULT_DURATION_MS = 4000;
  constexpr int TOAST_FADE_DURATION_MS = 200;
  constexpr int TYPING_INDICATOR_INTERVAL_MS = 150;
}

// =============================================================================
// Limits
// =============================================================================

namespace limits {
  constexpr int MAX_LOG_ENTRIES = 10000;
  constexpr int MAX_RECENT_PROJECTS = 10;
  constexpr int MAX_COMMAND_HISTORY = 20;
  constexpr int MAX_UNDO_STACK = 100;
  constexpr int MAX_TOAST_NOTIFICATIONS = 5;

  // File sizes
  constexpr qint64 MAX_TEXT_FILE_SIZE = 1024 * 1024;       // 1MB
  constexpr qint64 MAX_IMAGE_FILE_SIZE = 10 * 1024 * 1024;  // 10MB
  constexpr int MAX_TEXT_CONTENT_LENGTH = 50000;
}

// =============================================================================
// UI Dimensions
// =============================================================================

namespace ui {
  // Main window
  constexpr int MIN_WINDOW_WIDTH = 1200;
  constexpr int MIN_WINDOW_HEIGHT = 800;

  // Notifications
  constexpr int TOAST_WIDTH = 350;
  constexpr int TOAST_MARGIN = 20;
  constexpr int TOAST_SPACING = 10;

  // Panels
  constexpr int DEFAULT_DOCK_WIDTH = 300;
  constexpr int MIN_DOCK_WIDTH = 200;

  // Buttons
  constexpr int BUTTON_HEIGHT = 32;
  constexpr int SMALL_BUTTON_HEIGHT = 24;
  constexpr int ICON_BUTTON_SIZE = 32;
}

// =============================================================================
// Keyboard Shortcuts (as strings for QKeySequence)
// =============================================================================

namespace shortcuts {
  inline QString focusChat() { return "Ctrl+J"; }
  inline QString commandPalette() { return "Ctrl+Shift+P"; }
  inline QString fitToContents() { return "F"; }
  inline QString resetZoom() { return "Ctrl+0"; }
  inline QString scanSystem() { return "Ctrl+Shift+R"; }
  inline QString generateCode() { return "Ctrl+G"; }
  inline QString buildWorkspace() { return "Ctrl+B"; }
  inline QString openSettings() { return "Ctrl+,"; }
}

// =============================================================================
// Application Info
// =============================================================================

namespace app {
  inline QString name() { return "ROS Weaver"; }
  inline QString organization() { return "ROS Weaver"; }
  inline QString version() { return "1.0.0"; }
  inline QString projectExtension() { return ".rwproj"; }
  inline QString autoSaveExtension() { return ".rwproj.autosave"; }
}

}  // namespace constants
}  // namespace ros_weaver

#endif  // ROS_WEAVER_CONSTANTS_HPP
