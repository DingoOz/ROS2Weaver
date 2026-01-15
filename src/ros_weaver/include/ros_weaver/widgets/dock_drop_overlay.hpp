#ifndef ROS_WEAVER_DOCK_DROP_OVERLAY_HPP
#define ROS_WEAVER_DOCK_DROP_OVERLAY_HPP

#include <QWidget>
#include <QMainWindow>
#include <QDockWidget>
#include <QMap>
#include <QRect>
#include <QTimer>

namespace ros_weaver {

/**
 * @brief Visual indicator for dock drop zones
 *
 * Shows highlighted areas where a floating dock widget can be docked
 * when the user holds Ctrl while dragging.
 */
class DockDropOverlay : public QWidget {
  Q_OBJECT

public:
  enum class DropArea {
    None,
    Left,
    Right,
    Top,
    Bottom,
    Center,  // Tabify with existing dock
    LeftOuter,
    RightOuter,
    TopOuter,
    BottomOuter
  };

  explicit DockDropOverlay(QMainWindow* mainWindow);
  ~DockDropOverlay() override;

  /**
   * @brief Show the overlay for a dragging dock widget
   */
  void showOverlay(QDockWidget* draggedDock);

  /**
   * @brief Hide the overlay
   */
  void hideOverlay();

  /**
   * @brief Update the highlighted drop zone based on mouse position
   * @return The drop area under the cursor
   */
  DropArea updateDropZone(const QPoint& globalPos);

  /**
   * @brief Get the currently highlighted drop area
   */
  DropArea currentDropArea() const { return currentDropArea_; }

  /**
   * @brief Get the dock widget that would be tabified with (if Center drop)
   */
  QDockWidget* targetDock() const { return targetDock_; }

  /**
   * @brief Perform the actual dock operation
   */
  void performDock(QDockWidget* dockWidget);

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  void calculateDropZones();
  QRect getDropZoneRect(DropArea area) const;
  Qt::DockWidgetArea toDockWidgetArea(DropArea area) const;

  QMainWindow* mainWindow_;
  QDockWidget* draggedDock_ = nullptr;
  QDockWidget* targetDock_ = nullptr;
  DropArea currentDropArea_ = DropArea::None;

  // Drop zone rectangles (in overlay coordinates)
  QMap<DropArea, QRect> dropZones_;

  // Visual settings
  static constexpr int OUTER_ZONE_SIZE = 60;
  static constexpr int INNER_ZONE_SIZE = 80;
};

/**
 * @brief Event filter to enable Ctrl+drag docking
 *
 * When a floating dock widget is being dragged and Ctrl is held,
 * shows a visual overlay indicating where the dock can be dropped.
 */
class DockDragFilter : public QObject {
  Q_OBJECT

public:
  explicit DockDragFilter(QMainWindow* mainWindow, QObject* parent = nullptr);

protected:
  bool eventFilter(QObject* watched, QEvent* event) override;

private slots:
  void onPollTimer();

private:
  void stopTracking();

  QMainWindow* mainWindow_;
  DockDropOverlay* overlay_;
  QDockWidget* draggingDock_ = nullptr;
  QTimer* pollTimer_ = nullptr;
  bool ctrlHeld_ = false;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_DOCK_DROP_OVERLAY_HPP
