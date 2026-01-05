#ifndef ROS_WEAVER_WIDGETS_MINIMAP_PANEL_HPP
#define ROS_WEAVER_WIDGETS_MINIMAP_PANEL_HPP

#include <QWidget>
#include <QTimer>
#include <QRectF>
#include <QPointF>

namespace ros_weaver {

class WeaverCanvas;

/**
 * @brief Mini-map navigation panel for large canvas navigation
 *
 * Provides a bird's-eye view of the entire canvas with:
 * - Thumbnail view of all nodes and connections
 * - Viewport rectangle showing current view area
 * - Click-to-navigate functionality
 * - Drag viewport rectangle to pan
 */
class MinimapPanel : public QWidget {
  Q_OBJECT

public:
  explicit MinimapPanel(QWidget* parent = nullptr);
  ~MinimapPanel() override;

  /**
   * @brief Set the canvas to display in the minimap
   */
  void setCanvas(WeaverCanvas* canvas);

  /**
   * @brief Get the associated canvas
   */
  WeaverCanvas* canvas() const { return canvas_; }

public slots:
  /**
   * @brief Force a refresh of the minimap
   */
  void refresh();

protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

private slots:
  void onCanvasChanged();
  void onRefreshTimer();

private:
  void updateTransform();
  QPointF mapToCanvas(const QPointF& widgetPos) const;
  QPointF mapFromCanvas(const QPointF& canvasPos) const;
  QRectF getViewportRect() const;
  void centerCanvasOn(const QPointF& canvasPos);

  WeaverCanvas* canvas_;

  // Transform from canvas to minimap coordinates
  qreal scale_;
  QPointF offset_;

  // Cached content bounds
  QRectF contentBounds_;

  // Interaction state
  bool isDragging_;
  QPointF dragStartPos_;
  QPointF dragStartViewport_;

  // Refresh timer for periodic updates
  QTimer* refreshTimer_;

  // Visual settings
  static constexpr int MARGIN = 5;
  static constexpr int MIN_SIZE = 100;
  static constexpr int DEFAULT_WIDTH = 200;
  static constexpr int DEFAULT_HEIGHT = 150;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_MINIMAP_PANEL_HPP
