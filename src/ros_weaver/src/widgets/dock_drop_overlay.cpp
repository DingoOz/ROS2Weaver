#include "ros_weaver/widgets/dock_drop_overlay.hpp"
#include "ros_weaver/core/theme_manager.hpp"
#include <QPainter>
#include <QPaintEvent>
#include <QApplication>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QTimer>
#include <QDebug>

namespace ros_weaver {

// ============================================================================
// DockDropOverlay
// ============================================================================

DockDropOverlay::DockDropOverlay(QMainWindow* mainWindow)
    : QWidget(mainWindow), mainWindow_(mainWindow) {
  setWindowFlags(Qt::FramelessWindowHint | Qt::Tool | Qt::WindowStaysOnTopHint);
  setAttribute(Qt::WA_TranslucentBackground);
  setAttribute(Qt::WA_TransparentForMouseEvents);
  hide();
}

DockDropOverlay::~DockDropOverlay() = default;

void DockDropOverlay::showOverlay(QDockWidget* draggedDock) {
  draggedDock_ = draggedDock;
  currentDropArea_ = DropArea::None;
  targetDock_ = nullptr;

  // Position overlay over the main window
  QRect mainRect = mainWindow_->geometry();
  setGeometry(mainRect);
  calculateDropZones();

  raise();
  show();
  update();
}

void DockDropOverlay::hideOverlay() {
  hide();
  draggedDock_ = nullptr;
  targetDock_ = nullptr;
  currentDropArea_ = DropArea::None;
}

void DockDropOverlay::calculateDropZones() {
  dropZones_.clear();

  QRect centralRect = mainWindow_->centralWidget()
                          ? mainWindow_->centralWidget()->geometry()
                          : mainWindow_->rect();

  // Outer zones (edges of the main window)
  QRect mainRect = mainWindow_->rect();

  dropZones_[DropArea::LeftOuter] = QRect(
      0, OUTER_ZONE_SIZE,
      OUTER_ZONE_SIZE, mainRect.height() - 2 * OUTER_ZONE_SIZE);

  dropZones_[DropArea::RightOuter] = QRect(
      mainRect.width() - OUTER_ZONE_SIZE, OUTER_ZONE_SIZE,
      OUTER_ZONE_SIZE, mainRect.height() - 2 * OUTER_ZONE_SIZE);

  dropZones_[DropArea::TopOuter] = QRect(
      OUTER_ZONE_SIZE, 0,
      mainRect.width() - 2 * OUTER_ZONE_SIZE, OUTER_ZONE_SIZE);

  dropZones_[DropArea::BottomOuter] = QRect(
      OUTER_ZONE_SIZE, mainRect.height() - OUTER_ZONE_SIZE,
      mainRect.width() - 2 * OUTER_ZONE_SIZE, OUTER_ZONE_SIZE);

  // Inner zones around central widget
  int cx = centralRect.x();
  int cy = centralRect.y();
  int cw = centralRect.width();
  int ch = centralRect.height();

  dropZones_[DropArea::Left] = QRect(cx, cy, INNER_ZONE_SIZE, ch);
  dropZones_[DropArea::Right] = QRect(cx + cw - INNER_ZONE_SIZE, cy, INNER_ZONE_SIZE, ch);
  dropZones_[DropArea::Top] = QRect(cx, cy, cw, INNER_ZONE_SIZE);
  dropZones_[DropArea::Bottom] = QRect(cx, cy + ch - INNER_ZONE_SIZE, cw, INNER_ZONE_SIZE);

  // Center zone for tabifying
  int centerMargin = INNER_ZONE_SIZE + 20;
  dropZones_[DropArea::Center] = QRect(
      cx + centerMargin, cy + centerMargin,
      cw - 2 * centerMargin, ch - 2 * centerMargin);
}

DockDropOverlay::DropArea DockDropOverlay::updateDropZone(const QPoint& globalPos) {
  QPoint localPos = mapFromGlobal(globalPos);
  DropArea newArea = DropArea::None;
  targetDock_ = nullptr;

  // Check outer zones first (higher priority for edge docking)
  if (dropZones_[DropArea::LeftOuter].contains(localPos)) {
    newArea = DropArea::LeftOuter;
  } else if (dropZones_[DropArea::RightOuter].contains(localPos)) {
    newArea = DropArea::RightOuter;
  } else if (dropZones_[DropArea::TopOuter].contains(localPos)) {
    newArea = DropArea::TopOuter;
  } else if (dropZones_[DropArea::BottomOuter].contains(localPos)) {
    newArea = DropArea::BottomOuter;
  }
  // Check inner zones
  else if (dropZones_[DropArea::Left].contains(localPos)) {
    newArea = DropArea::Left;
  } else if (dropZones_[DropArea::Right].contains(localPos)) {
    newArea = DropArea::Right;
  } else if (dropZones_[DropArea::Top].contains(localPos)) {
    newArea = DropArea::Top;
  } else if (dropZones_[DropArea::Bottom].contains(localPos)) {
    newArea = DropArea::Bottom;
  }
  // Check center zone - find existing dock to tabify with
  else if (dropZones_[DropArea::Center].contains(localPos)) {
    // Find a visible dock widget under the cursor
    QPoint mainWindowPos = mainWindow_->mapFromGlobal(globalPos);
    for (QDockWidget* dock : mainWindow_->findChildren<QDockWidget*>()) {
      if (dock != draggedDock_ && dock->isVisible() && !dock->isFloating()) {
        QRect dockRect = dock->geometry();
        if (dockRect.contains(mainWindowPos)) {
          targetDock_ = dock;
          newArea = DropArea::Center;
          break;
        }
      }
    }
  }

  if (newArea != currentDropArea_) {
    currentDropArea_ = newArea;
    update();
  }

  return currentDropArea_;
}

QRect DockDropOverlay::getDropZoneRect(DropArea area) const {
  return dropZones_.value(area, QRect());
}

Qt::DockWidgetArea DockDropOverlay::toDockWidgetArea(DropArea area) const {
  switch (area) {
    case DropArea::Left:
    case DropArea::LeftOuter:
      return Qt::LeftDockWidgetArea;
    case DropArea::Right:
    case DropArea::RightOuter:
      return Qt::RightDockWidgetArea;
    case DropArea::Top:
    case DropArea::TopOuter:
      return Qt::TopDockWidgetArea;
    case DropArea::Bottom:
    case DropArea::BottomOuter:
      return Qt::BottomDockWidgetArea;
    default:
      return Qt::NoDockWidgetArea;
  }
}

void DockDropOverlay::performDock(QDockWidget* dockWidget) {
  if (!dockWidget || currentDropArea_ == DropArea::None) {
    return;
  }

  // Make the dock widget dockable again
  dockWidget->setFloating(false);

  if (currentDropArea_ == DropArea::Center && targetDock_) {
    // Tabify with the target dock
    mainWindow_->tabifyDockWidget(targetDock_, dockWidget);
    dockWidget->show();
    dockWidget->raise();
  } else {
    Qt::DockWidgetArea area = toDockWidgetArea(currentDropArea_);
    if (area != Qt::NoDockWidgetArea) {
      mainWindow_->addDockWidget(area, dockWidget);
      dockWidget->show();
    }
  }
}

void DockDropOverlay::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  auto& theme = ThemeManager::instance();
  QColor highlightColor = theme.primaryColor();
  highlightColor.setAlpha(60);

  QColor borderColor = theme.primaryColor();
  borderColor.setAlpha(200);

  QColor zoneColor = theme.isDarkTheme() ? QColor(255, 255, 255, 30) : QColor(0, 0, 0, 20);

  // Draw all drop zones with subtle background
  for (auto it = dropZones_.begin(); it != dropZones_.end(); ++it) {
    if (it.key() != DropArea::Center) {  // Don't draw center zone background
      painter.fillRect(it.value(), zoneColor);
    }
  }

  // Draw highlighted zone
  if (currentDropArea_ != DropArea::None) {
    QRect highlightRect;

    if (currentDropArea_ == DropArea::Center && targetDock_) {
      // Highlight the target dock widget
      QRect dockRect = targetDock_->geometry();
      highlightRect = dockRect;
    } else {
      highlightRect = getDropZoneRect(currentDropArea_);
    }

    // Fill with highlight color
    painter.fillRect(highlightRect, highlightColor);

    // Draw border
    painter.setPen(QPen(borderColor, 3));
    painter.drawRect(highlightRect.adjusted(1, 1, -1, -1));

    // Draw label
    QString label;
    switch (currentDropArea_) {
      case DropArea::Left:
      case DropArea::LeftOuter:
        label = tr("Dock Left");
        break;
      case DropArea::Right:
      case DropArea::RightOuter:
        label = tr("Dock Right");
        break;
      case DropArea::Top:
      case DropArea::TopOuter:
        label = tr("Dock Top");
        break;
      case DropArea::Bottom:
      case DropArea::BottomOuter:
        label = tr("Dock Bottom");
        break;
      case DropArea::Center:
        label = tr("Tabify");
        break;
      default:
        break;
    }

    if (!label.isEmpty()) {
      painter.setPen(theme.textPrimaryColor());
      painter.setFont(QFont("Sans", 14, QFont::Bold));

      // Draw text with background
      QRect textRect = painter.fontMetrics().boundingRect(label);
      textRect.moveCenter(highlightRect.center());
      textRect.adjust(-10, -5, 10, 5);

      QColor textBgColor = theme.surfaceColor();
      textBgColor.setAlpha(220);
      painter.fillRect(textRect, textBgColor);
      painter.drawText(highlightRect, Qt::AlignCenter, label);
    }
  }

  // Draw corner indicators
  painter.setPen(QPen(theme.borderColor(), 1));
  int cornerSize = 20;
  QRect mainRect = rect();

  // Top-left corner
  painter.drawLine(0, cornerSize, 0, 0);
  painter.drawLine(0, 0, cornerSize, 0);

  // Top-right corner
  painter.drawLine(mainRect.right() - cornerSize, 0, mainRect.right(), 0);
  painter.drawLine(mainRect.right(), 0, mainRect.right(), cornerSize);

  // Bottom-left corner
  painter.drawLine(0, mainRect.bottom() - cornerSize, 0, mainRect.bottom());
  painter.drawLine(0, mainRect.bottom(), cornerSize, mainRect.bottom());

  // Bottom-right corner
  painter.drawLine(mainRect.right() - cornerSize, mainRect.bottom(), mainRect.right(), mainRect.bottom());
  painter.drawLine(mainRect.right(), mainRect.bottom() - cornerSize, mainRect.right(), mainRect.bottom());
}

// ============================================================================
// DockDragFilter
// ============================================================================

DockDragFilter::DockDragFilter(QMainWindow* mainWindow, QObject* parent)
    : QObject(parent), mainWindow_(mainWindow) {
  overlay_ = new DockDropOverlay(mainWindow);

  // Create a timer to poll for floating dock positions and Ctrl key state
  pollTimer_ = new QTimer(this);
  pollTimer_->setInterval(16);  // ~60fps
  connect(pollTimer_, &QTimer::timeout, this, &DockDragFilter::onPollTimer);

  // Install as application-wide event filter to catch key events
  qApp->installEventFilter(this);
}

void DockDragFilter::onPollTimer() {
  if (!draggingDock_) {
    pollTimer_->stop();
    return;
  }

  // Check if Ctrl is still held
  bool ctrlNowHeld = QApplication::keyboardModifiers() & Qt::ControlModifier;

  if (ctrlNowHeld && !ctrlHeld_) {
    // Ctrl was just pressed
    ctrlHeld_ = true;
    overlay_->showOverlay(draggingDock_);
  } else if (!ctrlNowHeld && ctrlHeld_) {
    // Ctrl was just released - perform dock if over a valid zone
    if (overlay_->currentDropArea() != DockDropOverlay::DropArea::None) {
      overlay_->performDock(draggingDock_);
    }
    ctrlHeld_ = false;
    overlay_->hideOverlay();
  }

  if (ctrlHeld_) {
    overlay_->updateDropZone(QCursor::pos());
  }

  // Check if dock is still floating and being dragged
  if (!draggingDock_->isFloating()) {
    // Dock was re-docked through normal means
    stopTracking();
  }
}

void DockDragFilter::stopTracking() {
  pollTimer_->stop();
  overlay_->hideOverlay();
  draggingDock_ = nullptr;
  ctrlHeld_ = false;
}

bool DockDragFilter::eventFilter(QObject* watched, QEvent* event) {
  // Handle key events globally
  if (event->type() == QEvent::KeyPress) {
    QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
    if (keyEvent->key() == Qt::Key_Control && draggingDock_ && draggingDock_->isFloating()) {
      ctrlHeld_ = true;
      overlay_->showOverlay(draggingDock_);
    }
  } else if (event->type() == QEvent::KeyRelease) {
    QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
    if (keyEvent->key() == Qt::Key_Control && ctrlHeld_) {
      // Perform dock if we have a valid drop zone
      if (draggingDock_ && overlay_->currentDropArea() != DockDropOverlay::DropArea::None) {
        overlay_->performDock(draggingDock_);
      }
      ctrlHeld_ = false;
      overlay_->hideOverlay();
    }
  }

  // Check if this is a dock widget becoming floating
  QDockWidget* dockWidget = qobject_cast<QDockWidget*>(watched);
  if (dockWidget) {
    if (event->type() == QEvent::MouseButtonPress) {
      QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
      if (mouseEvent->button() == Qt::LeftButton) {
        // Start tracking this dock widget
        draggingDock_ = dockWidget;
        pollTimer_->start();
      }
    } else if (event->type() == QEvent::MouseButtonRelease) {
      QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
      if (mouseEvent->button() == Qt::LeftButton && draggingDock_ == dockWidget) {
        if (ctrlHeld_ && overlay_->currentDropArea() != DockDropOverlay::DropArea::None) {
          overlay_->performDock(draggingDock_);
        }
        stopTracking();
      }
    } else if (event->type() == QEvent::Close || event->type() == QEvent::Hide) {
      if (draggingDock_ == dockWidget) {
        stopTracking();
      }
    }
  }

  // Also check for title bar widget events (dock widgets use internal widgets for title bar)
  QWidget* widget = qobject_cast<QWidget*>(watched);
  if (widget && !dockWidget) {
    QDockWidget* parentDock = qobject_cast<QDockWidget*>(widget->parentWidget());
    if (parentDock) {
      if (event->type() == QEvent::MouseButtonPress) {
        QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
        if (mouseEvent->button() == Qt::LeftButton) {
          draggingDock_ = parentDock;
          pollTimer_->start();
        }
      } else if (event->type() == QEvent::MouseButtonRelease) {
        QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
        if (mouseEvent->button() == Qt::LeftButton && draggingDock_ == parentDock) {
          if (ctrlHeld_ && overlay_->currentDropArea() != DockDropOverlay::DropArea::None) {
            overlay_->performDock(draggingDock_);
          }
          stopTracking();
        }
      }
    }
  }

  return false;  // Don't consume events
}

}  // namespace ros_weaver
