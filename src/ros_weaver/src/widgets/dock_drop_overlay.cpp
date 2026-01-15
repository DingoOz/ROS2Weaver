#include "ros_weaver/widgets/dock_drop_overlay.hpp"
#include "ros_weaver/core/theme_manager.hpp"
#include <QPainter>
#include <QPaintEvent>
#include <QApplication>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QTimer>
#include <QDebug>
#include <QScreen>
#include <QShortcut>

namespace ros_weaver {

// ============================================================================
// DockDropOverlay
// ============================================================================

DockDropOverlay::DockDropOverlay(QMainWindow* mainWindow)
    : QWidget(nullptr), mainWindow_(mainWindow) {  // No parent so it can float above
  setWindowFlags(Qt::FramelessWindowHint | Qt::Tool | Qt::WindowStaysOnTopHint | Qt::WindowDoesNotAcceptFocus);
  setAttribute(Qt::WA_TranslucentBackground);
  setAttribute(Qt::WA_TransparentForMouseEvents);
  setAttribute(Qt::WA_ShowWithoutActivating);
  hide();
}

DockDropOverlay::~DockDropOverlay() = default;

void DockDropOverlay::showOverlay(QDockWidget* draggedDock) {
  draggedDock_ = draggedDock;
  currentDropArea_ = DropArea::None;
  targetDock_ = nullptr;

  // Position overlay over the main window using screen coordinates
  QRect mainRect = mainWindow_->geometry();  // Window geometry in screen coordinates
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

  // Get the main window's content area in screen coordinates
  QPoint mainTopLeft = mainWindow_->mapToGlobal(QPoint(0, 0));
  QRect mainRect = mainWindow_->rect();

  // Convert to overlay coordinates
  QPoint overlayOffset = mainTopLeft - geometry().topLeft();

  // Outer zones (edges of the main window)
  int ox = overlayOffset.x();
  int oy = overlayOffset.y();
  int mw = mainRect.width();
  int mh = mainRect.height();

  dropZones_[DropArea::LeftOuter] = QRect(
      ox, oy + OUTER_ZONE_SIZE,
      OUTER_ZONE_SIZE, mh - 2 * OUTER_ZONE_SIZE);

  dropZones_[DropArea::RightOuter] = QRect(
      ox + mw - OUTER_ZONE_SIZE, oy + OUTER_ZONE_SIZE,
      OUTER_ZONE_SIZE, mh - 2 * OUTER_ZONE_SIZE);

  dropZones_[DropArea::TopOuter] = QRect(
      ox + OUTER_ZONE_SIZE, oy,
      mw - 2 * OUTER_ZONE_SIZE, OUTER_ZONE_SIZE);

  dropZones_[DropArea::BottomOuter] = QRect(
      ox + OUTER_ZONE_SIZE, oy + mh - OUTER_ZONE_SIZE,
      mw - 2 * OUTER_ZONE_SIZE, OUTER_ZONE_SIZE);

  // Get central widget area
  QWidget* central = mainWindow_->centralWidget();
  if (central) {
    QPoint centralTopLeft = central->mapToGlobal(QPoint(0, 0));
    QPoint centralOffset = centralTopLeft - geometry().topLeft();
    int cx = centralOffset.x();
    int cy = centralOffset.y();
    int cw = central->width();
    int ch = central->height();

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
  // Check if over a docked panel (for tabifying)
  else {
    // Find a visible dock widget under the cursor
    for (QDockWidget* dock : mainWindow_->findChildren<QDockWidget*>()) {
      if (dock != draggedDock_ && dock->isVisible() && !dock->isFloating()) {
        QRect dockRect = dock->frameGeometry();
        QPoint dockGlobal = dock->mapToGlobal(QPoint(0, 0));
        dockRect.moveTopLeft(dockGlobal);

        if (dockRect.contains(globalPos)) {
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
    qDebug() << "performDock: early return - dockWidget:" << dockWidget << "dropArea:" << static_cast<int>(currentDropArea_);
    return;
  }

  qDebug() << "performDock: docking" << dockWidget->windowTitle() << "to area:" << static_cast<int>(currentDropArea_);

  // Store info before modifying
  Qt::DockWidgetArea area = toDockWidgetArea(currentDropArea_);
  QDockWidget* target = targetDock_;
  DropArea dropArea = currentDropArea_;
  QMainWindow* mainWin = mainWindow_;

  // Hide overlay first
  hide();

  // Defer the actual docking to let Qt process events
  QTimer::singleShot(50, dockWidget, [dockWidget, mainWin, area, target, dropArea]() {
    qDebug() << "performDock (deferred): executing, currently floating:" << dockWidget->isFloating();

    if (dropArea == DropArea::Center && target) {
      // Tabify with the target dock
      qDebug() << "performDock (deferred): tabifying";
      dockWidget->setFloating(false);
      mainWin->tabifyDockWidget(target, dockWidget);
      dockWidget->show();
      dockWidget->raise();
    } else if (area != Qt::NoDockWidgetArea) {
      qDebug() << "performDock (deferred): adding to area:" << area;

      // First un-float, then add to area
      dockWidget->setFloating(false);
      mainWin->addDockWidget(area, dockWidget);
      dockWidget->show();

      qDebug() << "performDock (deferred): after - isFloating:" << dockWidget->isFloating();
    }
  });
}

void DockDropOverlay::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  auto& theme = ThemeManager::instance();
  QColor highlightColor = theme.primaryColor();
  highlightColor.setAlpha(80);

  QColor borderColor = theme.primaryColor();
  borderColor.setAlpha(220);

  QColor zoneColor = theme.isDarkTheme() ? QColor(255, 255, 255, 40) : QColor(0, 0, 0, 30);

  // Draw all drop zones with subtle background
  for (auto it = dropZones_.begin(); it != dropZones_.end(); ++it) {
    if (it.key() != DropArea::Center && !it.value().isEmpty()) {
      painter.fillRect(it.value(), zoneColor);
      painter.setPen(QPen(theme.borderColor(), 1, Qt::DashLine));
      painter.drawRect(it.value());
    }
  }

  // Draw highlighted zone
  if (currentDropArea_ != DropArea::None) {
    QRect highlightRect;

    if (currentDropArea_ == DropArea::Center && targetDock_) {
      // Highlight the target dock widget
      QPoint dockGlobal = targetDock_->mapToGlobal(QPoint(0, 0));
      highlightRect = targetDock_->rect();
      highlightRect.moveTopLeft(mapFromGlobal(dockGlobal));
    } else {
      highlightRect = getDropZoneRect(currentDropArea_);
    }

    if (!highlightRect.isEmpty()) {
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
        painter.setFont(QFont("Sans", 14, QFont::Bold));

        // Draw text with background
        QRect textRect = painter.fontMetrics().boundingRect(label);
        textRect.moveCenter(highlightRect.center());
        textRect.adjust(-12, -6, 12, 6);

        QColor textBgColor = theme.surfaceColor();
        textBgColor.setAlpha(230);
        painter.setPen(Qt::NoPen);
        painter.setBrush(textBgColor);
        painter.drawRoundedRect(textRect, 4, 4);

        painter.setPen(theme.textPrimaryColor());
        painter.drawText(highlightRect, Qt::AlignCenter, label);
      }
    }
  }
}

// ============================================================================
// DockDragFilter
// ============================================================================

DockDragFilter::DockDragFilter(QMainWindow* mainWindow, QObject* parent)
    : QObject(parent), mainWindow_(mainWindow) {
  overlay_ = new DockDropOverlay(mainWindow);

  // Create a timer to poll for dock position and update overlay
  pollTimer_ = new QTimer(this);
  pollTimer_->setInterval(16);  // ~60fps
  connect(pollTimer_, &QTimer::timeout, this, &DockDragFilter::onPollTimer);

  // Connect to all existing dock widgets
  for (QDockWidget* dock : mainWindow->findChildren<QDockWidget*>()) {
    connectToDock(dock);
  }

  // Create keyboard shortcut to toggle docking mode (F8)
  QShortcut* dockModeShortcut = new QShortcut(QKeySequence(Qt::Key_F8), mainWindow);
  dockModeShortcut->setContext(Qt::ApplicationShortcut);
  connect(dockModeShortcut, &QShortcut::activated, this, [this]() {
    toggleDockingMode();
  });

  // Also allow Escape to cancel docking mode
  QShortcut* escapeShortcut = new QShortcut(QKeySequence(Qt::Key_Escape), mainWindow);
  escapeShortcut->setContext(Qt::ApplicationShortcut);
  connect(escapeShortcut, &QShortcut::activated, this, [this]() {
    if (dockingModeActive_) {
      dockingModeActive_ = false;
      overlay_->hideOverlay();
    }
  });

  // Start the poll timer
  pollTimer_->start();
}

void DockDragFilter::toggleDockingMode() {
  if (dockingModeActive_) {
    // Turn off docking mode
    dockingModeActive_ = false;
    overlay_->hideOverlay();
    return;
  }

  // Find a floating dock
  floatingDock_ = nullptr;
  for (QDockWidget* dock : mainWindow_->findChildren<QDockWidget*>()) {
    if (dock->isFloating() && dock->isVisible()) {
      floatingDock_ = dock;
      break;
    }
  }

  if (floatingDock_) {
    dockingModeActive_ = true;
    overlay_->showOverlay(floatingDock_);
  }
}

void DockDragFilter::connectToDock(QDockWidget* dock) {
  connect(dock, &QDockWidget::topLevelChanged, this, [this, dock](bool floating) {
    if (floating) {
      // Dock became floating - track it
      floatingDock_ = dock;
      lastDockPos_ = dock->pos();
    } else {
      // Dock was re-docked
      if (floatingDock_ == dock) {
        floatingDock_ = nullptr;
        if (dockingModeActive_) {
          dockingModeActive_ = false;
          overlay_->hideOverlay();
        }
      }
    }
  });
}

void DockDragFilter::onPollTimer() {
  // Only process when docking mode is active
  if (!dockingModeActive_) {
    return;
  }

  // Check if tracked dock is still valid
  if (!floatingDock_ || !floatingDock_->isFloating()) {
    dockingModeActive_ = false;
    overlay_->hideOverlay();
    floatingDock_ = nullptr;
    return;
  }

  // Update drop zone based on the floating dock's center position
  QPoint dockCenter = floatingDock_->mapToGlobal(floatingDock_->rect().center());
  DockDropOverlay::DropArea area = overlay_->updateDropZone(dockCenter);

  // Check if mouse was released (user stopped dragging)
  bool mouseHeld = QApplication::mouseButtons() & Qt::LeftButton;
  QPoint currentPos = floatingDock_->pos();
  bool isMoving = (currentPos != lastDockPos_);
  lastDockPos_ = currentPos;

  // Debug every ~1 second
  static int debugCount = 0;
  if (++debugCount % 60 == 0) {
    qDebug() << "DockDragFilter: area=" << static_cast<int>(area) << "moving=" << isMoving << "mouseHeld=" << mouseHeld;
  }

  // If not moving and mouse not held, and we're over a valid zone, perform dock
  if (!isMoving && !mouseHeld && area != DockDropOverlay::DropArea::None) {
    qDebug() << "DockDragFilter: performing dock to area" << static_cast<int>(area);
    overlay_->performDock(floatingDock_);
    dockingModeActive_ = false;
    overlay_->hideOverlay();
    floatingDock_ = nullptr;
  }
}

bool DockDragFilter::eventFilter(QObject* watched, QEvent* event) {
  Q_UNUSED(watched);
  Q_UNUSED(event);
  return false;  // Don't block any events
}

}  // namespace ros_weaver
