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
  // Create keyboard shortcut to snap floating dock to nearest edge (F8)
  QShortcut* dockModeShortcut = new QShortcut(QKeySequence(Qt::Key_F8), mainWindow);
  dockModeShortcut->setContext(Qt::ApplicationShortcut);
  connect(dockModeShortcut, &QShortcut::activated, this, [this]() {
    toggleDockingMode();
  });
}

void DockDragFilter::toggleDockingMode() {
  // Find a floating dock
  QDockWidget* floatingDock = nullptr;
  for (QDockWidget* dock : mainWindow_->findChildren<QDockWidget*>()) {
    if (dock->isFloating() && dock->isVisible()) {
      floatingDock = dock;
      break;
    }
  }

  if (!floatingDock) {
    qDebug() << "DockDragFilter: No floating dock found";
    return;
  }

  // Get the floating dock's center position
  QPoint dockCenter = floatingDock->mapToGlobal(floatingDock->rect().center());

  // Determine which dock area is closest based on position relative to main window
  QRect mainRect = mainWindow_->geometry();

  // Calculate distances to each edge
  int distLeft = dockCenter.x() - mainRect.left();
  int distRight = mainRect.right() - dockCenter.x();
  int distTop = dockCenter.y() - mainRect.top();
  int distBottom = mainRect.bottom() - dockCenter.y();

  // Find the nearest edge
  int minDist = qMin(qMin(distLeft, distRight), qMin(distTop, distBottom));

  Qt::DockWidgetArea area;
  if (minDist == distLeft) {
    area = Qt::LeftDockWidgetArea;
  } else if (minDist == distRight) {
    area = Qt::RightDockWidgetArea;
  } else if (minDist == distTop) {
    area = Qt::TopDockWidgetArea;
  } else {
    area = Qt::BottomDockWidgetArea;
  }

  qDebug() << "DockDragFilter: Docking" << floatingDock->windowTitle() << "to area:" << area;

  // Perform the dock
  mainWindow_->addDockWidget(area, floatingDock);
  floatingDock->setFloating(false);
  floatingDock->show();

  qDebug() << "DockDragFilter: After dock - isFloating:" << floatingDock->isFloating();
}

bool DockDragFilter::eventFilter(QObject* watched, QEvent* event) {
  Q_UNUSED(watched);
  Q_UNUSED(event);
  return false;
}

}  // namespace ros_weaver
