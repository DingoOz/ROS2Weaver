#include "ros_weaver/widgets/guided_tour.hpp"

#include <QMainWindow>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QDockWidget>
#include <QApplication>
#include <QScreen>
#include <QPainterPath>
#include <QTimer>

namespace ros_weaver {

// ============================================================================
// TourOverlay Implementation
// ============================================================================

TourOverlay::TourOverlay(QWidget* parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_TransparentForMouseEvents, false);
    setAttribute(Qt::WA_NoSystemBackground, true);
    setWindowFlags(Qt::FramelessWindowHint);
    setFocusPolicy(Qt::NoFocus);
}

void TourOverlay::setHighlightRect(const QRect& rect)
{
    highlightRect_ = rect;
    highlightedWidget_ = nullptr;
    update();
}

void TourOverlay::setHighlightWidget(QWidget* widget)
{
    highlightedWidget_ = widget;
    if (widget) {
        // Map widget geometry to overlay coordinates
        QPoint topLeft = widget->mapTo(parentWidget(), QPoint(0, 0));
        highlightRect_ = QRect(topLeft, widget->size());
    } else {
        highlightRect_ = QRect();
    }
    update();
}

void TourOverlay::clearHighlight()
{
    highlightRect_ = QRect();
    highlightedWidget_ = nullptr;
    update();
}

void TourOverlay::paintEvent(QPaintEvent* /*event*/)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Semi-transparent dark overlay
    QColor overlayColor(0, 0, 0, 160);

    if (highlightRect_.isValid()) {
        // Create a path that covers everything except the highlight area
        QPainterPath fullPath;
        fullPath.addRect(rect());

        // Expanded highlight rect with padding
        QRect expandedRect = highlightRect_.adjusted(-padding_, -padding_, padding_, padding_);

        QPainterPath highlightPath;
        highlightPath.addRoundedRect(expandedRect, 8, 8);

        // Subtract highlight from full path
        QPainterPath resultPath = fullPath.subtracted(highlightPath);

        // Fill the overlay (everything except the spotlight)
        painter.fillPath(resultPath, overlayColor);

        // Draw red border around the highlighted area
        QPen borderPen(QColor(220, 53, 69));  // Bootstrap danger red
        borderPen.setWidth(borderWidth_);
        painter.setPen(borderPen);
        painter.setBrush(Qt::NoBrush);
        painter.drawRoundedRect(expandedRect, 8, 8);

        // Add a subtle glow effect
        for (int i = 1; i <= 3; ++i) {
            QColor glowColor(220, 53, 69, 80 - i * 20);
            QPen glowPen(glowColor);
            glowPen.setWidth(borderWidth_ + i * 2);
            painter.setPen(glowPen);
            painter.drawRoundedRect(expandedRect.adjusted(-i * 2, -i * 2, i * 2, i * 2), 8 + i, 8 + i);
        }
    } else {
        // No highlight - just fill with overlay
        painter.fillRect(rect(), overlayColor);
    }
}

void TourOverlay::mousePressEvent(QMouseEvent* event)
{
    // Check if click is outside the highlight area
    if (!highlightRect_.isValid() || !highlightRect_.adjusted(-padding_, -padding_, padding_, padding_).contains(event->pos())) {
        emit clicked();
    }
    // Consume the event
    event->accept();
}

// ============================================================================
// TourTooltip Implementation
// ============================================================================

TourTooltip::TourTooltip(QWidget* parent)
    : QWidget(parent)
{
    setWindowFlags(Qt::FramelessWindowHint | Qt::Tool | Qt::WindowStaysOnTopHint);
    setAttribute(Qt::WA_ShowWithoutActivating);
    setFocusPolicy(Qt::StrongFocus);
    setupUi();
}

void TourTooltip::setupUi()
{
    setMinimumWidth(320);
    setMaximumWidth(450);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(16, 16, 16, 16);
    mainLayout->setSpacing(12);

    // Title
    titleLabel_ = new QLabel(this);
    titleLabel_->setWordWrap(true);
    titleLabel_->setStyleSheet(R"(
        QLabel {
            font-size: 16px;
            font-weight: bold;
            color: #2a82da;
        }
    )");
    mainLayout->addWidget(titleLabel_);

    // Description
    descriptionLabel_ = new QLabel(this);
    descriptionLabel_->setWordWrap(true);
    descriptionLabel_->setStyleSheet(R"(
        QLabel {
            font-size: 13px;
            color: #e0e0e0;
            line-height: 1.4;
        }
    )");
    mainLayout->addWidget(descriptionLabel_);

    mainLayout->addStretch();

    // Bottom row: counter on left, buttons on right
    QHBoxLayout* bottomLayout = new QHBoxLayout();
    bottomLayout->setSpacing(8);

    // Counter label
    counterLabel_ = new QLabel(this);
    counterLabel_->setStyleSheet(R"(
        QLabel {
            font-size: 12px;
            color: #888888;
            font-weight: 500;
        }
    )");
    bottomLayout->addWidget(counterLabel_);

    bottomLayout->addStretch();

    // Navigation buttons
    QString buttonStyle = R"(
        QPushButton {
            background-color: #3a3a3a;
            color: #e0e0e0;
            border: 1px solid #555555;
            border-radius: 4px;
            padding: 6px 14px;
            font-size: 12px;
            min-width: 70px;
        }
        QPushButton:hover {
            background-color: #4a4a4a;
            border-color: #666666;
        }
        QPushButton:pressed {
            background-color: #2a2a2a;
        }
        QPushButton:disabled {
            background-color: #2a2a2a;
            color: #666666;
            border-color: #444444;
        }
    )";

    QString primaryButtonStyle = R"(
        QPushButton {
            background-color: #2a82da;
            color: white;
            border: none;
            border-radius: 4px;
            padding: 6px 14px;
            font-size: 12px;
            min-width: 70px;
        }
        QPushButton:hover {
            background-color: #3a92ea;
        }
        QPushButton:pressed {
            background-color: #1a72ca;
        }
    )";

    closeButton_ = new QPushButton(tr("Skip"), this);
    closeButton_->setStyleSheet(buttonStyle);
    connect(closeButton_, &QPushButton::clicked, this, &TourTooltip::closeClicked);
    bottomLayout->addWidget(closeButton_);

    previousButton_ = new QPushButton(tr("Previous"), this);
    previousButton_->setStyleSheet(buttonStyle);
    connect(previousButton_, &QPushButton::clicked, this, &TourTooltip::previousClicked);
    bottomLayout->addWidget(previousButton_);

    nextButton_ = new QPushButton(tr("Next"), this);
    nextButton_->setStyleSheet(primaryButtonStyle);
    connect(nextButton_, &QPushButton::clicked, this, &TourTooltip::nextClicked);
    bottomLayout->addWidget(nextButton_);

    mainLayout->addLayout(bottomLayout);

    // Widget styling
    setStyleSheet(R"(
        TourTooltip {
            background-color: #2d2d2d;
            border: 1px solid #555555;
            border-radius: 8px;
        }
    )");
}

void TourTooltip::setStep(int current, int total, const QString& title, const QString& description)
{
    currentStep_ = current;
    totalSteps_ = total;

    titleLabel_->setText(title);
    descriptionLabel_->setText(description);
    counterLabel_->setText(tr("%1 / %2").arg(current + 1).arg(total));

    // Update button states
    previousButton_->setEnabled(current > 0);

    if (current >= total - 1) {
        nextButton_->setText(tr("Finish"));
    } else {
        nextButton_->setText(tr("Next"));
    }

    // Adjust size to content
    adjustSize();
}

void TourTooltip::positionNear(const QRect& targetRect, const QRect& parentRect)
{
    const int margin = 20;
    QSize tooltipSize = sizeHint();

    // Try to position the tooltip below the target
    QPoint pos;
    int x = targetRect.center().x() - tooltipSize.width() / 2;
    int y = targetRect.bottom() + margin;

    // Ensure tooltip stays within parent bounds
    x = qBound(parentRect.left() + margin, x, parentRect.right() - tooltipSize.width() - margin);

    // If tooltip would go below parent, try above
    if (y + tooltipSize.height() > parentRect.bottom() - margin) {
        y = targetRect.top() - tooltipSize.height() - margin;
    }

    // If still doesn't fit, try to the right
    if (y < parentRect.top() + margin) {
        y = targetRect.center().y() - tooltipSize.height() / 2;
        x = targetRect.right() + margin;

        // If doesn't fit on right, try left
        if (x + tooltipSize.width() > parentRect.right() - margin) {
            x = targetRect.left() - tooltipSize.width() - margin;
        }
    }

    // Final bounds check
    x = qBound(parentRect.left() + margin, x, parentRect.right() - tooltipSize.width() - margin);
    y = qBound(parentRect.top() + margin, y, parentRect.bottom() - tooltipSize.height() - margin);

    move(parentWidget()->mapToGlobal(QPoint(x, y)));
}

void TourTooltip::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Draw rounded rectangle background
    QRect r = rect().adjusted(1, 1, -1, -1);
    painter.setPen(QPen(QColor(85, 85, 85), 1));
    painter.setBrush(QColor(45, 45, 45));
    painter.drawRoundedRect(r, 8, 8);

    QWidget::paintEvent(event);
}

void TourTooltip::keyPressEvent(QKeyEvent* event)
{
    switch (event->key()) {
        case Qt::Key_Right:
        case Qt::Key_Return:
        case Qt::Key_Enter:
            emit nextClicked();
            break;
        case Qt::Key_Left:
            emit previousClicked();
            break;
        case Qt::Key_Escape:
            emit closeClicked();
            break;
        default:
            QWidget::keyPressEvent(event);
    }
}

// ============================================================================
// GuidedTour Implementation
// ============================================================================

GuidedTour::GuidedTour(QMainWindow* mainWindow)
    : QObject(mainWindow)
    , mainWindow_(mainWindow)
    , overlay_(nullptr)
    , tooltip_(nullptr)
{
    setupTourSteps();
}

GuidedTour::~GuidedTour()
{
    stop();
}

void GuidedTour::setupTourSteps()
{
    steps_.clear();

    // Welcome step (no specific widget)
    steps_.push_back({
        "",
        tr("Welcome to ROS2 Weaver!"),
        tr("This guided tour will show you the main features of the application. "
           "Use the Next and Previous buttons to navigate, or press Escape to skip."),
        "Introduction",
        nullptr
    });

    // Menu Bar
    steps_.push_back({
        "menuBar",
        tr("Menu Bar"),
        tr("The menu bar provides access to all application functions including "
           "file operations, editing tools, view options, ROS2 commands, and help resources."),
        "Navigation",
        [this]() -> QWidget* { return mainWindow_->menuBar(); }
    });

    // Main Toolbar
    steps_.push_back({
        "mainToolBar",
        tr("Main Toolbar"),
        tr("Quick access to common actions like creating new projects, opening files, "
           "saving, undo/redo, and build commands. Hover over icons for tooltips."),
        "Navigation",
        [this]() -> QWidget* {
            auto toolbars = mainWindow_->findChildren<QToolBar*>();
            return toolbars.isEmpty() ? nullptr : toolbars.first();
        }
    });

    // Canvas (central widget)
    steps_.push_back({
        "weaverCanvas",
        tr("Visual Canvas"),
        tr("The central canvas is where you design your ROS2 system visually. "
           "Drag package templates from the Package Browser to create nodes. "
           "Connect them by dragging between pins to define topic relationships."),
        "Canvas",
        [this]() -> QWidget* { return mainWindow_->centralWidget(); }
    });

    // Package Browser (left dock)
    steps_.push_back({
        "packageBrowserDock",
        tr("Package Browser"),
        tr("Browse and search available ROS2 package templates. "
           "Drag items onto the canvas to add them to your project. "
           "Use the search bar to filter packages by name or type."),
        "Panels",
        nullptr
    });

    // Properties Panel (right dock)
    steps_.push_back({
        "propertiesDock",
        tr("Properties & Tools Panel"),
        tr("When you select a node on the canvas, its properties appear here. "
           "This panel also contains tabs for Topic Viewer, TF Tree visualization, "
           "and data plotting tools."),
        "Panels",
        nullptr
    });

    // Output Panel (bottom dock)
    steps_.push_back({
        "outputDock",
        tr("Output Panel"),
        tr("View build output, ROS2 logs, and access an integrated terminal. "
           "Build errors and warnings are displayed here with clickable links "
           "to navigate to the source."),
        "Panels",
        nullptr
    });

    // System Mapping Dock
    steps_.push_back({
        "systemMappingDock",
        tr("System Mapping"),
        tr("Discover and visualize running ROS2 nodes and their connections. "
           "This panel shows the live state of your ROS2 system and helps you "
           "understand how nodes communicate."),
        "ROS2",
        nullptr
    });

    // Status Bar
    steps_.push_back({
        "statusBar",
        tr("Status Bar"),
        tr("The status bar shows ROS2 connection status, current operation progress, "
           "and helpful tips. Watch here for feedback when performing actions."),
        "Navigation",
        [this]() -> QWidget* { return mainWindow_->statusBar(); }
    });

    // LLM Chat (in Output Panel)
    steps_.push_back({
        "outputDock",
        tr("AI Assistant (LocalLLM Tab)"),
        tr("The Output Panel includes a LocalLLM tab where you can chat with an AI assistant. "
           "Ask questions about ROS2 development, get help with nodes and topics, "
           "or get suggestions for your project. Requires Ollama or compatible backend."),
        "AI Features",
        nullptr
    });

    // Keyboard shortcuts tip
    steps_.push_back({
        "",
        tr("Keyboard Shortcuts"),
        tr("Press F1 for context-sensitive help on any focused element. "
           "Use Ctrl+Shift+? to view all keyboard shortcuts. "
           "Many actions have shortcuts for faster workflow."),
        "Tips",
        nullptr
    });

    // Final step
    steps_.push_back({
        "",
        tr("You're Ready!"),
        tr("You now know the basics of ROS2 Weaver. Start by dragging a package "
           "template onto the canvas, or open an existing project from the File menu. "
           "\n\nFor more help, check the Help menu or press F1 anytime."),
        "Conclusion",
        nullptr
    });
}

void GuidedTour::start()
{
    if (isRunning_) {
        return;
    }

    isRunning_ = true;
    currentStep_ = 0;

    // Create overlay
    overlay_ = new TourOverlay(mainWindow_);
    overlay_->setGeometry(mainWindow_->rect());
    overlay_->show();
    overlay_->raise();

    connect(overlay_, &TourOverlay::clicked, this, &GuidedTour::stop);

    // Create tooltip
    tooltip_ = new TourTooltip(mainWindow_);
    connect(tooltip_, &TourTooltip::nextClicked, this, &GuidedTour::nextStep);
    connect(tooltip_, &TourTooltip::previousClicked, this, &GuidedTour::previousStep);
    connect(tooltip_, &TourTooltip::closeClicked, this, &GuidedTour::stop);

    // Install event filter to handle resize
    mainWindow_->installEventFilter(this);

    emit tourStarted();
    showCurrentStep();

    // Give focus to tooltip for keyboard navigation
    tooltip_->setFocus();
}

void GuidedTour::stop()
{
    if (!isRunning_) {
        return;
    }

    isRunning_ = false;
    mainWindow_->removeEventFilter(this);
    hideAllElements();
    emit tourFinished();
}

void GuidedTour::nextStep()
{
    if (currentStep_ < static_cast<int>(steps_.size()) - 1) {
        currentStep_++;
        showCurrentStep();
        emit stepChanged(currentStep_, totalSteps());
    } else {
        stop();
    }
}

void GuidedTour::previousStep()
{
    if (currentStep_ > 0) {
        currentStep_--;
        showCurrentStep();
        emit stepChanged(currentStep_, totalSteps());
    }
}

void GuidedTour::goToStep(int index)
{
    if (index >= 0 && index < static_cast<int>(steps_.size())) {
        currentStep_ = index;
        showCurrentStep();
        emit stepChanged(currentStep_, totalSteps());
    }
}

void GuidedTour::showCurrentStep()
{
    if (!isRunning_ || currentStep_ < 0 || currentStep_ >= static_cast<int>(steps_.size())) {
        return;
    }

    const TourStep& step = steps_[currentStep_];

    // Find target widget
    QWidget* targetWidget = findTargetWidget(step);

    // Update overlay
    if (targetWidget && targetWidget->isVisible()) {
        overlay_->setHighlightWidget(targetWidget);
    } else {
        overlay_->clearHighlight();
    }

    // Update tooltip content
    tooltip_->setStep(currentStep_, static_cast<int>(steps_.size()), step.title, step.description);

    // Position tooltip
    QRect targetRect;
    if (targetWidget && targetWidget->isVisible()) {
        QPoint topLeft = targetWidget->mapTo(mainWindow_, QPoint(0, 0));
        targetRect = QRect(topLeft, targetWidget->size());
    } else {
        // Center in window if no target widget
        int w = 300, h = 100;
        targetRect = QRect(
            mainWindow_->width() / 2 - w / 2,
            mainWindow_->height() / 2 - h / 2,
            w, h
        );
    }

    tooltip_->positionNear(targetRect, mainWindow_->rect());
    tooltip_->show();
    tooltip_->raise();
    tooltip_->setFocus();
}

void GuidedTour::hideAllElements()
{
    if (overlay_) {
        overlay_->hide();
        overlay_->deleteLater();
        overlay_ = nullptr;
    }

    if (tooltip_) {
        tooltip_->hide();
        tooltip_->deleteLater();
        tooltip_ = nullptr;
    }
}

QWidget* GuidedTour::findTargetWidget(const TourStep& step)
{
    // Use custom finder if provided
    if (step.widgetFinder) {
        return step.widgetFinder();
    }

    // Find by object name
    if (!step.targetWidgetName.isEmpty()) {
        return mainWindow_->findChild<QWidget*>(step.targetWidgetName);
    }

    return nullptr;
}

bool GuidedTour::eventFilter(QObject* watched, QEvent* event)
{
    if (watched == mainWindow_ && event->type() == QEvent::Resize) {
        updateOverlayGeometry();
    }
    return QObject::eventFilter(watched, event);
}

void GuidedTour::updateOverlayGeometry()
{
    if (overlay_) {
        overlay_->setGeometry(mainWindow_->rect());
    }

    // Reposition current step to handle resize
    if (isRunning_) {
        showCurrentStep();
    }
}

}  // namespace ros_weaver
