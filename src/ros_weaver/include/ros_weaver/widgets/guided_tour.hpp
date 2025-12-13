#ifndef GUIDED_TOUR_HPP
#define GUIDED_TOUR_HPP

#include <QWidget>
#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPropertyAnimation>
#include <QGraphicsDropShadowEffect>
#include <QPainter>
#include <QEvent>
#include <QKeyEvent>
#include <vector>
#include <functional>

class QMainWindow;

namespace ros_weaver {

/**
 * @brief Represents a single step in the guided tour
 */
struct TourStep {
    QString targetWidgetName;       // Object name of the widget to highlight
    QString title;                  // Step title
    QString description;            // Step description
    QString category;               // Category for grouping (optional)
    std::function<QWidget*()> widgetFinder;  // Custom widget finder (optional)
};

/**
 * @brief Overlay widget that darkens the screen and creates a "spotlight" effect
 */
class TourOverlay : public QWidget {
    Q_OBJECT

public:
    explicit TourOverlay(QWidget* parent = nullptr);

    void setHighlightRect(const QRect& rect);
    void setHighlightWidget(QWidget* widget);
    void clearHighlight();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;

signals:
    void clicked();

private:
    QRect highlightRect_;
    QWidget* highlightedWidget_ = nullptr;
    int borderWidth_ = 3;
    int padding_ = 8;
};

/**
 * @brief Tooltip widget that displays step information and navigation controls
 */
class TourTooltip : public QWidget {
    Q_OBJECT

public:
    explicit TourTooltip(QWidget* parent = nullptr);

    void setStep(int current, int total, const QString& title, const QString& description);
    void positionNear(const QRect& targetRect, const QRect& parentRect);

signals:
    void nextClicked();
    void previousClicked();
    void closeClicked();

protected:
    void paintEvent(QPaintEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    void setupUi();

    QLabel* titleLabel_;
    QLabel* descriptionLabel_;
    QLabel* counterLabel_;
    QPushButton* previousButton_;
    QPushButton* nextButton_;
    QPushButton* closeButton_;
    int currentStep_ = 0;
    int totalSteps_ = 0;
};

/**
 * @brief Main guided tour controller class
 *
 * Manages the tour sequence, overlay, and tooltip positioning.
 * Usage:
 *   GuidedTour* tour = new GuidedTour(mainWindow);
 *   tour->start();
 */
class GuidedTour : public QObject {
    Q_OBJECT

public:
    explicit GuidedTour(QMainWindow* mainWindow);
    ~GuidedTour();

    void start();
    void stop();
    bool isRunning() const { return isRunning_; }

    int currentStep() const { return currentStep_; }
    int totalSteps() const { return static_cast<int>(steps_.size()); }

public slots:
    void nextStep();
    void previousStep();
    void goToStep(int index);

signals:
    void tourStarted();
    void tourFinished();
    void stepChanged(int current, int total);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void setupTourSteps();
    void showCurrentStep();
    void hideAllElements();
    QWidget* findTargetWidget(const TourStep& step);
    void updateOverlayGeometry();

    QMainWindow* mainWindow_;
    TourOverlay* overlay_;
    TourTooltip* tooltip_;

    std::vector<TourStep> steps_;
    int currentStep_ = 0;
    bool isRunning_ = false;
};

}  // namespace ros_weaver

#endif // GUIDED_TOUR_HPP
