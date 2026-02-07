#include "ros_weaver/widgets/rolling_digit_label.hpp"

#include <QPainter>
#include <QPaintEvent>
#include <QFontMetrics>
#include <cmath>

namespace ros_weaver {

RollingDigitLabel::RollingDigitLabel(QWidget* parent)
  : QWidget(parent)
  , currentValue_("-")
  , targetValue_("-")
  , displayValue_("-")
  , textColor_(Qt::white)
  , font_(QFont("Sans", 10))
  , animationDuration_(300)
  , animationProgress_(1.0)
  , animationsEnabled_(true)
  , animation_(nullptr)
{
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  setMinimumHeight(fontMetrics().height() + 4);
}

RollingDigitLabel::~RollingDigitLabel() {
  if (animation_) {
    animation_->stop();
    delete animation_;
  }
}

void RollingDigitLabel::setValue(const QString& formattedValue) {
  if (formattedValue == targetValue_) {
    return;  // No change
  }

  // Store the old value as the starting point for animation
  currentValue_ = targetValue_;
  targetValue_ = formattedValue;
  displayValue_ = formattedValue;  // For size calculations

  if (animationsEnabled_ && !currentValue_.isEmpty() && currentValue_ != "-") {
    startAnimation();
  } else {
    // No animation - just update immediately
    animationProgress_ = 1.0;
    currentValue_ = targetValue_;
    update();
  }
}

void RollingDigitLabel::setTextColor(const QColor& color) {
  if (textColor_ != color) {
    textColor_ = color;
    update();
  }
}

void RollingDigitLabel::setFont(const QFont& font) {
  if (font_ != font) {
    font_ = font;
    QWidget::setFont(font);
    updateGeometry();
    update();
  }
}

void RollingDigitLabel::setAnimationDuration(int ms) {
  animationDuration_ = qMax(50, ms);  // Minimum 50ms
  if (animation_) {
    animation_->setDuration(animationDuration_);
  }
}

void RollingDigitLabel::setAnimationsEnabled(bool enabled) {
  animationsEnabled_ = enabled;
  if (!enabled && animation_) {
    stopAnimation();
  }
}

void RollingDigitLabel::setAnimationProgress(qreal progress) {
  if (animationProgress_ != progress) {
    animationProgress_ = progress;
    update();
  }
}

void RollingDigitLabel::startAnimation() {
  if (!animation_) {
    animation_ = new QPropertyAnimation(this, "animationProgress");
    animation_->setEasingCurve(QEasingCurve::OutCubic);
  }

  animation_->stop();
  animation_->setDuration(animationDuration_);
  animation_->setStartValue(0.0);
  animation_->setEndValue(1.0);
  animation_->start();
}

void RollingDigitLabel::stopAnimation() {
  if (animation_) {
    animation_->stop();
  }
  animationProgress_ = 1.0;
  currentValue_ = targetValue_;
  update();
}

int RollingDigitLabel::digitScrollDistance(int fromDigit, int toDigit) const {
  // Calculate shortest path between two digits (0-9)
  // e.g., 3→7 = +4, 7→3 = -4, 9→1 = +2 (wraps through 0), 1→9 = -2
  int directDist = toDigit - fromDigit;
  int wrapDist = directDist > 0 ? directDist - 10 : directDist + 10;

  // Return the shorter distance
  return std::abs(directDist) <= std::abs(wrapDist) ? directDist : wrapDist;
}

qreal RollingDigitLabel::digitWidth() const {
  QFontMetrics fm(font_);
  // Use '0' as reference - all digits should have same width in most fonts
  // For proportional fonts, this creates consistent column alignment
  qreal maxWidth = 0;
  for (char c = '0'; c <= '9'; ++c) {
    maxWidth = qMax(maxWidth, (qreal)fm.horizontalAdvance(QChar(c)));
  }
  return maxWidth;
}

QSize RollingDigitLabel::sizeHint() const {
  QFontMetrics fm(font_);
  QString measureText = displayValue_.isEmpty() ? "-" : displayValue_;
  int width = fm.horizontalAdvance(measureText) + 8;  // Padding
  int height = fm.height() + 4;
  return QSize(width, height);
}

QSize RollingDigitLabel::minimumSizeHint() const {
  return sizeHint();
}

void RollingDigitLabel::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event)

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setRenderHint(QPainter::TextAntialiasing);
  painter.setFont(font_);

  QFontMetrics fm(font_);
  qreal charHeight = fm.height();
  qreal digWidth = digitWidth();

  // Center text vertically
  qreal y = (height() + fm.ascent() - fm.descent()) / 2.0;

  // If animation is complete or values are the same, just draw the target
  if (animationProgress_ >= 1.0 || currentValue_ == targetValue_) {
    painter.setPen(textColor_);
    painter.drawText(rect().adjusted(4, 0, -4, 0), Qt::AlignVCenter | Qt::AlignLeft, targetValue_);
    return;
  }

  // Animate digit by digit
  qreal x = 4;  // Left padding

  // Determine the max length to iterate
  int maxLen = qMax(currentValue_.length(), targetValue_.length());

  // Set up clipping to prevent digits from drawing outside bounds
  painter.setClipRect(rect());

  for (int i = 0; i < maxLen; ++i) {
    QChar fromChar = i < currentValue_.length() ? currentValue_.at(i) : QChar(' ');
    QChar toChar = i < targetValue_.length() ? targetValue_.at(i) : QChar(' ');

    qreal charWidth;
    if (fromChar.isDigit() || toChar.isDigit()) {
      charWidth = digWidth;
    } else {
      charWidth = fm.horizontalAdvance(toChar);
    }

    // Check if both characters are digits - only animate digit transitions
    if (fromChar.isDigit() && toChar.isDigit()) {
      int fromDigit = fromChar.digitValue();
      int toDigit = toChar.digitValue();

      if (fromDigit != toDigit) {
        // Calculate the scroll distance and current position
        int scrollDist = digitScrollDistance(fromDigit, toDigit);
        qreal currentDigitPos = fromDigit + scrollDist * animationProgress_;

        // Normalize to 0-9 range
        while (currentDigitPos < 0) currentDigitPos += 10;
        while (currentDigitPos >= 10) currentDigitPos -= 10;

        // Get the two digits we're between
        int lowerDigit = static_cast<int>(std::floor(currentDigitPos)) % 10;
        int upperDigit = (lowerDigit + 1) % 10;
        qreal fraction = currentDigitPos - std::floor(currentDigitPos);

        // Draw the rolling effect - clip to character cell
        painter.save();
        QRectF clipRect(x, 0, charWidth, height());
        painter.setClipRect(clipRect);

        // Calculate vertical offset for rolling effect
        qreal scrollOffset = fraction * charHeight;

        // Draw lower digit (scrolling up and out)
        painter.setPen(textColor_);
        painter.drawText(QPointF(x, y - scrollOffset), QString::number(lowerDigit));

        // Draw upper digit (scrolling up and in)
        painter.drawText(QPointF(x, y - scrollOffset + charHeight), QString::number(upperDigit));

        painter.restore();
      } else {
        // Same digit, no animation needed
        painter.setPen(textColor_);
        painter.drawText(QPointF(x, y), toChar);
      }
    } else {
      // Non-digit character - instant transition
      // Use the target character
      painter.setPen(textColor_);
      painter.drawText(QPointF(x, y), toChar);
    }

    x += charWidth;
  }
}

}  // namespace ros_weaver
