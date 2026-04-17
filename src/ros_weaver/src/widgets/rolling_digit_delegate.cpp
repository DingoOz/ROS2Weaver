#include "ros_weaver/widgets/rolling_digit_delegate.hpp"

#include <QPainter>
#include <QApplication>
#include <cmath>

namespace ros_weaver {

RollingDigitDelegate::RollingDigitDelegate(QObject* parent)
  : QStyledItemDelegate(parent)
{
}

RollingDigitDelegate::~RollingDigitDelegate() {
  clearAnimations();
}

void RollingDigitDelegate::clearAnimations() {
  for (auto& state : animations_) {
    if (state.animation) {
      state.animation->stop();
      delete state.animation;
    }
  }
  animations_.clear();
}

QString RollingDigitDelegate::formatRate(double rate) const {
  if (rate >= 1000) {
    return QString("%1 kHz").arg(rate / 1000.0, 0, 'f', 1);
  } else if (rate >= 1) {
    return QString("%1 Hz").arg(rate, 0, 'f', 1);
  } else if (rate > 0) {
    return QString("%1 Hz").arg(rate, 0, 'f', 2);
  } else {
    return QString("-");
  }
}

int RollingDigitDelegate::digitScrollDistance(int fromDigit, int toDigit) const {
  int directDist = toDigit - fromDigit;
  int wrapDist = directDist > 0 ? directDist - 10 : directDist + 10;
  return std::abs(directDist) <= std::abs(wrapDist) ? directDist : wrapDist;
}

void RollingDigitDelegate::notifyRateChanged(int row, double oldRate, double newRate) {
  QString oldValue = formatRate(oldRate);
  QString newValue = formatRate(newRate);

  if (oldValue == newValue) {
    return;  // No visible change
  }

  // Stop any existing animation for this row
  if (animations_.contains(row)) {
    if (animations_[row].animation) {
      animations_[row].animation->stop();
      delete animations_[row].animation;
    }
  }

  // Create new animation state
  AnimationState state;
  state.oldValue = oldValue;
  state.newValue = newValue;
  state.progress = 0.0;

  // Create animation
  state.animation = new QPropertyAnimation(this, QByteArray());
  state.animation->setDuration(animationDuration_);
  state.animation->setStartValue(0.0);
  state.animation->setEndValue(1.0);
  state.animation->setEasingCurve(QEasingCurve::OutCubic);

  // Store the row in the animation's property for identification
  state.animation->setProperty("row", row);

  connect(state.animation, &QPropertyAnimation::valueChanged,
          this, &RollingDigitDelegate::onAnimationValueChanged);
  connect(state.animation, &QPropertyAnimation::finished,
          this, &RollingDigitDelegate::onAnimationFinished);

  animations_[row] = state;
  state.animation->start();
}

void RollingDigitDelegate::onAnimationValueChanged(const QVariant& value) {
  QPropertyAnimation* anim = qobject_cast<QPropertyAnimation*>(sender());
  if (!anim) return;

  int row = anim->property("row").toInt();
  if (animations_.contains(row)) {
    animations_[row].progress = value.toReal();
    emit needsRepaint(row);
  }
}

void RollingDigitDelegate::onAnimationFinished() {
  QPropertyAnimation* anim = qobject_cast<QPropertyAnimation*>(sender());
  if (!anim) return;

  int row = anim->property("row").toInt();
  if (animations_.contains(row)) {
    animations_[row].progress = 1.0;
    animations_[row].animation = nullptr;
    anim->deleteLater();
    emit needsRepaint(row);
  }
}

QSize RollingDigitDelegate::sizeHint(const QStyleOptionViewItem& option,
                                      const QModelIndex& index) const {
  Q_UNUSED(index)
  QFontMetrics fm(option.font);
  // Size for "999.9 kHz" plus padding
  return QSize(fm.horizontalAdvance("999.9 kHz") + 16, fm.height() + 4);
}

void RollingDigitDelegate::paint(QPainter* painter,
                                  const QStyleOptionViewItem& option,
                                  const QModelIndex& index) const {
  // Draw background (selection, hover, etc.)
  QStyleOptionViewItem opt = option;
  initStyleOption(&opt, index);

  // Draw the background
  QStyle* style = opt.widget ? opt.widget->style() : QApplication::style();
  style->drawPrimitive(QStyle::PE_PanelItemViewItem, &opt, painter, opt.widget);

  // Get the current value from model
  QString currentValue = index.data(Qt::DisplayRole).toString();
  double rate = index.data(Qt::UserRole).toDouble();

  // Determine text color
  QColor textColor = activeColor_;
  if (rate <= 0) {
    textColor = inactiveColor_;
  }

  // Check if we have an active animation for this row
  int row = index.row();
  if (animations_.contains(row) && animations_[row].progress < 1.0) {
    const AnimationState& state = animations_[row];
    drawRollingText(painter, opt.rect, state.oldValue, state.newValue,
                    state.progress, textColor);
  } else {
    // No animation - draw normally
    painter->save();
    painter->setPen(textColor);
    painter->drawText(opt.rect.adjusted(4, 0, -4, 0),
                      Qt::AlignVCenter | Qt::AlignRight, currentValue);
    painter->restore();
  }
}

void RollingDigitDelegate::drawRollingText(QPainter* painter, const QRect& rect,
                                            const QString& oldValue, const QString& newValue,
                                            qreal progress, const QColor& textColor) const {
  painter->save();
  painter->setClipRect(rect);

  QFont font = painter->font();
  QFontMetrics fm(font);
  qreal charHeight = fm.height();

  // Get monospace-style digit width
  qreal digitWidth = fm.horizontalAdvance('0');

  // Calculate starting x position (right-aligned)
  int maxLen = qMax(oldValue.length(), newValue.length());
  qreal totalWidth = 0;
  for (int i = 0; i < maxLen; ++i) {
    QChar toChar = i < newValue.length() ? newValue.at(i) : QChar(' ');
    if (toChar.isDigit()) {
      totalWidth += digitWidth;
    } else {
      totalWidth += fm.horizontalAdvance(toChar);
    }
  }

  qreal x = rect.right() - 4 - totalWidth;
  qreal y = rect.top() + (rect.height() + fm.ascent() - fm.descent()) / 2.0;

  for (int i = 0; i < maxLen; ++i) {
    QChar fromChar = i < oldValue.length() ? oldValue.at(i) : QChar(' ');
    QChar toChar = i < newValue.length() ? newValue.at(i) : QChar(' ');

    qreal charWidth = (fromChar.isDigit() || toChar.isDigit()) ? digitWidth : fm.horizontalAdvance(toChar);

    if (fromChar.isDigit() && toChar.isDigit() && fromChar != toChar) {
      // Animate digit transition
      int fromDigit = fromChar.digitValue();
      int toDigit = toChar.digitValue();
      int scrollDist = digitScrollDistance(fromDigit, toDigit);
      qreal currentDigitPos = fromDigit + scrollDist * progress;

      while (currentDigitPos < 0) currentDigitPos += 10;
      while (currentDigitPos >= 10) currentDigitPos -= 10;

      int lowerDigit = static_cast<int>(std::floor(currentDigitPos)) % 10;
      int upperDigit = (lowerDigit + 1) % 10;
      qreal fraction = currentDigitPos - std::floor(currentDigitPos);

      qreal scrollOffset = fraction * charHeight;

      // Clip to this character's column
      painter->save();
      painter->setClipRect(QRectF(x, rect.top(), charWidth, rect.height()));

      painter->setPen(textColor);
      painter->drawText(QPointF(x, y - scrollOffset), QString::number(lowerDigit));
      painter->drawText(QPointF(x, y - scrollOffset + charHeight), QString::number(upperDigit));

      painter->restore();
    } else {
      // No animation for this character
      painter->setPen(textColor);
      painter->drawText(QPointF(x, y), toChar);
    }

    x += charWidth;
  }

  painter->restore();
}

}  // namespace ros_weaver
