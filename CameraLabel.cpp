#include "CameraLabel.h"
#include <QPainter>

CameraLabel::CameraLabel(QWidget* parent)
    : QLabel(parent)
{
    m_rubberBand = new QRubberBand(QRubberBand::Rectangle, this);
    setMouseTracking(true);
}

void CameraLabel::clearRects()
{
    m_hrRect = QRect();
    m_rrRect = QRect();
    update();
}

void CameraLabel::mousePressEvent(QMouseEvent* event)
{
    if (m_selectMode == None) return;
    m_startPos = event->pos();
    m_rubberBand->setGeometry(QRect(m_startPos, QSize()));
    m_rubberBand->show();
}

void CameraLabel::mouseMoveEvent(QMouseEvent* event)
{
    if (m_selectMode == None || !m_rubberBand->isVisible()) return;
    m_rubberBand->setGeometry(QRect(m_startPos, event->pos()).normalized());
}

void CameraLabel::mouseReleaseEvent(QMouseEvent* event)
{
    if (m_selectMode == None) return;
    m_rubberBand->hide();

    QRect selected = QRect(m_startPos, event->pos()).normalized();
    if (selected.width() < 5 || selected.height() < 5) return;

    if (m_selectMode == SelectHR) {
        m_hrRect = selected;
        emit hrRectChanged(selected);
    } else if (m_selectMode == SelectRR) {
        m_rrRect = selected;
        emit rrRectChanged(selected);
    }

    m_selectMode = None;
    update();
}

void CameraLabel::paintEvent(QPaintEvent* event)
{
    QLabel::paintEvent(event);
    QPainter painter(this);

    // 画心率框（绿色）
    if (!m_hrRect.isNull()) {
        painter.setPen(QPen(Qt::green, 2));
        painter.drawRect(m_hrRect);
        painter.setPen(Qt::green);
        painter.drawText(m_hrRect.topLeft() + QPoint(2, -4), "HR");
    }

    // 画呼吸率框（黄色）
    if (!m_rrRect.isNull()) {
        painter.setPen(QPen(Qt::yellow, 2));
        painter.drawRect(m_rrRect);
        painter.setPen(Qt::yellow);
        painter.drawText(m_rrRect.topLeft() + QPoint(2, -4), "RR");
    }
}
