#ifndef CAMERALABEL_H
#define CAMERALABEL_H

#include <QLabel>
#include <QMouseEvent>
#include <QRubberBand>

class CameraLabel : public QLabel
{
    Q_OBJECT

public:
    explicit CameraLabel(QWidget* parent = nullptr);

    QRect getHRRect() const { return m_hrRect; }
    QRect getRRRect() const { return m_rrRect; }
    void clearRects();

    enum SelectMode { None, SelectHR, SelectRR };
    void setSelectMode(SelectMode mode) { m_selectMode = mode; }

signals:
    void hrRectChanged(const QRect& rect);
    void rrRectChanged(const QRect& rect);

protected:
    void mousePressEvent(QMouseEvent* event)   override;
    void mouseMoveEvent(QMouseEvent* event)    override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void paintEvent(QPaintEvent* event)        override;

private:
    SelectMode   m_selectMode = None;
    QPoint       m_startPos;
    QRubberBand* m_rubberBand = nullptr;
    QRect        m_hrRect;   // 心率框选区域
    QRect        m_rrRect;   // 呼吸率框选区域
};


#endif // CAMERALABEL_H
