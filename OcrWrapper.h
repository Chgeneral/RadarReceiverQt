#ifndef OCRWRAPPER_H
#define OCRWRAPPER_H

#include <QString>
#include <QImage>
#include <QProcess>
#include <QTemporaryFile>

class OcrWrapper
{
public:
    OcrWrapper() = default;
    ~OcrWrapper() = default;

    bool init(const QString& tessPath, const QString& lang = "eng") {
        m_tessPath = tessPath;
        m_lang     = lang;
        m_ready    = true;
        return true;
    }

    QString recognizeDigits(const QImage& image);
    bool isReady() const { return m_ready; }

private:
    QString m_tessPath = "tesseract";  // tesseract可执行文件路径
    QString m_lang     = "eng";
    bool    m_ready    = false;
};

#endif // OCRWRAPPER_H
