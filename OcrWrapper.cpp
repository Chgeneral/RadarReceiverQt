#include "OcrWrapper.h"
#include <QTemporaryDir>
#include <QRegularExpression>
#include <QProcess>

QString OcrWrapper::recognizeDigits(const QImage& image)
{
    if (!m_ready || image.isNull()) return QString();

    // 放大3倍提高识别率
    QImage enlarged = image.scaled(
        image.width()  * 3,
        image.height() * 3,
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation);

    QImage gray = enlarged.convertToFormat(QImage::Format_Grayscale8);

    // 保存为临时图片文件
    QTemporaryDir tmpDir;
    if (!tmpDir.isValid()) return QString();

    QString imgPath = tmpDir.path() + "/ocr_input.png";
    QString outBase = tmpDir.path() + "/ocr_output";

    if (!gray.save(imgPath, "PNG")) return QString();

    // 调用tesseract命令行
    QProcess proc;
    QStringList args;
    args << imgPath
         << outBase
         << "-l" << m_lang
         << "--psm" << "7"          // 单行模式
         << "-c" << "tessedit_char_whitelist=0123456789.";

    proc.start(m_tessPath, args);
    if (!proc.waitForFinished(3000)) {
        proc.kill();
        return QString();
    }

    // 读取输出文件
    QFile outFile(outBase + ".txt");
    if (!outFile.open(QIODevice::ReadOnly | QIODevice::Text))
        return QString();

    QString result = QString::fromUtf8(outFile.readAll())
                         .trimmed()
                         .remove(QRegularExpression("[^0-9.]"));
    return result;
}
