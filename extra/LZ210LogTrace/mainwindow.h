#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFile>
#include <QTcpSocket>
#include <QTimer>
#include <QSerialPort>

class QTextEdit;
class QLineEdit;
class QSpinBox;
class QPushButton;
class QLabel;
class QComboBox;
class QCheckBox;
class QTabWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // TCP
    void connectToServer();
    void disconnectFromServer();
    void socketConnected();
    void socketDisconnected();
    void socketError(QAbstractSocket::SocketError socketError);
    void readData();
    void reconnectTcp();
    void clearTcpWindow();
    void changeLogFile();

    // Serial
    void openSerialPort();
    void closeSerialPort();
    void serialReadyRead();
    void serialError(QSerialPort::SerialPortError error);
    void clearSerialWindow();
    void changeSerialLogFile();

private:
    void setupUi();
    void setupTcpTab();
    void setupSerialTab();
    QString createTimestamp() const;
    void writeToLog(const QString &text);
    void writeSerialToLog(const QString &text);
    void updateTcpButtons();
    void updateSerialButtons();
    void appendTcpText(const QString &text);
    void appendSerialText(const QString &text);

    // TCP
    QTcpSocket *m_socket = nullptr;
    QTimer *m_reconnectTimer = nullptr;
    bool m_tcpAutoReconnect = true;
    bool m_tcpManualDisconnect = false;

    QTextEdit *m_receiveWindow = nullptr;
    QLineEdit *m_ipEdit = nullptr;
    QSpinBox *m_portSpinBox = nullptr;
    QPushButton *m_connectButton = nullptr;
    QPushButton *m_disconnectButton = nullptr;
    QPushButton *m_clearTcpButton = nullptr;
    QPushButton *m_logFileButton = nullptr;
    QLabel *m_logFileLabel = nullptr;
    QLabel *m_statusLabel = nullptr;
    QSpinBox *m_reconnectIntervalSpinBox = nullptr;
    QCheckBox *m_autoReconnectCheckBox = nullptr;
    QFile m_logFile;

    // Serial
    QByteArray serialBuffer;
    QSerialPort *m_serialPort = nullptr;
    QTextEdit *m_serialWindow = nullptr;
    QComboBox *m_serialPortCombo = nullptr;
    QComboBox *m_baudCombo = nullptr;
    QComboBox *m_dataBitsCombo = nullptr;
    QComboBox *m_parityCombo = nullptr;
    QComboBox *m_stopBitsCombo = nullptr;
    QComboBox *m_flowControlCombo = nullptr;
    QPushButton *m_openSerialButton = nullptr;
    QPushButton *m_closeSerialButton = nullptr;
    QPushButton *m_clearSerialButton = nullptr;
    QPushButton *m_serialLogFileButton = nullptr;
    QLabel *m_serialLogFileLabel = nullptr;
    QLabel *m_serialStatusLabel = nullptr;
    QFile m_serialLogFile;
};

#endif // MAINWINDOW_H
