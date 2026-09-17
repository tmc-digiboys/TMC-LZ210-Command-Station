#include "mainwindow.h"

#include <QTextEdit>
#include <QLineEdit>
#include <QSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QCheckBox>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QFileDialog>
#include <QDateTime>
#include <QMessageBox>
#include <QTextCursor>
#include <QSerialPortInfo>
#include <QFont>

// ============================================================
// Constructor / destructor
// ============================================================

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      m_socket(new QTcpSocket(this)),
      m_reconnectTimer(new QTimer(this)),
      m_serialPort(new QSerialPort(this))
{
    m_reconnectTimer->setSingleShot(true);

    setupUi();

    // ---------------- TCP ----------------
    connect(m_socket, &QTcpSocket::readyRead,
            this, &MainWindow::readData);

    connect(m_socket, &QTcpSocket::connected,
            this, &MainWindow::socketConnected);

    connect(m_socket, &QTcpSocket::disconnected,
            this, &MainWindow::socketDisconnected);

    connect(m_socket, &QTcpSocket::errorOccurred,
            this, &MainWindow::socketError);

    connect(m_reconnectTimer, &QTimer::timeout,
            this, &MainWindow::reconnectTcp);

    connect(m_connectButton, &QPushButton::clicked,
            this, &MainWindow::connectToServer);

    connect(m_disconnectButton, &QPushButton::clicked,
            this, &MainWindow::disconnectFromServer);

    connect(m_clearTcpButton, &QPushButton::clicked,
            this, &MainWindow::clearTcpWindow);

    connect(m_logFileButton, &QPushButton::clicked,
            this, &MainWindow::changeLogFile);

    // ---------------- Serial ----------------
    connect(m_serialPort, &QSerialPort::readyRead,
            this, &MainWindow::serialReadyRead);

    connect(m_serialPort, &QSerialPort::errorOccurred,
            this, &MainWindow::serialError);

    connect(m_openSerialButton, &QPushButton::clicked,
            this, &MainWindow::openSerialPort);

    connect(m_closeSerialButton, &QPushButton::clicked,
            this, &MainWindow::closeSerialPort);

    connect(m_clearSerialButton, &QPushButton::clicked,
            this, &MainWindow::clearSerialWindow);

    connect(m_serialLogFileButton, &QPushButton::clicked,
            this, &MainWindow::changeSerialLogFile);

    updateTcpButtons();
    updateSerialButtons();
}

MainWindow::~MainWindow()
{
    if (m_socket->state() != QAbstractSocket::UnconnectedState)
        m_socket->abort();

    if (m_reconnectTimer)
        m_reconnectTimer->stop();

    if (m_serialPort->isOpen())
        m_serialPort->close();

    if (m_logFile.isOpen())
        m_logFile.close();

    if (m_serialLogFile.isOpen())
        m_serialLogFile.close();
}

// ============================================================
// UI
// ============================================================

void MainWindow::setupUi()
{
    setWindowTitle("TCP / Serial Monitor");
    resize(1200, 700);

    auto *tabs = new QTabWidget(this);
    setCentralWidget(tabs);

    // TCP tab
    auto *tcpTab = new QWidget(this);
    auto *tcpLayout = new QHBoxLayout(tcpTab);

    m_receiveWindow = new QTextEdit(tcpTab);
    m_receiveWindow->setReadOnly(true);

    QFont font("Consolas");
    font.setStyleHint(QFont::Monospace);
    m_receiveWindow->setFont(font);

    tcpLayout->addWidget(m_receiveWindow, 1);

    auto *tcpPanel = new QWidget(tcpTab);
    tcpPanel->setFixedWidth(300);
    auto *tcpRightLayout = new QVBoxLayout(tcpPanel);

    auto *connectionLayout = new QFormLayout();

    m_ipEdit = new QLineEdit("192.168.54.102", tcpPanel);

    m_portSpinBox = new QSpinBox(tcpPanel);
    m_portSpinBox->setRange(1, 65535);
    m_portSpinBox->setValue(23456);

    connectionLayout->addRow("IP-adres:", m_ipEdit);
    connectionLayout->addRow("Poort:", m_portSpinBox);

    tcpRightLayout->addLayout(connectionLayout);

    m_autoReconnectCheckBox = new QCheckBox("Automatisch reconnecten", tcpPanel);
    m_autoReconnectCheckBox->setChecked(true);
    tcpRightLayout->addWidget(m_autoReconnectCheckBox);

    auto *reconnectLayout = new QFormLayout();

    m_reconnectIntervalSpinBox = new QSpinBox(tcpPanel);
    m_reconnectIntervalSpinBox->setRange(1, 3600);
    m_reconnectIntervalSpinBox->setValue(5);
    m_reconnectIntervalSpinBox->setSuffix(" s");

    reconnectLayout->addRow("Reconnect interval:", m_reconnectIntervalSpinBox);
    tcpRightLayout->addLayout(reconnectLayout);

    m_connectButton = new QPushButton("Connect", tcpPanel);
    m_disconnectButton = new QPushButton("Disconnect", tcpPanel);
    m_clearTcpButton = new QPushButton("Clear display", tcpPanel);

    tcpRightLayout->addWidget(m_connectButton);
    tcpRightLayout->addWidget(m_disconnectButton);
    tcpRightLayout->addWidget(m_clearTcpButton);

    tcpRightLayout->addSpacing(15);

    tcpRightLayout->addWidget(new QLabel("TCP logfile:", tcpPanel));

    m_logFileLabel = new QLabel("Geen logfile geselecteerd", tcpPanel);
    m_logFileLabel->setWordWrap(true);
    tcpRightLayout->addWidget(m_logFileLabel);

    m_logFileButton = new QPushButton("Wijzig logfile...", tcpPanel);
    tcpRightLayout->addWidget(m_logFileButton);

    tcpRightLayout->addSpacing(15);

    m_statusLabel = new QLabel("Niet verbonden", tcpPanel);
    m_statusLabel->setWordWrap(true);
    tcpRightLayout->addWidget(m_statusLabel);

    tcpRightLayout->addStretch();

    tcpLayout->addWidget(tcpPanel);

    tabs->addTab(tcpTab, "TCP");

    // Serial tab
    auto *serialTab = new QWidget(this);
    auto *serialLayout = new QHBoxLayout(serialTab);

    m_serialWindow = new QTextEdit(serialTab);
    m_serialWindow->setReadOnly(true);
    m_serialWindow->setFont(font);

    serialLayout->addWidget(m_serialWindow, 1);

    auto *serialPanel = new QWidget(serialTab);
    serialPanel->setFixedWidth(300);
    auto *serialRightLayout = new QVBoxLayout(serialPanel);

    auto *serialSettings = new QFormLayout();

    m_serialPortCombo = new QComboBox(serialPanel);
    serialSettings->addRow("Seriële poort:", m_serialPortCombo);

    m_baudCombo = new QComboBox(serialPanel);
    m_baudCombo->addItems({
        "1200", "2400", "4800", "9600", "19200",
        "38400", "57600", "115200", "230400"
    });
    m_baudCombo->setCurrentText("9600");
    serialSettings->addRow("Baudrate:", m_baudCombo);

    m_dataBitsCombo = new QComboBox(serialPanel);
    m_dataBitsCombo->addItems({"5", "6", "7", "8"});
    m_dataBitsCombo->setCurrentText("8");
    serialSettings->addRow("Databits:", m_dataBitsCombo);

    m_parityCombo = new QComboBox(serialPanel);
    m_parityCombo->addItems({"None", "Even", "Odd", "Mark", "Space"});
    serialSettings->addRow("Parity:", m_parityCombo);

    m_stopBitsCombo = new QComboBox(serialPanel);
    m_stopBitsCombo->addItems({"1", "1.5", "2"});
    m_stopBitsCombo->setCurrentText("1");
    serialSettings->addRow("Stopbits:", m_stopBitsCombo);

    m_flowControlCombo = new QComboBox(serialPanel);
    m_flowControlCombo->addItems({"None", "RTS/CTS", "XON/XOFF"});
    serialSettings->addRow("Flow control:", m_flowControlCombo);

    serialRightLayout->addLayout(serialSettings);

    m_openSerialButton = new QPushButton("Open serial port", serialPanel);
    m_closeSerialButton = new QPushButton("Close serial port", serialPanel);
    m_clearSerialButton = new QPushButton("Clear display", serialPanel);

    serialRightLayout->addWidget(m_openSerialButton);
    serialRightLayout->addWidget(m_closeSerialButton);
    serialRightLayout->addWidget(m_clearSerialButton);

    serialRightLayout->addSpacing(15);

    serialRightLayout->addWidget(new QLabel("Seriële logfile:", serialPanel));

    m_serialLogFileLabel = new QLabel("Geen logfile geselecteerd", serialPanel);
    m_serialLogFileLabel->setWordWrap(true);
    serialRightLayout->addWidget(m_serialLogFileLabel);

    m_serialLogFileButton = new QPushButton("Wijzig logfile...", serialPanel);
    serialRightLayout->addWidget(m_serialLogFileButton);

    serialRightLayout->addSpacing(15);

    m_serialStatusLabel = new QLabel("Poort gesloten", serialPanel);
    m_serialStatusLabel->setWordWrap(true);
    serialRightLayout->addWidget(m_serialStatusLabel);

    serialRightLayout->addStretch();

    serialLayout->addWidget(serialPanel);

    tabs->addTab(serialTab, "Serial");

    // Vul de beschikbare COM/tty-poorten.
    const auto ports = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : ports)
        m_serialPortCombo->addItem(
            QString("%1 (%2)")
                .arg(info.portName(), info.description()),
            info.portName());

    if (m_serialPortCombo->count() == 0)
        m_serialPortCombo->addItem("Geen seriële poorten gevonden");
}

void MainWindow::setupTcpTab()
{
    // Gereserveerd voor eventuele toekomstige uitbreidingen.
}

void MainWindow::setupSerialTab()
{
    // Gereserveerd voor eventuele toekomstige uitbreidingen.
}

// ============================================================
// TCP
// ============================================================

void MainWindow::connectToServer()
{
    m_tcpManualDisconnect = false;

    if (m_socket->state() != QAbstractSocket::UnconnectedState)
        return;

    m_reconnectTimer->stop();

    const QString ip = m_ipEdit->text().trimmed();
    const quint16 port =
        static_cast<quint16>(m_portSpinBox->value());

    m_statusLabel->setText(
        QString("Verbinden met %1:%2...")
            .arg(ip)
            .arg(port));

    m_socket->connectToHost(ip, port);
}

void MainWindow::disconnectFromServer()
{
    // Expliciete disconnect voorkomt automatische reconnect.
    m_tcpManualDisconnect = true;
    m_reconnectTimer->stop();

    if (m_socket->state() != QAbstractSocket::UnconnectedState)
        m_socket->disconnectFromHost();

    m_statusLabel->setText("Handmatig verbroken");
    updateTcpButtons();
}

void MainWindow::socketConnected()
{
    m_statusLabel->setText("Verbonden");

    m_tcpManualDisconnect = false;
    m_reconnectTimer->stop();

    updateTcpButtons();

    appendTcpText(
        QString("[%1] TCP verbinding geopend\n")
            .arg(createTimestamp()));
}

void MainWindow::socketDisconnected()
{
    updateTcpButtons();

    if (m_tcpManualDisconnect) {
        m_statusLabel->setText("Handmatig verbroken");
        return;
    }

    m_statusLabel->setText("Verbinding verbroken");

    appendTcpText(
        QString("[%1] TCP verbinding verbroken\n")
            .arg(createTimestamp()));

    if (m_autoReconnectCheckBox->isChecked())
        m_reconnectTimer->start(
            m_reconnectIntervalSpinBox->value() * 1000);
}

void MainWindow::socketError(
    QAbstractSocket::SocketError socketError)
{
    Q_UNUSED(socketError);

    m_statusLabel->setText(
        "TCP fout: " + m_socket->errorString());

    updateTcpButtons();

    if (!m_tcpManualDisconnect &&
        m_autoReconnectCheckBox->isChecked()) {

        if (!m_reconnectTimer->isActive())
            m_reconnectTimer->start(
                m_reconnectIntervalSpinBox->value() * 1000);
    }
}

void MainWindow::reconnectTcp()
{
    if (m_tcpManualDisconnect ||
        !m_autoReconnectCheckBox->isChecked())
        return;

    if (m_socket->state() == QAbstractSocket::UnconnectedState) {
        m_statusLabel->setText("Automatisch reconnecten...");
        connectToServer();
    }
}

void MainWindow::readData()
{
    const QByteArray data = m_socket->readAll();

    if (data.isEmpty())
        return;

    QString receivedText = QString::fromUtf8(data);

    if (!receivedText.endsWith('\n'))
        receivedText.append('\n');

    const QString text =
        QString("[%1] %2")
            .arg(createTimestamp(), receivedText);

    appendTcpText(text);
    writeToLog(text);
}

void MainWindow::clearTcpWindow()
{
    m_receiveWindow->clear();
}

void MainWindow::updateTcpButtons()
{
    const bool connected =
        m_socket->state() == QAbstractSocket::ConnectedState;

    const bool connecting =
        m_socket->state() == QAbstractSocket::ConnectingState;

    m_connectButton->setEnabled(!connected && !connecting);
    m_disconnectButton->setEnabled(connected || connecting);
}

// ============================================================
// TCP logfile
// ============================================================

void MainWindow::writeToLog(const QString &text)
{
    if (!m_logFile.isOpen())
        return;

    m_logFile.write(text.toUtf8());
    m_logFile.flush();
}

void MainWindow::changeLogFile()
{
    const QString fileName =
        QFileDialog::getSaveFileName(
            this,
            "Selecteer TCP logfile",
            "",
            "Log files (*.log);;Text files (*.txt);;All files (*.*)");

    if (fileName.isEmpty())
        return;

    if (m_logFile.isOpen())
        m_logFile.close();

    m_logFile.setFileName(fileName);

    if (!m_logFile.open(
            QIODevice::WriteOnly |
            QIODevice::Append |
            QIODevice::Text)) {

        QMessageBox::critical(
            this,
            "Fout",
            "Kan TCP logfile niet openen:\n" +
                m_logFile.errorString());

        return;
    }

    m_logFileLabel->setText(fileName);

    const QString message =
        QString("[%1] TCP logfile geopend: %2\n")
            .arg(createTimestamp(), fileName);

    appendTcpText(message);
    writeToLog(message);
}

// ============================================================
// Serial
// ============================================================

void MainWindow::openSerialPort()
{
    if (m_serialPort->isOpen())
        return;

    if (m_serialPortCombo->currentData().isNull()) {
        m_serialStatusLabel->setText("Geen seriële poort geselecteerd");
        return;
    }

    const QString portName =
        m_serialPortCombo->currentData().toString();

    if (portName.isEmpty() || portName == "Geen seriële poorten gevonden") {
        m_serialStatusLabel->setText("Geen seriële poort geselecteerd");
        return;
    }

    m_serialPort->setPortName(portName);

    m_serialPort->setBaudRate(
        m_baudCombo->currentText().toInt());

    switch (m_dataBitsCombo->currentText().toInt()) {
    case 5: m_serialPort->setDataBits(QSerialPort::Data5); break;
    case 6: m_serialPort->setDataBits(QSerialPort::Data6); break;
    case 7: m_serialPort->setDataBits(QSerialPort::Data7); break;
    default: m_serialPort->setDataBits(QSerialPort::Data8); break;
    }

    if (m_parityCombo->currentText() == "Even")
        m_serialPort->setParity(QSerialPort::EvenParity);
    else if (m_parityCombo->currentText() == "Odd")
        m_serialPort->setParity(QSerialPort::OddParity);
    else if (m_parityCombo->currentText() == "Mark")
        m_serialPort->setParity(QSerialPort::MarkParity);
    else if (m_parityCombo->currentText() == "Space")
        m_serialPort->setParity(QSerialPort::SpaceParity);
    else
        m_serialPort->setParity(QSerialPort::NoParity);

    if (m_stopBitsCombo->currentText() == "1.5")
        m_serialPort->setStopBits(QSerialPort::OneAndHalfStop);
    else if (m_stopBitsCombo->currentText() == "2")
        m_serialPort->setStopBits(QSerialPort::TwoStop);
    else
        m_serialPort->setStopBits(QSerialPort::OneStop);

    if (m_flowControlCombo->currentText() == "RTS/CTS")
        m_serialPort->setFlowControl(QSerialPort::HardwareControl);
    else if (m_flowControlCombo->currentText() == "XON/XOFF")
        m_serialPort->setFlowControl(QSerialPort::SoftwareControl);
    else
        m_serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (!m_serialPort->open(QIODevice::ReadOnly)) {
        m_serialStatusLabel->setText(
            "Fout: " + m_serialPort->errorString());
        updateSerialButtons();
        return;
    }

    m_serialStatusLabel->setText(
        QString("Verbonden met %1").arg(portName));

    appendSerialText(
        QString("[%1] Seriële poort geopend: %2\n")
            .arg(createTimestamp(), portName));

    updateSerialButtons();
}

void MainWindow::closeSerialPort()
{
    if (!m_serialPort->isOpen())
        return;

    const QString portName = m_serialPort->portName();

    m_serialPort->close();

    appendSerialText(
        QString("[%1] Seriële poort gesloten: %2\n")
            .arg(createTimestamp(), portName));

    m_serialStatusLabel->setText("Poort gesloten");
    updateSerialButtons();
}

void MainWindow::serialReadyRead()
{
    serialBuffer.append(m_serialPort->readAll());

    // Verwerk complete regels
    while (serialBuffer.contains('\n')) {
        const int endOfLine = serialBuffer.indexOf('\n');

        QByteArray line = serialBuffer.left(endOfLine + 1);
        serialBuffer.remove(0, endOfLine + 1);

        // CR/LF of LF verwijderen
        //line = line.trimmed();

        if (line.isEmpty())
            continue;

        // Ontvang de data als tekst
        const QString receivedText = QString::fromUtf8(line);

        const QString text =
            QString("[%1] %2")
                .arg(createTimestamp(), receivedText);

        appendSerialText(text);
        writeSerialToLog(text);
    }
}

void MainWindow::serialError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::NoError)
        return;

    const QString message =
        QString("[%1] Serial fout: %2\n")
            .arg(createTimestamp(),
                 m_serialPort->errorString());

    appendSerialText(message);
    writeSerialToLog(message);

    // Bij een fysieke disconnect kan Qt de poort sluiten.
    if (error == QSerialPort::ResourceError ||
        error == QSerialPort::DeviceNotFoundError) {

        if (m_serialPort->isOpen())
            m_serialPort->close();

        m_serialStatusLabel->setText(
            "Seriële verbinding verbroken");

        updateSerialButtons();
    }
}

void MainWindow::clearSerialWindow()
{
    m_serialWindow->clear();
}

void MainWindow::updateSerialButtons()
{
    const bool open = m_serialPort->isOpen();

    m_openSerialButton->setEnabled(!open);
    m_closeSerialButton->setEnabled(open);

    m_serialPortCombo->setEnabled(!open);
    m_baudCombo->setEnabled(!open);
    m_dataBitsCombo->setEnabled(!open);
    m_parityCombo->setEnabled(!open);
    m_stopBitsCombo->setEnabled(!open);
    m_flowControlCombo->setEnabled(!open);
}

// ============================================================
// Serial logfile
// ============================================================

void MainWindow::writeSerialToLog(const QString &text)
{
    if (!m_serialLogFile.isOpen())
        return;

    m_serialLogFile.write(text.toUtf8());
    m_serialLogFile.flush();
}

void MainWindow::changeSerialLogFile()
{
    const QString fileName =
        QFileDialog::getSaveFileName(
            this,
            "Selecteer seriële logfile",
            "",
            "Log files (*.log);;Text files (*.txt);;All files (*.*)");

    if (fileName.isEmpty())
        return;

    if (m_serialLogFile.isOpen())
        m_serialLogFile.close();

    m_serialLogFile.setFileName(fileName);

    if (!m_serialLogFile.open(
            QIODevice::WriteOnly |
            QIODevice::Append |
            QIODevice::Text)) {

        QMessageBox::critical(
            this,
            "Fout",
            "Kan seriële logfile niet openen:\n" +
                m_serialLogFile.errorString());

        return;
    }

    m_serialLogFileLabel->setText(fileName);

    const QString message =
        QString("[%1] Seriële logfile geopend: %2\n")
            .arg(createTimestamp(), fileName);

    appendSerialText(message);
    writeSerialToLog(message);
}

// ============================================================
// Common helpers
// ============================================================

QString MainWindow::createTimestamp() const
{
    return QDateTime::currentDateTime()
        .toString("yyyy-MM-dd HH:mm:ss.zzz");
}

void MainWindow::appendTcpText(const QString &text)
{
    m_receiveWindow->moveCursor(QTextCursor::End);
    m_receiveWindow->insertPlainText(text);
    m_receiveWindow->moveCursor(QTextCursor::End);
}

void MainWindow::appendSerialText(const QString &text)
{
    m_serialWindow->moveCursor(QTextCursor::End);
    m_serialWindow->insertPlainText(text);
    m_serialWindow->moveCursor(QTextCursor::End);
}
