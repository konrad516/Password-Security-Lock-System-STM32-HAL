#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QList>
#include <QDateTime>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->discoveryAgent = new QBluetoothDeviceDiscoveryAgent(this);
    connect(this->discoveryAgent, SIGNAL(deviceDiscovered(QBluetoothDeviceInfo)),
            this, SLOT(captureDeviceProperties(QBluetoothDeviceInfo)));

    connect(this->discoveryAgent, SIGNAL(finished()),
            this, SLOT(searchingFinished()));

    this->socket = new QBluetoothSocket(QBluetoothServiceInfo::RfcommProtocol, this);
    connect(this->socket, SIGNAL(connected()),
            this, SLOT(connectionEstablished()));
    connect(this->socket, SIGNAL(disconnected()),
            this, SLOT(connectionInterrupted()));
    connect(this->socket, SIGNAL(readyRead()),
            this, SLOT(socketReadyToRead()));
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButtonSearch_clicked()
{
    addToLogs("Searching devices");
    ui->comboBoxDevices->clear();
    ui->pushButtonSearch->setEnabled(false);
    this->discoveryAgent->start();
}

void MainWindow::addToLogs(QString message)
{
    QString currentDateTime = QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss");
    ui->textEditLogs->append(currentDateTime + "\t" + message);
   // ui->lcdNumberTemperature->display();
}

void MainWindow::sendMessageToDevice(QString message)
{
    if(this->socket->isOpen() && this->socket->isWritable()) {
    //  this->addToLogs("Sending message: " + message);
      this->socket->write(message.toStdString().c_str());
    } else {
      this->addToLogs("Can't send message. Connection is not set!");
    }
}

void MainWindow::on_pushButtonConnect_clicked()
{
    QString comboBoxQString = ui->comboBoxDevices->currentText();
    QStringList portList = comboBoxQString.split(" ");
    QString deviceAddres = portList.last();

    static const QString serviceUuid(QStringLiteral("00001101-0000-1000-8000-00805F9B34FB"));
    this->socket->connectToService(QBluetoothAddress(deviceAddres),QBluetoothUuid(serviceUuid),QIODevice::ReadWrite);
    this->addToLogs("Wait for connection with: " + portList.first() + " address: " + deviceAddres);
}

void MainWindow::on_pushButtonDisconnect_clicked()
{
    this->addToLogs("Connection close.");
    this->socket->disconnectFromService();
}

void MainWindow::readFromPort()
{
}

void MainWindow::on_pushButtonOpen_clicked()
{
    this->addToLogs("Open door.");
    this->sendMessageToDevice("0");
}

void MainWindow::captureDeviceProperties(const QBluetoothDeviceInfo &device)
{
    ui->comboBoxDevices->addItem(device.name() + " " + device.address().toString());
}

void MainWindow::searchingFinished()
{
    ui->pushButtonSearch->setEnabled(true);
}

void MainWindow::connectionEstablished()
{
  this->addToLogs("Successful connection.");
}

void MainWindow::connectionInterrupted()
{
  this->addToLogs("Connection has been interrupted.");
}

void MainWindow::socketReadyToRead()
{
    while(this->socket->canReadLine()) {
      QString line = this->socket->readLine();
      qDebug()<<line;
      QString terminator = "\r";
      int pos = line.lastIndexOf(terminator);

      if(line.left(pos)!="")
         this->addToLogs(line.left(pos));
    }
}

void MainWindow::on_pushButtonGetTemperature_clicked()
{
      this->sendMessageToDevice("1");
}
