#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtBluetooth>
#include <QBluetoothSocket>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButtonSearch_clicked();

    void on_pushButtonConnect_clicked();

    void on_pushButtonDisconnect_clicked();

    void readFromPort();

    void on_pushButtonOpen_clicked();

    void captureDeviceProperties(const QBluetoothDeviceInfo &device);

    void searchingFinished();

    void connectionEstablished();
    void connectionInterrupted();
    void socketReadyToRead();

    void on_pushButtonGetTemperature_clicked();

private:
    Ui::MainWindow *ui;
    QBluetoothDeviceDiscoveryAgent *discoveryAgent;
    void addToLogs(QString message);
    void sendMessageToDevice(QString message);
    QBluetoothSocket *socket;
};
#endif // MAINWINDOW_H
