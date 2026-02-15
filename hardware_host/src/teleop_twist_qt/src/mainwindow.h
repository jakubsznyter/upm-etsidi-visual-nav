#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QtGamepad/QGamepad>
#include <QtGamepad/QGamepadManager>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"



QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE
//Se definen los modos
enum GamepadMode {ARCADE, INDIVIDUAL};

class MainWindow : public QMainWindow, public rclcpp::Node
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();  // sLOT PARA BOTON STOP
    void on_Timer(); // SLOT TIMER
 private:
    Ui::MainWindow *ui;
    QTimer *timer=nullptr;
    QGamepad *gamepad=nullptr;
    GamepadMode gamepadmode=ARCADE;
    // METODOS DE CONTROL
    void virtualGamePadControl();// CONTROL VIRTUAL DE JOYSTICK
    void handleGamePadStatus(); // MANEJAR ESTADO GAMEPAD
    void gamePadControl(); // CONTROL DEL GMEPAD
    //ROS2 components
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};
#endif // MAINWINDOW_H
