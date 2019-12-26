#include "testbench_interface/mainwindow.h"
#include <QApplication>

int main(int argc, char** argv)
{
    // Starting the node "testbench_interface".
    ros::init(argc,argv,"testbench_interface");

    // starting up the gui
    QApplication a(argc, argv);
    MainWindow w;

    w.show();

    return a.exec();
}
