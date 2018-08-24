#include "mainwindow.h"
#include <QApplication>
#include <QStyle>

//#include <vld.h>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    w.setWindowIcon(a.style()->standardIcon(QStyle::SP_TitleBarMenuButton));
    return a.exec();
}
