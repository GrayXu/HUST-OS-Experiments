#include "mainwindow.h"
#include <QApplication>
#include "unistd.h"
#include <qtimer.h>

int pid1;
int pid2;
int pid3;

void sub1();
void sub2();
void sub3();

MainWindow * wP;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow w;
    wP = &w;

    w.show();
    return a.exec();
}
