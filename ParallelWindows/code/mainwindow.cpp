#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "unistd.h"
#include <qtimer.h>
#include <qdatetime.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QString str1 = "hello world";
    ui->textBrowser->insertPlainText(str1);

    QTimer *timer = new QTimer();
    connect(timer, SIGNAL(timeout()), this,SLOT(update1()));
    timer->start(1000);

//    update1();
}

int count = 1;
int sum = 0;

void MainWindow::update1(){
//    QDateTime time = QDateTime::currentDateTime();
//    QString str = time.toString("yyyy-MM-dd hh:mm:ss dddd");
    if(count <= 1000){
        QString str;
        sum += count;
        str.sprintf("%d",sum);
        ui->textBrowser->clear();
        ui->textBrowser->insertPlainText(str);
        count++;
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
