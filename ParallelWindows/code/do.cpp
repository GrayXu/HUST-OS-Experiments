#include <do.h>

Do::Do(QObject *parent):QObject(parent){

}

void Do::sub1(){
    int count = 1;
//    for(count=1;count<=1000;count++)
}

void Do::sub2(){
    int count = 0;
    while(1){
        if(count == 9) count = 0;
        else count++;
        QString str;
        str.sprintf("%d",count);
        (*wP).update2(str);
    }
}

void Do::sub3(){
    int count = 1;
//    for(count=1;count<=1000;count++)
}
