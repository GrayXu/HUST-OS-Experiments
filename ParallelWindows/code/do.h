#include <QObject>

class Do: public QObject
{
    Q_OBJECT

public:
    explicit Do(QObject *parent = 0);

    void sub1();
    void sub2();
    void sub3();

public slots:
};
