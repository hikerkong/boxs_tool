#include <QApplication>
#include "MainWindow.h"

int main(int argc, char *argv[])
{
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);

    MainWindow w;
    w.resize(1200, 800);
    w.setWindowTitle("Boxs Tool");
    w.show();

    return app.exec();
}
