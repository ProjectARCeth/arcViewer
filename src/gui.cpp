#include <QApplication>
#include "ControlWindow.hpp"

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/images/arc.jpeg"));
    ControlWindow window(argc, argv);
    return app.exec();
}
