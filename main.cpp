#include "mainwindow.h"
namespace plt = matplotlibcpp;

int main(int argc, char *argv[])
{
	/*std::vector<int> x, y, u, v;
    for (int i = -5; i <= 5; i++) {
        for (int j = -5; j <= 5; j++) {
            x.push_back(i);
            u.push_back(-i);
            y.push_back(j);
            v.push_back(-j);
        }
    }

    plt::quiver(x, y, u, v);
    plt::show();*/
	QApplication app(argc, argv);
	MainWindow window(argc);
	window.setWindowTitle("GUI");
    window.show();
    return app.exec();
}
