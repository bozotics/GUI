#include "mainwindow.h"

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
	strcpy(config_file, getenv("HOME"));
	strcat(config_file, config_txt);
	strcpy(config_temp, getenv("HOME"));
	strcat(config_temp, config_temp_txt);

	QApplication app(argc, argv);
	MainWindow window(argc);
	window.setWindowTitle("GUI");
    window.show();
    return app.exec();
}
