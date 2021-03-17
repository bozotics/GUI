#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QApplication>
#include <QtWidgets/QMainWindow>
#include <QLabel>
#include <QWidget>
#include <QPixmap>
#include <QObject>
#include <QSpinBox>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <QLayout>
#include <QDebug>
#include <QThread>
#include <QDoubleSpinBox>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <memory>
#include <iostream>
#include <QFile>
#include <QMessageBox>
#include <QPushButton>
#include <QProcess>
#include <libssh/libsshpp.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <QComboBox>
#include <notepad.h>
#include <QTextStream>
#include <fstream>
#include <QMenu>
#include <QAction>
#include <QMenuBar>
#include <QStandardPaths>


/*QT_BEGIN_NAMESPACE
class QAction;
class QActionGroup;
class QLabel;
class QMenu;
class QPixmap;
class QObject;
class QWidget;
class QSpinBox;
class QLayout;
class QDebug;
class QThread;
class QFile;
class QMessageBox;
class QPushButton;
class QProcess;
class QComboBox;
class QTextStream;
QT_END_NAMESPACE*/

using namespace std;
using namespace cv;

class sshComm : public QObject{
		Q_OBJECT
	public:
		explicit sshComm(QObject *parent = nullptr);
		QThread *sThread;
		ssh_session session;
		ssh_channel channel;
		bool make = false;
		int cnt = 0;
	signals: 
		void afterMake();
		
	public slots:
		void startCamera();
		void stopCamera();
	
	private:
		int vbs = SSH_LOG_RARE;
		bool stopped = false;
};

class socketServer : public QObject{
		Q_OBJECT	
	public:
  		explicit socketServer(QObject *parent = nullptr);
  		void connectToNetwork();
		void receiveImageDims();
		int receiveImage(Mat& image);
		void sendCommands(int type, int value);
		void showImage(Mat& image);
		int getWidth();
		int getHeight();		
		int sock_connect;
		int sock_init;
		socklen_t client_len;
		struct sockaddr_in client_addr;

	private:
		struct sockaddr_in server_addr;
		size_t server_addr_size;
		int port;
		int sock;
		Size2i image_dims;
		int16_t conv;
};	

class videoStream : public QObject{
    Q_OBJECT
	public:
    	explicit videoStream(QObject *parent = nullptr);	
		QThread* vThread;
		int mode;
		Mat clrValue;
		bool drawRect = false;
		QPoint point1;
		QPoint point2;
		socketServer *server_ptr;
		bool stopped = false;
	
	signals:
    	void display(QPixmap pixmap);
		void display2(QPixmap pixmap);

	public slots:
	    void startVideo();
	    void stopVideo();
};



class MainWindow : public QMainWindow{
	Q_OBJECT
	public:
		explicit MainWindow(int, QWidget *parent=nullptr);
		~MainWindow();

	private:
		void initUI(int arguments);
		void initLayout();
		void initSignal(int arguments);
		void initThreads(int arguments);
		void initMenu();
		void initActions();
		bool eventFilter(QObject* obj, QEvent *event);
		void mousePressEvent(QMouseEvent *e);
		void mouseReleaseEvent(QMouseEvent *e);
		void mouseMoveEvent(QMouseEvent *e);
		void readConfig();
	
	signals:
		void curText(QString pos, QString color);
		void roiText(QString roi);

	public slots:
		void updateCursor(QString pos, QString color);
		void updateRoi(QString roi);

	private slots:
		void spinBoxUpdate(int value);
		void comboBoxUpdate(QString mode);
		void comboBoxUpdate2(QString mode);
		void trackTypeUpdate(QString mode);
		void make();
		void saveConfig();
		void copyVal();
		void showDScreen();
		void showCamCal();
		void pauseCam();

	private:
		QMenu *sendFiles;
		QMenu *actions;
		QMenu *advanced;
		QPoint cursor;
		QLabel *frame;
		QLabel *frame2;
		QLabel *cursorPos;
		QLabel *cursorColor;
		QLabel *roiColor;
		QLabel *presetLabel;
		QLabel *presetLabel2;
		QVBoxLayout *main_layout; 
		QVBoxLayout *note_layout;
		QVBoxLayout *left_layout;
		QHBoxLayout *header_box;
		QHBoxLayout *sub_layout;
		QHBoxLayout *cursor_details;
		QHBoxLayout *hi_box;
		QHBoxLayout *lo_box;
		QHBoxLayout *preset_box;
		QHBoxLayout *preset_box2;
		QHBoxLayout *save_bar;
		videoStream *videoPi;
		sshComm *ssh;
		QWidget *centralWidget;
		notePad *note;
		QAction *camera_c;
		QAction *camera_h;
		QAction *main_c;
		QAction *config;
		QAction *debug_c;
		QAction *makeFile;
		QPushButton *copyThres;
		QAction *saveVal;
		QAction *dualScreen;
		QAction *camCal;
		QAction *pauseCamera;
		QList<QSpinBox*> spinBox;
		QStringList commands = { "orginal", "cvtColor", "Blur", "inRange", "er dl", "contour", "Rect"};
		//QStringList configFormat = { "Lab", "Comps"};
		QStringList thresList = {"Ball", "Blue Goal", "Yellow Goal", "Field", "All"};
		QComboBox* debugFrame2;
		QComboBox* debugFrame;
		QComboBox* trackType;
		QPoint point1;
		QPoint point2;
		int sbVal[13];
		int threshold[6];
		int allThres[24];
};

const string config_txt = "/cache/current/config.txt", config_temp_txt = "/cache/current/config_temp.txt";
const string camera_cpp = "/cache/current/camera.cpp", camera_h = "/cache/current/camera.h";
const string debug_cpp = "/cache/current/debug.cpp", main_cpp = "/cache/current/main.cpp";

#endif

