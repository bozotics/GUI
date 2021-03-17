#include "mainwindow.h"

void MainWindow::initUI(int arguments){
	videoPi = new videoStream();
	if (arguments == 1) ssh = new sshComm();
	note = new notePad();
}

void MainWindow::initThreads(int arguments){
	videoPi -> vThread = new QThread();
	videoPi -> moveToThread(videoPi -> vThread);
	if (arguments == 1){
		ssh -> sThread = new QThread();
		ssh -> moveToThread(ssh -> sThread);
	}
	if(arguments == 1){
		connect(ssh -> sThread,
    	        SIGNAL(started()),
    	        ssh,
    	        SLOT(startCamera()));

    	connect(ssh -> sThread,
    	        SIGNAL(finished()),
    	        ssh,
    	        SLOT(deleteLater()));
	}
}

void MainWindow::initSignal(int arguments){
	connect(videoPi -> vThread,
            SIGNAL(started()),
            videoPi,
            SLOT(startVideo()));
	
    connect(videoPi -> vThread,
            SIGNAL(finished()),
            videoPi,
            SLOT(deleteLater()));
	
    connect(videoPi,
            SIGNAL(display(QPixmap)),
            frame,
            SLOT(setPixmap(QPixmap)));

	connect(ssh,
            SIGNAL(afterMake()),
            this,
            SLOT(close()));
	
	connect(videoPi,
            SIGNAL(display2(QPixmap)),
            frame2,
            SLOT(setPixmap(QPixmap)));

	connect(this,
            SIGNAL(curText(QString, QString)),
            this,
            SLOT(updateCursor(QString, QString)));

	connect(this,
            SIGNAL(roiText(QString)),
            this,
            SLOT(updateRoi(QString)));

	connect(main_c, 
			SIGNAL(triggered()),
			note,
			SLOT(main_c()));

	connect(camera_c, 
			SIGNAL(triggered()),
			note,
			SLOT(camera_c()));

	connect(camera_h, 
			SIGNAL(triggered()),
			note,
			SLOT(camera_h()));

	connect(config, 
			SIGNAL(triggered()),
			note,
			SLOT(config()));

	connect(debug_c, 
			SIGNAL(triggered()),
			note,
			SLOT(debug_c()));

	connect(makeFile, 
			SIGNAL(triggered()),
			this,
			SLOT(make()));
	connect(pauseCamera, 
			SIGNAL(triggered()),
			this,
			SLOT(pauseCam()));
	connect(copyThres, 
			SIGNAL(clicked()),
			this,
			SLOT(copyVal()));
	connect(saveVal,
			SIGNAL(triggered()),
			this,
			SLOT(saveConfig()));
	connect(dualScreen,
			SIGNAL(triggered()),
			this,
			SLOT(showDScreen()));
	connect(camCal,
			SIGNAL(triggered()),
			this,
			SLOT(showCamCal()));
	QSpinBox* spin;
	//cout << "init signals\n";
    for(const auto spin : spinBox)
        QObject::connect(spin, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &MainWindow::spinBoxUpdate);

	connect(debugFrame, SIGNAL(currentTextChanged(QString)), this, SLOT(comboBoxUpdate(QString)));
	connect(debugFrame2, SIGNAL(currentTextChanged(QString)), this, SLOT(comboBoxUpdate2(QString)));
	connect(trackType, SIGNAL(currentTextChanged(QString)), this, SLOT(trackTypeUpdate(QString)));

}

void MainWindow::spinBoxUpdate(int value)
{	
	int location;
	QSpinBox* sp = qobject_cast<QSpinBox*>(sender());
	sbVal[sp->objectName().toInt()] = value;
	if(sp->objectName().toInt() <= 5){
		location = trackType -> currentIndex() * 6 + sp->objectName().toInt();
		if(trackType -> currentIndex() <= 4){
			allThres[location] = value;
			videoPi -> server_ptr -> sendCommands(location + 1, value);
		}
	}
	else{
		location = sp->objectName().toInt();
		videoPi -> server_ptr -> sendCommands(location + 19, value);
	}
}

void MainWindow::comboBoxUpdate(QString mode)
{
	videoPi -> server_ptr -> sendCommands(32, debugFrame -> currentIndex());
}

void MainWindow::comboBoxUpdate2(QString mode)
{
	videoPi -> server_ptr -> sendCommands(33, debugFrame2 -> currentIndex());
}

void MainWindow::trackTypeUpdate(QString mode)
{

	videoPi -> server_ptr -> sendCommands(34, trackType -> currentIndex());
	for(int i = 0; i < 6; ++i){
		sbVal[i] = allThres[trackType -> currentIndex() * 6 + i];
		if(mode != "All") spinBox[i] -> setValue(sbVal[i]);
	}
}

void MainWindow::showDScreen()
{
	if(debugFrame2 -> isVisible())
	{
		debugFrame2 -> setVisible(false);
		frame2 -> setVisible(false);
		note -> editor -> setVisible(false);
	}
	else{
		debugFrame2 -> setVisible(true);
		frame2 -> setVisible(true);
		note -> editor -> setVisible(true);
	}
	centralWidget -> adjustSize();
	adjustSize();
}

void MainWindow::showCamCal()
{
	if(presetLabel-> isVisible())
	{
		presetLabel -> setVisible(false);
		presetLabel2 -> setVisible(false);
		for(int i = 6; i < 13; ++i) spinBox[i] -> setVisible(false);
	}
	else{
		presetLabel -> setVisible(true);
		presetLabel2 -> setVisible(true);
		for(int i = 6; i < 13; ++i) spinBox[i] -> setVisible(true);
	}
	centralWidget -> adjustSize();
	adjustSize();
}

void MainWindow::pauseCam()
{
	videoPi -> server_ptr -> sendCommands(0, 0);
	videoPi->stopVideo();
}

void MainWindow::initActions()
{
	camera_c = new QAction(tr("camera.cpp"), this);
	camera_h = new QAction(tr("camera.h"), this);
	main_c = new QAction(tr("main.cpp"), this);
	debug_c = new QAction(tr("debug.cpp"), this);
	config = new QAction(tr("config.txt"), this);
	saveVal = new QAction (tr("Save All"), this);

	makeFile = new QAction("make", this);
	pauseCamera = new QAction("Pause Stream", this);

	dualScreen = new QAction(tr("Dual Screen"), this);
	camCal = new QAction(tr("Camera Calibration"), this);
}

void MainWindow::initMenu()
{
	sendFiles = menuBar()->addMenu(tr("Send"));
	sendFiles -> addAction(camera_c);
	sendFiles -> addAction(camera_h);
	sendFiles -> addAction(main_c);
	sendFiles -> addAction(debug_c);
	sendFiles -> addSeparator();
	sendFiles -> addAction(saveVal);
	sendFiles -> addAction(config);
	
	actions = menuBar()->addMenu(tr("Actions"));
	actions -> addAction(makeFile);
	actions -> addAction(pauseCamera);
	
	advanced = menuBar()->addMenu(tr("Advanced"));
	advanced -> addAction(dualScreen);
	advanced -> addAction(camCal);
}

void MainWindow::initLayout(){
	centralWidget = new QWidget();
	setCentralWidget(centralWidget);
	main_layout = new QVBoxLayout();
	sub_layout = new QHBoxLayout();
	left_layout = new QVBoxLayout();
	note_layout = new QVBoxLayout();
	header_box = new QHBoxLayout();
	cursor_details = new QHBoxLayout();
	save_bar = new QHBoxLayout();
	frame = new QLabel(this);
	frame2 = new QLabel(this);
	cursorColor = new QLabel(this);
	cursorColor -> setAlignment(Qt::AlignRight);
	cursorPos = new QLabel(this);
	cursorPos -> setAlignment(Qt::AlignLeft);
	roiColor = new QLabel(this);
	roiColor -> setAlignment(Qt::AlignCenter);
	copyThres = new QPushButton("CopyThres", this);
	QLabel *highLabel = new QLabel(tr("Upper Threshold (BGR, HSV, Lab): "));
	QLabel *lowLabel = new QLabel(tr("Lower Threshold (BGR, HSV, Lab): "));
	presetLabel = new QLabel(tr("Camera config (AWB_R, AWB_B, exposure): "));
	presetLabel2 = new QLabel(tr("Camera config (Brightness, saturation, shutterSpeed, ISO): "));
	for( auto i = 0; i < 13; ++i ){
   		QSpinBox *sb = new QSpinBox();
		QString name = "%1";
		sb->setRange(0, 255);
		sb->setSingleStep(1);
		sb -> setObjectName(name.arg(i));
    	spinBox << sb;
	}
	spinBox[6] -> setRange(0, 50);
	spinBox[7] -> setRange(0, 50);
	spinBox[8] -> setRange(-10, 10);
	spinBox[9] -> setRange(0, 100);
	spinBox[10] -> setRange(-100, 100);
	spinBox[11] -> setRange(8000, 20000);
	spinBox[11] ->setSingleStep(100);
	spinBox[12] -> setRange(100, 800);
	for(auto i = 0; i < 13; ++i) spinBox[i] -> setValue(sbVal[i]);
	debugFrame = new QComboBox();
	debugFrame -> addItems(commands);
	debugFrame2 = new QComboBox();
	debugFrame2 -> addItems(commands);
	debugFrame2 -> setCurrentIndex(3);
	trackType = new QComboBox();
	trackType -> addItems(thresList);
	preset_box = new QHBoxLayout();
	preset_box2 = new QHBoxLayout();
	hi_box = new QHBoxLayout();
	lo_box = new QHBoxLayout();
	//text -> resize (640, 1000);
	frame->setAlignment(Qt::AlignCenter);
	//note_layout -> addWidget(saveSend);
	cursor_details -> addWidget(cursorPos);
	cursor_details -> addWidget(roiColor);
	cursor_details -> addWidget(cursorColor);
	for( auto i = 0; i < 3; ++i ){
    	hi_box -> addWidget(spinBox[i]);
		lo_box -> addWidget(spinBox[3+i]);
		spinBox[6+i]->setVisible(false);
		preset_box -> addWidget(spinBox[6+i]);
		spinBox[9+i]->setVisible(false);
		preset_box2 -> addWidget(spinBox[9+i]);
	}
	spinBox[12]->setVisible(false);
	preset_box2 -> addWidget(spinBox[12]);
	header_box -> addWidget(debugFrame);
	header_box -> addWidget(trackType);
	left_layout -> addLayout(header_box);
	left_layout -> addWidget(frame);
	left_layout -> addLayout(cursor_details);
	left_layout -> addWidget(copyThres);
	left_layout -> addWidget(highLabel);
	left_layout -> addLayout(hi_box);
	left_layout -> addWidget(lowLabel);
	left_layout -> addLayout(lo_box);
	left_layout -> addWidget(presetLabel);
	presetLabel -> setVisible(false);
	left_layout -> addLayout(preset_box);
	left_layout -> addWidget(presetLabel2);
	presetLabel2 -> setVisible(false);
	left_layout -> addLayout(preset_box2);
	sub_layout -> addLayout(left_layout);
	note_layout -> addWidget(debugFrame2);
	debugFrame2 -> setVisible(false);
	note_layout -> addWidget(frame2);
	frame2 -> setVisible(false);
	note_layout -> addWidget(note -> editor);
	note -> editor -> setVisible(false);
	sub_layout -> addLayout(note_layout);
	main_layout -> addLayout(sub_layout);
	centralWidget -> setLayout(main_layout);
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
 	cursor = frame -> mapFromGlobal(QCursor::pos());
	if(cursor.x() >= 0 && cursor.y() >= 0 && cursor.x() < 640 && cursor.y() < 480){
		emit curText(QStringLiteral("(%1, %2)").arg(cursor.x()).arg(cursor.y()), 
					 QStringLiteral("(%1, %2, %3)").arg(videoPi->clrValue.at<Vec3b>(cursor.y(), cursor.x())[0]).arg(videoPi->clrValue.at<Vec3b>(cursor.y(), cursor.x())[1]).arg(videoPi->clrValue.at<Vec3b>(cursor.y(), cursor.x())[2]));
	}
  	return false;
}

void MainWindow::updateCursor(QString pos, QString color)
{
	cursorPos -> setText(pos);
	cursorColor -> setText(color);
}

void MainWindow::make(){
	if(!(videoPi -> stopped)){
		videoPi -> server_ptr -> sendCommands(0, 0);
		videoPi->stopVideo();
	}
	ssh->make = true;
}

MainWindow::MainWindow(int arguments, QWidget *parent): QMainWindow(parent)
{
	initUI(arguments);
	initThreads(arguments);	
	if (arguments == 1) ssh -> sThread -> start();
	usleep(100000);
	readConfig();
	initActions();
	initMenu();
	initLayout();
	initSignal(arguments);
	//setMouseTracking(true);[0][0]
	installEventFilter(this);
	videoPi -> mode = arguments;
	videoPi -> vThread -> start();
}


MainWindow::~MainWindow()
{	
	if (videoPi -> mode == 1){
		cout << "destroy\n";
		if(!(videoPi -> stopped)) videoPi -> server_ptr -> sendCommands(0, 0);
		//pauseCam();
		//usleep(500000);
		ssh -> stopCamera();
		ssh -> sThread->quit();
    	ssh -> sThread->wait();
   		ssh_channel_close(ssh -> channel);
   		ssh_channel_free(ssh -> channel);
   		ssh_free(ssh -> session);
		videoPi->stopVideo();
    	videoPi->vThread->quit();
    	videoPi->vThread->wait();
	}
}

void MainWindow::mousePressEvent(QMouseEvent *e)
{
    point1 = frame -> mapFromGlobal(QCursor::pos());
	videoPi -> drawRect = true;
}

void MainWindow::mouseMoveEvent(QMouseEvent *e)
{
	point2 = frame -> mapFromGlobal(QCursor::pos());
	if((point1.x() != point2.x()) && (point1.y() != point2.y()) && point1.x() >= 0 && point1.x() <= 639 && point1.y() >= 0 && point1.y() <= 479 && point2.x() >= 0 && point2.x() <= 639 && point2.y() >= 0 && point2.y() <= 479){
		videoPi -> drawRect = true;
		videoPi -> point1 = point1;
		videoPi -> point2 = point2;
	}
}

void MainWindow::mouseReleaseEvent(QMouseEvent *e)
{
    point2 = frame -> mapFromGlobal(QCursor::pos());
	if((point1.x() != point2.x()) && (point1.y() != point2.y()) && point1.x() >= 0 && point1.x() <= 639 && point1.y() >= 0 && point1.y() <= 479 && point2.x() >= 0 && point2.x() <= 639 && point2.y() >= 0 && point2.y() <= 479){
		Mat roi;
		if(point1.x() < point2.x()) roi = videoPi -> clrValue(Rect(Point(point1.x(), point1.y()), Point(point2.x(), point2.y())));
		else roi = videoPi -> clrValue(Rect(Point(point2.x(), point2.y()), Point(point1.x(), point1.y())));
		Mat chn[3];
		split(roi, chn);
    	Mat maxR;
		for(int i = 0; i < 3; ++i){
			reduce(chn[i], maxR, 0, REDUCE_MAX);	
			reduce(maxR, maxR, 1, REDUCE_MAX);
			threshold[i] = maxR.at<uchar>(0);
			reduce(chn[i], maxR, 0, REDUCE_MIN);	
			reduce(maxR, maxR, 1, REDUCE_MIN);
			threshold[i+3] = maxR.at<uchar>(0);
		}
		emit roiText(QStringLiteral("ROI: (%1,%2)-(%3,%4) Thres: (%5,%6,%7),(%8,%9,%10)").arg(point1.x()).arg(point1.y()).arg(point2.x()).arg(point2.y()).arg(threshold[0]).arg(threshold[1]).arg(threshold[2]).arg(threshold[3]).arg(threshold[4]).arg(threshold[5]));
		videoPi -> drawRect = true;
		videoPi -> point1 = point1;
		videoPi -> point2 = point2;
	}
	else videoPi -> drawRect = false;
}

void MainWindow::updateRoi(QString roi)
{
	roiColor -> setText(roi);
}

void MainWindow::readConfig()
{
    char *filePath = static_cast<char*>(malloc(strlen(QCoreApplication::applicationDirPath().toLocal8Bit().constData()) + strlen(config_txt.c_str()) + 1));
	strcpy(filePath, QCoreApplication::applicationDirPath().toLocal8Bit().constData());
    strcat(filePath, config_txt.c_str());
	ifstream cFile (filePath);
	//cout << filePath << endl;
    if (cFile.is_open())
    {
        string line;
		while(getline(cFile, line)){
            line.erase(remove_if(line.begin(), line.end(), [](unsigned char x){return isspace(x);}),
                                 line.end());
            if(line[0] == '#' || line.empty())
                continue;
            auto delimiterPos = line.find("=");
    		string name = line.substr(0, delimiterPos);
            string value = line.substr(delimiterPos + 1);
			if(name == "1-hi")	allThres[0] = stoi(value);
			else if(name == "2-hi")	allThres[1] = stoi(value);
			else if(name == "3-hi") allThres[2] = stoi(value);
			else if(name == "1-lo") allThres[3] = stoi(value);
			else if(name == "2-lo") allThres[4] = stoi(value);
			else if(name == "3-lo") allThres[5] = stoi(value);
			else if(name == "b1-hi") allThres[6] = stoi(value);
			else if(name == "b2-hi") allThres[7] = stoi(value);
			else if(name == "b3-hi") allThres[8] = stoi(value);
			else if(name == "b1-lo") allThres[9] = stoi(value);
			else if(name == "b2-lo") allThres[10] = stoi(value);
			else if(name == "b3-lo") allThres[11] = stoi(value);
			else if(name == "y1-hi") allThres[12] = stoi(value);
			else if(name == "y2-hi") allThres[13] = stoi(value);
			else if(name == "y3-hi") allThres[14] = stoi(value);
			else if(name == "y1-lo") allThres[15] = stoi(value);
			else if(name == "y2-lo") allThres[16] = stoi(value);
			else if(name == "y3-lo") allThres[17] = stoi(value);
			else if(name == "f1-hi") allThres[18] = stoi(value);
			else if(name == "f2-hi") allThres[19] = stoi(value);
			else if(name == "f3-hi") allThres[20] = stoi(value);
			else if(name == "f1-lo") allThres[21] = stoi(value);
			else if(name == "f2-lo") allThres[22] = stoi(value);
			else if(name == "f3-lo") allThres[23] = stoi(value);
			else if(name == "awb_r") sbVal[6] = stoi(value);
			else if(name == "awb_b") sbVal[7] = stoi(value);
			else if(name == "exposure") sbVal[8] = stoi(value);
			else if(name == "brightness") sbVal[9] = stoi(value);
			else if(name == "saturation") sbVal[10] = stoi(value);
			else if(name == "ss") sbVal[11] = stoi(value);
			else if(name == "ISO") sbVal[12] = stoi(value);
         }
		 for(int i=0; i<6; ++i) sbVal[i] = allThres[i];
	}
    else 
	{
    	std::cerr << "Couldn't open config file for reading.\n";
    }
}

void MainWindow::saveConfig()
{
    char *filePath = static_cast<char*>(malloc(strlen(QCoreApplication::applicationDirPath().toLocal8Bit().constData()) + strlen(config_txt.c_str()) + 1));
    char *tempFilePath = static_cast<char*>(malloc(strlen(QCoreApplication::applicationDirPath().toLocal8Bit().constData()) + strlen(config_temp_txt.c_str()) + 1));
    strcpy(filePath, QCoreApplication::applicationDirPath().toLocal8Bit().constData());
    strcat(filePath, config_txt.c_str());
	ifstream cFile (filePath);
    strcpy(tempFilePath, QCoreApplication::applicationDirPath().toLocal8Bit().constData());
    strcat(tempFilePath, config_temp_txt.c_str());
	ofstream outFile(tempFilePath);
    if (cFile.is_open())
    {
        string line;
		while(getline(cFile, line)){
            if(line[0] != '#') line.erase(remove_if(line.begin(), line.end(), [](unsigned char x){return isspace(x);}), line.end());
			else{
				outFile << line;
				continue;
			}
            if(line.empty())
			{
				outFile << "\n";
				continue; 
			}
            auto delimiterPos = line.find("=");
    		string name = line.substr(0, delimiterPos);
            string value = line.substr(delimiterPos + 1);
			if(name == "1-hi")	outFile << name << " = " << allThres[0] << "\n";
			else if(name == "2-hi")	outFile << name << " = " << allThres[1] << "\n";
			else if(name == "3-hi") outFile << name << " = " << allThres[2] << "\n";
			else if(name == "1-lo") outFile << name << " = " << allThres[3] << "\n";
			else if(name == "2-lo") outFile << name << " = " << allThres[4] << "\n";
			else if(name == "3-lo") outFile << name << " = " << allThres[5] << "\n";
			else if(name == "b1-hi") outFile << name << " = " << allThres[6] << "\n";
			else if(name == "b2-hi") outFile << name << " = " << allThres[7] << "\n";
			else if(name == "b3-hi") outFile << name << " = " << allThres[8] << "\n";
			else if(name == "b1-lo") outFile << name << " = " << allThres[9] << "\n";
			else if(name == "b2-lo") outFile << name << " = " << allThres[10] << "\n";
			else if(name == "b3-lo") outFile << name << " = " << allThres[11] << "\n";
			else if(name == "y1-hi") outFile << name << " = " << allThres[12] << "\n";
			else if(name == "y2-hi") outFile << name << " = " << allThres[13] << "\n";
			else if(name == "y3-hi") outFile << name << " = " << allThres[14] << "\n";
			else if(name == "y1-lo") outFile << name << " = " << allThres[15] << "\n";
			else if(name == "y2-lo") outFile << name << " = " << allThres[16] << "\n";
			else if(name == "y3-lo") outFile << name << " = " << allThres[17] << "\n";
			else if(name == "f1-hi") outFile << name << " = " << allThres[18] << "\n";
			else if(name == "f2-hi") outFile << name << " = " << allThres[19] << "\n";
			else if(name == "f3-hi") outFile << name << " = " << allThres[20] << "\n";
			else if(name == "f1-lo") outFile << name << " = " << allThres[21] << "\n";
			else if(name == "f2-lo") outFile << name << " = " << allThres[22] << "\n";
			else if(name == "f3-lo") outFile << name << " = " << allThres[23] << "\n";
			else if(name == "awb_r") outFile << name << " = " << to_string(sbVal[6]) << "\n";
			else if(name == "awb_b") outFile << name << " = " << to_string(sbVal[7]) << "\n";
			else if(name == "exposure") outFile << name << " = " << to_string(sbVal[8]) << "\n";
			else if(name == "brightness") outFile << name << " = " << to_string(sbVal[9]) << "\n";
			else if(name == "saturation") outFile << name << " = " << to_string(sbVal[10]) << "\n";
			else if(name == "ss") outFile << name << " = " << to_string(sbVal[11]) << "\n";
			else if(name == "ISO") outFile << name << " = " << to_string(sbVal[12]) << "\n";
			else if(line.length() != 0) outFile << name << " = " << value << "\n";
        }
		outFile.close();
		rename(tempFilePath, filePath);
		remove(tempFilePath);
	}
}

void MainWindow::copyVal()
{
	for(auto i = 0; i < 6; i++) spinBox[i] -> setValue(threshold[i]);
}