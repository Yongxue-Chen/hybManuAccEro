#include "stdafx.h"

#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QFileDialog>
#include <QtDebug>
#include <QDesktopWidget>
#include <QCoreApplication>
#include <QMimeData>
#include <QTreeView>
#include <QThread>
#include <QTimer>
#include <QDateTime>
#include <QMessageBox>
#include <QScreen>
#include <QStyleFactory>
#include <fstream>

#include "../GLKLib/GLKCameraTool.h"
#include "../GLKLib/InteractiveTool.h"
#include "../GLKLib/GLKMatrixLib.h"
#include "../GLKLib/GLKGeometry.h"
#include "../QMeshLib/QMeshPatch.h"
#include "../QMeshLib/QMeshTetra.h"
#include "../QMeshLib/QMeshFace.h"
#include "../QMeshLib/QMeshEdge.h"
#include "../QMeshLib/QMeshNode.h"

#include "alphanum.hpp"
#include <dirent.h>

#include "fileIO.h"
#include <chrono>

//#undef slots
//#include <torch/torch.h>
//#include <torch/script.h>
//#define slots Q_SLOTS

//#include <Python.h>
#include <cstdlib>
//#include "AccEro3D.h"
#include "AccEro3DNew.h"
#include "postProcess.h"

#include <random>
//#include <filesystem>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	QApplication::setStyle(QStyleFactory::create("Fusion"));

    signalMapper = new QSignalMapper(this);
    addToolBar(ui->toolBar);
    addToolBar(ui->navigationToolBar);
    addToolBar(ui->selectionToolBar);

    createTreeView();
    createActions();

    pGLK = new GLKLib();
    ui->horizontalLayout->addWidget(pGLK);
    ui->horizontalLayout->setMargin(0);
    pGLK->setFocus();

    pGLK->clear_tools();
    pGLK->set_tool(new GLKCameraTool(pGLK,ORBITPAN));
	
	//connect timer with timer function
	//connect(&Gcode_timer, SIGNAL(timeout()), this, SLOT(doTimerGcodeMoving()));

    hmParaObj = new hmPara();
    modelObj3D = new model3D();
}

MainWindow::~MainWindow()
{
    delete hmParaObj;
    delete modelObj3D;
    delete ui;
}

void MainWindow::createActions()
{
    // file IO
    connect(ui->actionOpen, SIGNAL(triggered(bool)), this, SLOT(open()));
    connect(ui->actionSave, SIGNAL(triggered(bool)), this, SLOT(save()));
	connect(ui->actionSaveSelection, SIGNAL(triggered(bool)), this, SLOT(saveSelection()));
	connect(ui->actionReadSelection, SIGNAL(triggered(bool)), this, SLOT(readSelection()));

    // navigation
    connect(ui->actionFront, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionBack, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionTop, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionBottom, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionLeft, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionRight, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionIsometric, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_In, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_Out, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_All, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_Window, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    signalMapper->setMapping (ui->actionFront, 0);
    signalMapper->setMapping (ui->actionBack, 1);
    signalMapper->setMapping (ui->actionTop, 2);
    signalMapper->setMapping (ui->actionBottom, 3);
    signalMapper->setMapping (ui->actionLeft, 4);
    signalMapper->setMapping (ui->actionRight, 5);
    signalMapper->setMapping (ui->actionIsometric, 6);
    signalMapper->setMapping (ui->actionZoom_In, 7);
    signalMapper->setMapping (ui->actionZoom_Out, 8);
    signalMapper->setMapping (ui->actionZoom_All, 9);
    signalMapper->setMapping (ui->actionZoom_Window, 10);

    // view
    connect(ui->actionShade, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionMesh, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionNode, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionProfile, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionFaceNormal, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionNodeNormal, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    signalMapper->setMapping (ui->actionShade, 20);
    signalMapper->setMapping (ui->actionMesh, 21);
    signalMapper->setMapping (ui->actionNode, 22);
    signalMapper->setMapping (ui->actionProfile, 23);
    signalMapper->setMapping (ui->actionFaceNormal, 24);
    signalMapper->setMapping (ui->actionNodeNormal, 25);
    ui->actionShade->setChecked(true);

    connect(ui->actionShifttoOrigin, SIGNAL(triggered(bool)), this, SLOT(shiftToOrigin()));

    // select
    connect(ui->actionSelectNode, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionSelectEdge, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionSelectFace, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionSelectFix, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionSelectHandle, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));

	signalMapper->setMapping (ui->actionSelectNode, 30);
    signalMapper->setMapping (ui->actionSelectEdge, 31);
    signalMapper->setMapping (ui->actionSelectFace, 32);
	signalMapper->setMapping(ui->actionSelectFix, 33);
	signalMapper->setMapping(ui->actionSelectHandle, 34);


    connect (signalMapper, SIGNAL(mapped(int)), this, SLOT(signalNavigation(int)));

	//Button
	//connect(ui->pushButton_ShowAllLayers, SIGNAL(released()), this, SLOT(viewAllWaypointLayers()));
	//connect(ui->spinBox_ShowLayerIndex, SIGNAL(valueChanged(int)), this, SLOT(changeWaypointDisplay()));
	connect(ui->pushButton_accEro3D, SIGNAL(released()), this, SLOT(getAccEroSolution3D()));
	//connect(ui->pushButton_getTraining3D, SIGNAL(released()), this, SLOT(getTrainingDatas3D()));
	connect(ui->pushButton_inputHMP3D, SIGNAL(released()), this, SLOT(inputHMP3D()));
	connect(ui->pushButton_outputPath3D, SIGNAL(released()), this, SLOT(outputPath3D()));
    connect(ui->pushButton_redunCheck, SIGNAL(released()), this, SLOT(checkRedundancy()));
	connect(ui->pushButton_preProcess, SIGNAL(released()), this, SLOT(preProcess()));
	connect(ui->pushButton_accEroBatch, SIGNAL(released()), this, SLOT(accEroBatch()));
}

void MainWindow::open()
{
    QString filenameStr = QFileDialog::getOpenFileName(this, tr("Open File,"), "..", tr(""));
    QFileInfo fileInfo(filenameStr);
    QString fileSuffix = fileInfo.suffix();
    QByteArray filenameArray = filenameStr.toLatin1();
    char *filename = filenameArray.data();

    // set polygen name
    std::string strFilename(filename);
    std::size_t foundStart = strFilename.find_last_of("/");
    std::size_t foundEnd = strFilename.find_last_of(".");
    std::string modelName;
    modelName = strFilename.substr(0,foundEnd);
    modelName = modelName.substr(foundStart+1);
    
    if (QString::compare(fileSuffix,"obj") == 0){
        PolygenMesh *polygenMesh = new PolygenMesh(UNDEFINED);
        polygenMesh->ImportOBJFile(filename,modelName);
        polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
        pGLK->AddDisplayObj(polygenMesh,true);
        polygenMeshList.AddTail(polygenMesh);
    }

	else if (QString::compare(fileSuffix, "tet") == 0) {
		PolygenMesh *polygenMesh = new PolygenMesh(TET);
		std::cout << filename << std::endl;
		std::cout << modelName << std::endl;
		polygenMesh->ImportTETFile(filename, modelName);
		polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
		pGLK->AddDisplayObj(polygenMesh, true);
		polygenMeshList.AddTail(polygenMesh);
	}

    updateTree();

    shiftToOrigin();
    pGLK->refresh(true);
}

void MainWindow::save()
{
	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	if (!polygenMesh)
		return;
	QString filenameStr = QFileDialog::getSaveFileName(this, tr("OBJ File Export,"), "..", tr("OBJ(*.obj)"));
	QFileInfo fileInfo(filenameStr);
	QString fileSuffix = fileInfo.suffix();

	if (QString::compare(fileSuffix, "obj") == 0) {
		QFile exportFile(filenameStr);
		if (exportFile.open(QFile::WriteOnly | QFile::Truncate)) {
			QTextStream out(&exportFile);
			for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
				QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);
				for (GLKPOSITION posNode = patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
					QMeshNode *node = (QMeshNode*)patch->GetNodeList().GetNext(posNode);
					double xx, yy, zz;
					node->GetCoord3D(xx, yy, zz);
					float r, g, b;
					node->GetColor(r, g, b);
					out << "v " << xx << " " << yy << " " << zz << " " << node->value1 << endl;
				}
				for (GLKPOSITION posFace = patch->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
					QMeshFace *face = (QMeshFace*)patch->GetFaceList().GetNext(posFace);
					out << "f " << face->GetNodeRecordPtr(0)->GetIndexNo() << " " << face->GetNodeRecordPtr(1)->GetIndexNo() << " " << face->GetNodeRecordPtr(2)->GetIndexNo() << endl;
				}
			}
		}
		exportFile.close();
	}
}

void MainWindow::saveSelection()
{
	//printf("%s exported\n", Model->ModelName);

	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	std::string filename = polygenMesh->getModelName();
	const char * c = filename.c_str();
	char *cstr = new char[filename.length() + 1];
	strcpy(cstr, filename.c_str());

	const char * split = ".";
	char* p = strtok(cstr, split);

	char output_filename[256];
	strcpy(output_filename, "..\\selection_file\\");
	strcat(output_filename, cstr);
	char filetype[64];
	strcpy(filetype, ".txt");
	strcat(output_filename, filetype);

	ofstream nodeSelection(output_filename);
	if (!nodeSelection)
		cerr << "Sorry!We were unable to build the file NodeSelect!\n";
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *CheckNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		nodeSelection << CheckNode->GetIndexNo() << ":";
		//for the selection of fixing part
		if (CheckNode->isFixed == true) nodeSelection << "1:";
		else nodeSelection << "0:";
		//for the selection of hard part
		if (CheckNode->isHandle == true) nodeSelection << "1:" << endl;
		else nodeSelection << "0:" << endl;
	}

	nodeSelection.close();
	printf("Finish output selection \n");
}

void MainWindow::readSelection()
{
	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	std::string filename = polygenMesh->getModelName();
	const char * c = filename.c_str();

	char *cstr = new char[filename.length() + 1];
	strcpy(cstr, filename.c_str());

	const char * split = ".";
	char* p = strtok(cstr, split);

	char input_filename[256];
	strcpy(input_filename, "..\\selection_file\\");
	strcat(input_filename, cstr);
	char filetype[64];
	strcpy(filetype, ".txt");
	strcat(input_filename, filetype);

	ifstream nodeSelect(input_filename);
	if (!nodeSelect)
		cerr << "Sorry!We were unable to open the file!\n";
	vector<int> NodeIndex(patch->GetNodeNumber()), checkNodeFixed(patch->GetNodeNumber()), checkNodeHandle(patch->GetNodeNumber());
	//string line;
	int LineIndex1 = 0;
	string sss;
	while (getline(nodeSelect, sss)){
		const char * c = sss.c_str();
		sscanf(c, "%d:%d:%d", &NodeIndex[LineIndex1], &checkNodeFixed[LineIndex1], &checkNodeHandle[LineIndex1]);
		LineIndex1++;
	}

	nodeSelect.close();
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *CheckNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		if (checkNodeFixed[CheckNode->GetIndexNo() - 1] == 1) CheckNode->isFixed = true;
		if (checkNodeHandle[CheckNode->GetIndexNo() - 1] == 1) CheckNode->isHandle = true;
	}

	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshFace* Face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		if (Face->GetNodeRecordPtr(0)->isHandle == true &&
			Face->GetNodeRecordPtr(1)->isHandle == true &&
			Face->GetNodeRecordPtr(2)->isHandle == true)
			Face->isHandleDraw = true;
		else Face->isHandleDraw = false;

		if (Face->GetNodeRecordPtr(0)->isFixed == true &&
			Face->GetNodeRecordPtr(1)->isFixed == true &&
			Face->GetNodeRecordPtr(2)->isFixed == true)
			Face->isFixedDraw = true;
		else Face->isFixedDraw = false;
	}
	printf("Finish input selection \n");
	pGLK->refresh(true);

}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
	//QMouseEvent *e = (QMouseEvent*)event;
	//QPoint pos = e->pos();
	//cout << "Mouse position updated" << endl;
	//double wx, wy, wz;
	//pGLK->screen_to_wcl(100.0, 100.0, wx, wy, wz);
	//ui->CorrdinateMouse->setText(QString("X = %1").arg(wx));

	//QString text;
	//text = QString("%1 X %2").arg(event->pos().x()).arg(event->pos().y());
	///** Update the info text */
	//ui->statusBar->showMessage(text);
}

void MainWindow::signalNavigation(int flag)
{
    if (flag <= 10)
        pGLK->setNavigation(flag);
    if (flag >=20 && flag <=25){
        pGLK->setViewModel(flag-20);
        switch (flag) {
        case 20:
            ui->actionShade->setChecked(pGLK->getViewModel(0));
            break;
        case 21:
            ui->actionMesh->setChecked(pGLK->getViewModel(1));
            break;
        case 22:
            ui->actionNode->setChecked(pGLK->getViewModel(2));
            break;
        case 23:
            ui->actionProfile->setChecked(pGLK->getViewModel(3));
            break;
        case 24:
            ui->actionFaceNormal->setChecked(pGLK->getViewModel(4));
            break;
        case 25:
            ui->actionNodeNormal->setChecked(pGLK->getViewModel(5));
            break;
        }
    }
  //  if (flag==30 || flag==31 || flag==32 || flag == 33 || flag == 34){
  //      InteractiveTool *tool;
  //      switch (flag) {
  //      case 30:
  //          tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), NODE, ui->boxDeselect->isChecked());
  //          break;
  //      case 31:
  //          tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), EDGE, ui->boxDeselect->isChecked());
  //          break;
  //      case 32:
  //          tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), FACE, ui->boxDeselect->isChecked());
  //          break;
		//case 33:
		//	tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), FIX, ui->boxDeselect->isChecked());
		//	break;
		//case 34:
		//	tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), NHANDLE, ui->boxDeselect->isChecked());
		//	break;
  //      }
  //      pGLK->set_tool(tool);
  //  }
}

void MainWindow::shiftToOrigin()
{
    
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasUrls())
        event->acceptProposedAction();
}

void MainWindow::dropEvent(QDropEvent *event)
{
    QString filenameStr;
    foreach (const QUrl &url, event->mimeData()->urls())
        filenameStr = url.toLocalFile();
    QByteArray filenameArray = filenameStr.toLatin1();
    char *filename = filenameArray.data();

    PolygenMesh *polygenMesh = new PolygenMesh(UNDEFINED);

    // set polygen name
    std::string strFilename(filename);
    std::size_t foundStart = strFilename.find_last_of("/");
    std::size_t foundEnd = strFilename.find_last_of(".");
    std::string modelName;
    modelName = strFilename.substr(0,foundEnd);
    modelName = modelName.substr(foundStart+1);
    int i = 0;
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr;){
        PolygenMesh *polygen = (PolygenMesh*)polygenMeshList.GetNext(pos);
        std::string name = (polygen->getModelName()).substr(0,(polygen->getModelName()).find(' '));
        if (name == modelName)
            i++;
    }
    if (i > 0)
        modelName += " "+std::to_string(i);

	QFileInfo fileInfo(filenameStr);
	QString fileSuffix = fileInfo.suffix();
	if (QString::compare(fileSuffix, "obj") == 0) {
		polygenMesh->ImportOBJFile(filename, modelName);
	}
	else if (QString::compare(fileSuffix, "tet") == 0) {
		polygenMesh->ImportTETFile(filename, modelName);
        polygenMesh->meshType = TET;
	}
	polygenMesh->m_bVertexNormalShading = false;	
    polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
    pGLK->AddDisplayObj(polygenMesh,true);
    polygenMeshList.AddTail(polygenMesh);
    
    updateTree();
}

void MainWindow::createTreeView()
{
    treeModel = new QStandardItemModel();
    ui->treeView->setModel(treeModel);
    ui->treeView->setHeaderHidden(true);
    ui->treeView->setContextMenuPolicy(Qt::CustomContextMenu);
    ui->treeView->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->treeView->expandAll();
}

void MainWindow::updateTree()
{
    treeModel->clear();
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr;){
        PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        QString modelName = QString::fromStdString(polygenMesh->getModelName());
        QStandardItem *modelListItem = new QStandardItem(modelName);
        modelListItem->setCheckable(true);
        modelListItem->setCheckState(Qt::Checked);
        treeModel->appendRow(modelListItem);
    }
	pGLK->refresh(true);
}

PolygenMesh *MainWindow::getSelectedPolygenMesh()
{
    if (!treeModel->hasChildren())
        return nullptr;
    QModelIndex index = ui->treeView->currentIndex();
    QString selectedModelName = index.data(Qt::DisplayRole).toString();
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr;){
        PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        QString modelName = QString::fromStdString(polygenMesh->getModelName());
        if (QString::compare(selectedModelName,modelName) == 0)
            return polygenMesh;
    }
    return nullptr;
}

void MainWindow::on_pushButton_clearAll_clicked()
{
    int i = 0;
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr; i++){
        PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();
        if (i<2)
            continue;
        for (GLKPOSITION pos2=patch->GetFaceList().GetHeadPosition(); pos2!=nullptr;){
            QMeshFace *face = (QMeshFace*)patch->GetFaceList().GetNext(pos2);
            face->m_nIdentifiedPatchIndex = 0;
        }
    }
    pGLK->refresh(true);
}

void MainWindow::on_treeView_clicked(const QModelIndex &index)
{
    ui->treeView->currentIndex();
    QStandardItem *modelListItem = treeModel->itemFromIndex(index);
    ui->treeView->setCurrentIndex(index);
    PolygenMesh *polygenMesh = getSelectedPolygenMesh();
    if (modelListItem->checkState() == Qt::Checked)
        polygenMesh->bShow = true;
    else
        polygenMesh->bShow = false;
    pGLK->refresh(true);
}


//yongxue

void MainWindow::input3DModelMatrix() {
    fileIO* IO_operator = new fileIO();

    string modelName = ui->comboBox_modelChoice3D->currentText().toStdString();
    Eigen::Vector3i size;
    Eigen::VectorXi modelValue;
    Eigen::VectorXi outerStatus;
    //IO_operator->read3DMatrix("../DataSet/inputModelMatrix/" + modelName + ".txt", size, modelValue, outerStatus);

	// Copy the model file from inputVoxModel to outputTime3D
    std::string sourceFileName = modelName + ".txt";
    std::string sourceFolderPath = "../DataSet/inputVoxModel/";
    QString qSourceFilePath = QString::fromStdString(sourceFolderPath + sourceFileName);
    QString qDestinationFolderPath = "../DataSet/outputTime3D/";
    QDir destinationDir(qDestinationFolderPath);
    QString qDestinationFileName = QString::fromStdString(sourceFileName);
    QString qDestinationFilePath = destinationDir.filePath(qDestinationFileName);
    if (!QFile::exists(qSourceFilePath)) {
        qDebug() << "Error: Source file does not exist:" << qSourceFilePath;
        return; // Or handle error appropriately
    }
    if (!destinationDir.exists()) {
        if (destinationDir.mkpath(".")) {
            //qDebug() << "Created destination directory:" << qDestinationFolderPath;
        }
        else {
            qDebug() << "Error: Could not create destination directory:" << qDestinationFolderPath;
            return;
        }
    }
    if (QFile::exists(qDestinationFilePath)) {
        if (QFile::remove(qDestinationFilePath)) {
            //qDebug() << "Removed existing destination file:" << qDestinationFilePath;
        }
        else {
            qDebug() << "Error: Could not remove existing destination file:" << qDestinationFilePath;
        }
    }
    if (QFile::copy(qSourceFilePath, qDestinationFilePath)) {
       /* qDebug() << "File copied successfully from" << qSourceFilePath << "to" << qDestinationFilePath;*/
    }
    else {
        QFile sourceFile(qSourceFilePath); // Needed to get errorString
        qDebug() << "Error: File copy failed." << "Source:" << qSourceFilePath << "Dest:" << qDestinationFilePath << "Reason:" << sourceFile.errorString();
    }


    IO_operator->read3DMatrix("../DataSet/outputTime3D/" + modelName + ".txt", size, modelValue, outerStatus);
    Eigen::Vector3i coreRegionSize = size;
    Eigen::Vector3i shellRegionSize = outerStatus;
    Eigen::VectorXi model = modelObj3D->augmentModel(modelValue, size, Eigen::Vector3i(0, 0, size(2) - 1), coreRegionSize, shellRegionSize);
    size << coreRegionSize(0) + 2 * shellRegionSize(0) + 2,
        coreRegionSize(1) + 2 * shellRegionSize(1) + 2,
        coreRegionSize(2) + shellRegionSize(2) + 1;

    delete modelObj3D;
	modelObj3D = new model3D();
    modelObj3D->init(size, model, shellRegionSize);

    delete IO_operator;

    std::cout << "get 3D target model!\n";
    std::cout << "size: " << size(0) << " " << size(1) << " " << size(2) << std::endl;
}

void MainWindow::preProcess() {
    this->input3DModelMatrix();

    string modelName = ui->comboBox_modelChoice3D->currentText().toStdString();

    int checkWindow = ui->spinBox_checkWindow->value();
    bool globalSearch = ui->checkBox_globalSearch->isChecked();
    int SMLength = ui->spinBox_toolLength->value();
    AccEro3DNew* AccEro_operator = new AccEro3DNew(modelObj3D, checkWindow, globalSearch, SMLength);

    Eigen::VectorXi resultModel;
    std::vector<int> growedList;
    double time;
    AccEro_operator->preProcess(resultModel, growedList, time);
    delete AccEro_operator;

    std::vector<int> coreGrowList;
	coreGrowList.reserve(growedList.size());
    int nxCore = modelObj3D->nx - 2;
	int nyCore = modelObj3D->ny - 2;
	int nzCore = modelObj3D->nz - 1;
    for (int i = 0; i < growedList.size(); i++) {
		Eigen::Vector3i posCore = modelObj3D->indexToCoordinate(growedList[i]);
		posCore(0) = posCore(0) - 1;
		posCore(1) = posCore(1) - 1;
		posCore(2) = posCore(2) - 1;
		if (posCore(0) < 0 || posCore(0) > modelObj3D->nx - 3 ||
			posCore(1) < 0 || posCore(1) > modelObj3D->ny - 3 ||
			posCore(2) < 0 || posCore(2) > modelObj3D->nz - 2) {
			std::cout << "error: posCore out of range!" << std::endl;
		}
		int idx = posCore(2) * nxCore * nyCore + posCore(1) * nxCore + posCore(0);
		coreGrowList.push_back(idx);
    }

	fileIO* IO_operator = new fileIO();
    Eigen::Vector3i size;
    Eigen::VectorXi modelValue;
    Eigen::VectorXi outerStatus;
    IO_operator->read3DMatrix("../DataSet/outputTime3D/" + modelName + ".txt", size, modelValue, outerStatus);
	if (size(0) != nxCore || size(1) != nyCore || size(2) != nzCore) {
		std::cout << "error: size not match!" << std::endl;
	}
    for (int i = 0; i < coreGrowList.size(); i++) {
        if (modelValue(coreGrowList[i]) == 1) {
            std::cout << "error: modelValue not match!" << std::endl;
        }
		modelValue(coreGrowList[i]) = 1;
    }
    

    std::string lengthStr = std::to_string(SMLength);
    std::string modelName2 = modelName + "_afterPre" + lengthStr;
    IO_operator->writeModel("../DataSet/inputVoxModel/" + modelName2 + ".txt", size, modelValue);
    IO_operator->writeModel("../DataSet/outputTime3D/" + modelName2 + ".txt", size, modelValue);

	QString modelNameQ = QString::fromStdString(modelName2);
	ui->comboBox_modelChoice3D->setCurrentText(modelNameQ);

	Eigen::VectorXi coreGrowListEigen = Eigen::Map<Eigen::VectorXi>(coreGrowList.data(), coreGrowList.size());
	
    IO_operator->writeVector(coreGrowListEigen, "../DataSet/outputTime3D/" + modelName + "_tool" + lengthStr + "_coreGrowList.txt");

	delete IO_operator;

	std::cout << "get 3D target model!\n";
	//std::cout << "size: " << size(0) << " " << size(1) << " " << size(2) << std::endl;

	modelObj3D->preProcessTime = time;
}


void MainWindow::getAccEroSolution3D() {
    int SMLength = ui->spinBox_toolLength->value();
    string modelName = ui->comboBox_modelChoice3D->currentText().toStdString();

	if (ui->checkBox_preProcess->isChecked()) {
		string lengthStr = std::to_string(SMLength);
        std::string modelNameAfterPre = modelName + "_afterPre" + lengthStr;
        std::string modelPath = "../DataSet/outputTime3D/" + modelNameAfterPre + ".txt";
		std::string growListPath = "../DataSet/outputTime3D/" + modelName + "_tool" + lengthStr + "_coreGrowList.txt";
        if (std::filesystem::exists(modelPath) && std::filesystem::exists(growListPath)) {
			QString modelNameQ = QString::fromStdString(modelNameAfterPre);
            ui->comboBox_modelChoice3D->setCurrentText(modelNameQ);
        }
        else {
            this->preProcess();
        }
	}
    double preProcessTime = modelObj3D->preProcessTime;

	std::cout << "preProcessTime: " << preProcessTime << std::endl;

	this->input3DModelMatrix();

    modelName = ui->comboBox_modelChoice3D->currentText().toStdString();

	int checkWindow = ui->spinBox_checkWindow->value();
	bool globalSearch = ui->checkBox_globalSearch->isChecked();

    AccEro3DNew* AccEro_operator = new AccEro3DNew(modelObj3D, checkWindow, globalSearch, SMLength);

    Eigen::VectorXi brimModle = modelObj3D->onlyBrimValue;
	fileIO* IO_operator = new fileIO();
	IO_operator->writeVector(brimModle, "../DataSet/outputTime3D/" + modelName + "_brimModel.txt");

    //set cpu cores
	int numOfCores = ui->comboBox_numOfCores->currentText().toInt();
	omp_set_num_threads(numOfCores);

    bool optInAE = ui->checkBox_optAE3D->isChecked();
    Eigen::VectorXi resultModel;

	auto start = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd timeList;
    HMP_struct3D result = AccEro_operator->solveByAccEro3d(modelName, resultModel, timeList, optInAE, false, true);
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed_seconds = end - start;
	double solveTime = elapsed_seconds.count();

    int threshold;
    Eigen::MatrixXi Time = AccEro_operator->hmp2Matrix(result, threshold);
    Eigen::MatrixXd TimeDouble = Time.cast<double>();

    std::string modelNameOriginal = modelName;
	int preProcessNum = 0;
    if (ui->checkBox_preProcess->isChecked()) {
        Eigen::VectorXi preProcessList;
		modelNameOriginal = modelName.substr(0, modelName.find("_afterPre"));
		std::string lengthStr = std::to_string(SMLength);
        IO_operator->readVector("../DataSet/outputTime3D/" + modelNameOriginal + "_tool" + lengthStr + "_coreGrowList.txt", preProcessList);

        if (TimeDouble.cols() > 2) {
			std::cout << "error: Time.cols() > 2!" << std::endl;
            return;
        }

        if (TimeDouble.cols() == 1) {
			Eigen::MatrixXd TimeNew = Eigen::MatrixXd::Ones(TimeDouble.rows(), 2);
			TimeNew.col(0) = TimeDouble.col(0);
            TimeNew.col(1) = TimeNew.col(1) * (threshold + 1);
            TimeDouble = TimeNew;
        }

        double diffPre = 0.5 / preProcessList.size();

        for (int i = 0; i < preProcessList.size(); i++) {
            int idx = preProcessList(i);
            if (TimeDouble(idx, 1) < threshold) {
                std::cout << "error: Time(idx,1)<threshold!" << std::endl;
            }
            TimeDouble(idx, 1) = threshold - (i + 1) * diffPre;
        }

        std::cout << "\npreProcess size: " << preProcessList.size() << std::endl;
		preProcessNum = preProcessList.size();
    }
    std::cout << "preProcessTime: " << preProcessTime << std::endl;
    std::cout << std::endl;

    // change Time and threshold to double type
	double thresholdDouble = threshold;

	IO_operator->writeHMP3D(TimeDouble, thresholdDouble, "../DataSet/outputTime3D/" + modelName + "_tool" + std::to_string(SMLength) + "_local" + std::to_string(checkWindow) + "_AccEroHMP.txt");

    ui->comboBox_hmpType->setCurrentText(QString::fromStdString("tool" + std::to_string(SMLength) + "_local" + std::to_string(checkWindow) + "_AccEro"));


	delete AccEro_operator;

	std::cout << "get 3D AccEro solution!\n";

	int numOpt = (TimeDouble.array() < (thresholdDouble)).count();

	this->inputHMP3D();

    this->ui->comboBox_modelChoice3D->setCurrentText(QString::fromStdString(modelNameOriginal));

    //std::cout << "1" << std::endl;

	Eigen::Vector3i size;
	Eigen::VectorXi modelValue;
	Eigen::VectorXi outerStatus;
	IO_operator->read3DMatrix("../DataSet/outputTime3D/" + modelNameOriginal + ".txt", size, modelValue, outerStatus);

    //std::cout << "2" << std::endl;

    std::string logFile = "../DataSet/log/log.txt";
	std::ofstream infoStream(logFile, std::ios::app);
    if (infoStream.is_open()) {
        infoStream << "currentTime: " << QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString()
            << " modelName: " << modelName << " toolLength: " << SMLength << " checkWindow: " << checkWindow << " solveTime: " << solveTime
            << " preProcessTime: " << preProcessTime << " numOpt: " << numOpt << " preProcessNum: " << preProcessNum
            << " solid number: " << modelValue.sum() << " size: " << size(0) << " " << size(1) << " " << size(2) 
            << " check Stability Time: "<< timeList(0)
            << std::endl;
        std::cout << "logAdded" << std::endl;
    }
    else {
		std::cerr << "Unable to open file: " << logFile << std::endl;
    }

    delete IO_operator;
}

void MainWindow::accEroBatch() {
 //   Eigen::VectorXi toolLength;
	//toolLength.resize(6);
	//toolLength << 2, 5, 10, 30, 50, 100;

 //   for (int i = 0; i < toolLength.size(); i++) {
	//	ui->comboBox_modelChoice3D->setCurrentText(QString::fromStdString("tube100"));
 //       ui->spinBox_toolLength->setValue(toolLength(i));
	//	this->preProcess();
 //   }


    std::vector<std::string> modelNames;
    std::vector<Eigen::Vector3i> paras;

    std::ifstream infile("../DataSet/outputTime3D/testList.txt");
    if (!infile) {
		std::cerr << "Error opening file" << std::endl;
		return;
    }

	std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        std::string modelName;
        int a, b, c;
        if (!(iss >> modelName >> a >> b >> c)) {
            std::cerr << "Error reading line: " << line << std::endl;
            continue;
        }
        modelNames.push_back(modelName);
        Eigen::Vector3i para(a, b, c);
        paras.push_back(para);
    }
	infile.close();

	std::cout << "modelNames size: " << modelNames.size() << std::endl;

    for (int i = 0; i < modelNames.size(); i++) {
		ui->comboBox_modelChoice3D->setCurrentText(QString::fromStdString(modelNames[i]));
		ui->spinBox_toolLength->setValue(paras[i](0));
		ui->spinBox_checkWindow->setValue(paras[i](1));
        if (paras[i](2) == 1) {
			ui->checkBox_preProcess->setChecked(true);
        }
        else {
			ui->checkBox_preProcess->setChecked(false);
        }

        this->getAccEroSolution3D();

		std::cout << "get 3D AccEro solution for model: " << modelNames[i] << std::endl;

		// pause for 5 seconds
		std::this_thread::sleep_for(std::chrono::seconds(5));
    }

}

void MainWindow::checkRedundancy() {
	this->input3DModelMatrix();

    string modelName = ui->comboBox_modelChoice3D->currentText().toStdString();

    int checkWindow = ui->spinBox_checkWindow->value();
    bool globalSearch = ui->checkBox_globalSearch->isChecked();
    int SMLength = ui->spinBox_toolLength->value();

    AccEro3DNew* AccEro_operator = new AccEro3DNew(modelObj3D, checkWindow, globalSearch, SMLength);

    fileIO* IO_operator = new fileIO();

    //set cpu cores
    int numOfCores = ui->comboBox_numOfCores->currentText().toInt();
    omp_set_num_threads(numOfCores);

    int range = ui->spinBox_reduncheckWIndow->value();
    HMP_struct3D result = AccEro_operator->redundancyCheck(modelName, range);

    int threshold;
    Eigen::MatrixXi Time = AccEro_operator->hmp2Matrix(result, threshold);

    // change Time and threshold to double type
    Eigen::MatrixXd TimeDouble = Time.cast<double>();
    double thresholdDouble = threshold;

	std::string rangeString = std::to_string(range);
    IO_operator->writeHMP3D(TimeDouble, thresholdDouble, "../DataSet/outputTime3D/" + modelName + "_tool" + std::to_string(SMLength) + "_local" + std::to_string(checkWindow) + "_redundancy" + rangeString + "_AccEroHMP.txt");
    delete IO_operator;

    delete AccEro_operator;

    ui->comboBox_hmpType->setCurrentText(QString::fromStdString("tool" + std::to_string(SMLength) + "_local" + std::to_string(checkWindow) + "_redundancy" + rangeString + "_AccEro"));

    std::cout << "get 3D AccEro solution!\n";

	this->inputHMP3D();
}



void MainWindow::inputHMP3D() {
	fileIO* IO_operator = new fileIO();
    std::string modelName = ui->comboBox_modelChoice3D->currentText().toStdString();

	// input model
    Eigen::Vector3i size;
	Eigen::VectorXi modelValueA, modelValue;
	Eigen::VectorXi outerStatus;
    //IO_operator->read3DMatrix("../DataSet/tmp/" + modelName + "_board.txt", size, modelValueA, outerStatus);
    //std::cout << "num solid: " << modelValueA.sum() << std::endl;

    IO_operator->read3DMatrix("../DataSet/outputTime3D/" + modelName + ".txt", size, modelValue, outerStatus);
    delete modelObj3D;
    modelObj3D = new model3D();
    modelObj3D->init(size, modelValue, outerStatus, true);
    std::cout << "model size: " << size(0) << " " << size(1) << " " << size(2) << std::endl;
    std::cout << "num solid: " << modelValue.sum() << std::endl;

	// input HMP
    double tEnd;
    Eigen::MatrixXd HMP;
	std::string hmpType = ui->comboBox_hmpType->currentText().toStdString();

    IO_operator->readHMP3D("../DataSet/outputTime3D/" + modelName + "_" + hmpType + "HMP.txt", HMP, tEnd);

	HMP = HMP.array() + 1;
	tEnd = tEnd + 1;
    for (int i = 0; i < modelValueA.size(); i++) {
        if (modelValueA(1) != 0)
        {
            Eigen::VectorXd tmp = HMP.row(i);
            if (HMP(i, HMP.cols() - 1) > tEnd) {
                HMP(i, 0) = 0;
                HMP.row(i).rightCols(HMP.cols() - 1) = tmp.leftCols(HMP.cols() - 1);
            }
            else {
                std::cout << "Error" << std::endl;
            }
        }

    }

    modelObj3D->HMP = HMP;
	modelObj3D->tEnd = tEnd;

	std::cout << "numMP: " << (HMP.array() < tEnd).count() << std::endl;


	// the number of odd and even columns in HMP
	int SMCount = 0;
	int AMCount = 0;
	
	for (int i = 0; i < HMP.rows(); i++) {
		for (int j = 0; j < HMP.cols(); j++) {
			if (HMP(i, j) < tEnd) {
				if (j % 2 == 0) {
					AMCount++; 
				} else {
					SMCount++;
				}
			}
		}
	}
	
	std::cout << "AM count: " << AMCount << std::endl;
	std::cout << "SM count: " << SMCount << std::endl;

	delete IO_operator;

	std::cout << "get 3D HMP!\n";
}

void MainWindow::outputPath3D() {
    //Eigen::Vector2d zeroInCenterFrame(-4.5 * 1.6, -12 * 1.6);
    Eigen::Vector2d zeroInCenterFrame;
    zeroInCenterFrame << -1 * modelObj3D->nx / 2 * 1.2 - 0.5, -1 * modelObj3D->ny / 2 * 1.2;
	std::cout << "nx: " << modelObj3D->nx << " ny: " << modelObj3D->ny << std::endl;
    commandParas cmParas;

	postProcess* postPro_operator = new postProcess(modelObj3D, cmParas, zeroInCenterFrame);

    if (ui->checkBox_forShowing->isChecked()) {
		postPro_operator->justForShowing = true;
    }
    else {
		postPro_operator->justForShowing = false;
    }

	postPro_operator->tP2PathFile(modelObj3D->HMP, modelObj3D->tEnd, ui->comboBox_modelChoice3D->currentText().toStdString());

	delete postPro_operator;
	std::cout << "output path!" << std::endl;
}