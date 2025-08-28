/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QTreeView>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionOpen;
    QAction *actionFront;
    QAction *actionBack;
    QAction *actionTop;
    QAction *actionBottom;
    QAction *actionLeft;
    QAction *actionRight;
    QAction *actionIsometric;
    QAction *actionZoom_In;
    QAction *actionZoom_Out;
    QAction *actionZoom_All;
    QAction *actionZoom_Window;
    QAction *actionShade;
    QAction *actionMesh;
    QAction *actionNode;
    QAction *actionSave;
    QAction *actionSelectNode;
    QAction *actionSelectFace;
    QAction *actionShifttoOrigin;
    QAction *actionProfile;
    QAction *actionFaceNormal;
    QAction *actionNodeNormal;
    QAction *actionSelectEdge;
    QAction *actionGenerate;
    QAction *actionTest_1;
    QAction *actionSelectFix;
    QAction *actionSelectHandle;
    QAction *actionSaveSelection;
    QAction *actionReadSelection;
    QAction *actionSelectChamber;
    QAction *actionExport_to_Abaqus_model;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QToolBar *navigationToolBar;
    QStatusBar *statusBar;
    QToolBar *selectionToolBar;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_MANY_3DP_CNC_CAM;
    QFrame *line;
    QFrame *line_4;
    QTabWidget *tabWidget;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_6;
    QComboBox *comboBox_modelChoice3D;
    QLabel *label_18;
    QComboBox *comboBox_hmpType;
    QHBoxLayout *horizontalLayout_20;
    QLabel *label_19;
    QSpinBox *spinBox_checkWindow;
    QCheckBox *checkBox_globalSearch;
    QLabel *label_21;
    QSpinBox *spinBox_toolLength;
    QHBoxLayout *horizontalLayout_22;
    QCheckBox *checkBox_preProcess;
    QCheckBox *checkBox_optAE3D;
    QPushButton *pushButton_accEro3D;
    QPushButton *pushButton_accEroBatch;
    QSpacerItem *verticalSpacer_7;
    QHBoxLayout *horizontalLayout_18;
    QCheckBox *checkBox_forShowing;
    QPushButton *pushButton_inputHMP3D;
    QPushButton *pushButton_getField;
    QPushButton *pushButton_outputPath3D;
    QWidget *tab;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_input3DMatrix;
    QPushButton *pushButton_preProcess;
    QHBoxLayout *horizontalLayout_21;
    QSpinBox *spinBox_reduncheckWIndow;
    QPushButton *pushButton_redunCheck;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_5;
    QComboBox *comboBox_numOfCores;
    QTreeView *treeView;
    QPushButton *pushButton_clearAll;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuView;
    QMenu *menuSelect;
    QToolBar *toolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1343, 1109);
        MainWindow->setMinimumSize(QSize(0, 0));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        MainWindow->setFont(font);
        MainWindow->setMouseTracking(true);
        MainWindow->setFocusPolicy(Qt::StrongFocus);
        MainWindow->setAcceptDrops(true);
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/resource/Open Folder.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpen->setIcon(icon);
        actionFront = new QAction(MainWindow);
        actionFront->setObjectName(QString::fromUtf8("actionFront"));
        actionFront->setCheckable(false);
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/resource/Front View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFront->setIcon(icon1);
        actionBack = new QAction(MainWindow);
        actionBack->setObjectName(QString::fromUtf8("actionBack"));
        actionBack->setCheckable(false);
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/resource/Back View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionBack->setIcon(icon2);
        actionTop = new QAction(MainWindow);
        actionTop->setObjectName(QString::fromUtf8("actionTop"));
        actionTop->setCheckable(false);
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/resource/Top View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionTop->setIcon(icon3);
        actionBottom = new QAction(MainWindow);
        actionBottom->setObjectName(QString::fromUtf8("actionBottom"));
        actionBottom->setCheckable(false);
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/resource/Bottom View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionBottom->setIcon(icon4);
        actionLeft = new QAction(MainWindow);
        actionLeft->setObjectName(QString::fromUtf8("actionLeft"));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/resource/Left View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionLeft->setIcon(icon5);
        actionRight = new QAction(MainWindow);
        actionRight->setObjectName(QString::fromUtf8("actionRight"));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/resource/Right View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionRight->setIcon(icon6);
        actionIsometric = new QAction(MainWindow);
        actionIsometric->setObjectName(QString::fromUtf8("actionIsometric"));
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/resource/Isometric View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionIsometric->setIcon(icon7);
        actionZoom_In = new QAction(MainWindow);
        actionZoom_In->setObjectName(QString::fromUtf8("actionZoom_In"));
        QIcon icon8;
        icon8.addFile(QString::fromUtf8(":/resource/Zoom In.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_In->setIcon(icon8);
        actionZoom_Out = new QAction(MainWindow);
        actionZoom_Out->setObjectName(QString::fromUtf8("actionZoom_Out"));
        QIcon icon9;
        icon9.addFile(QString::fromUtf8(":/resource/Zoom Out.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_Out->setIcon(icon9);
        actionZoom_All = new QAction(MainWindow);
        actionZoom_All->setObjectName(QString::fromUtf8("actionZoom_All"));
        QIcon icon10;
        icon10.addFile(QString::fromUtf8(":/resource/Zoom All.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_All->setIcon(icon10);
        actionZoom_Window = new QAction(MainWindow);
        actionZoom_Window->setObjectName(QString::fromUtf8("actionZoom_Window"));
        QIcon icon11;
        icon11.addFile(QString::fromUtf8(":/resource/Zoom Window.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_Window->setIcon(icon11);
        actionShade = new QAction(MainWindow);
        actionShade->setObjectName(QString::fromUtf8("actionShade"));
        actionShade->setCheckable(true);
        QIcon icon12;
        icon12.addFile(QString::fromUtf8(":/resource/Shade.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionShade->setIcon(icon12);
        actionMesh = new QAction(MainWindow);
        actionMesh->setObjectName(QString::fromUtf8("actionMesh"));
        actionMesh->setCheckable(true);
        QIcon icon13;
        icon13.addFile(QString::fromUtf8(":/resource/Mesh.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionMesh->setIcon(icon13);
        actionNode = new QAction(MainWindow);
        actionNode->setObjectName(QString::fromUtf8("actionNode"));
        actionNode->setCheckable(true);
        QIcon icon14;
        icon14.addFile(QString::fromUtf8(":/resource/Node.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNode->setIcon(icon14);
        actionSave = new QAction(MainWindow);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        QIcon icon15;
        icon15.addFile(QString::fromUtf8(":/resource/Save as.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSave->setIcon(icon15);
        actionSelectNode = new QAction(MainWindow);
        actionSelectNode->setObjectName(QString::fromUtf8("actionSelectNode"));
        QIcon icon16;
        icon16.addFile(QString::fromUtf8(":/resource/selectNode.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectNode->setIcon(icon16);
        actionSelectFace = new QAction(MainWindow);
        actionSelectFace->setObjectName(QString::fromUtf8("actionSelectFace"));
        QIcon icon17;
        icon17.addFile(QString::fromUtf8(":/resource/selectFace.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectFace->setIcon(icon17);
        actionShifttoOrigin = new QAction(MainWindow);
        actionShifttoOrigin->setObjectName(QString::fromUtf8("actionShifttoOrigin"));
        actionProfile = new QAction(MainWindow);
        actionProfile->setObjectName(QString::fromUtf8("actionProfile"));
        actionProfile->setCheckable(true);
        QIcon icon18;
        icon18.addFile(QString::fromUtf8(":/resource/Profile.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionProfile->setIcon(icon18);
        actionFaceNormal = new QAction(MainWindow);
        actionFaceNormal->setObjectName(QString::fromUtf8("actionFaceNormal"));
        actionFaceNormal->setCheckable(true);
        QIcon icon19;
        icon19.addFile(QString::fromUtf8(":/resource/FaceNormal.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFaceNormal->setIcon(icon19);
        actionNodeNormal = new QAction(MainWindow);
        actionNodeNormal->setObjectName(QString::fromUtf8("actionNodeNormal"));
        actionNodeNormal->setCheckable(true);
        QIcon icon20;
        icon20.addFile(QString::fromUtf8(":/resource/NodeNormal.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNodeNormal->setIcon(icon20);
        actionSelectEdge = new QAction(MainWindow);
        actionSelectEdge->setObjectName(QString::fromUtf8("actionSelectEdge"));
        QIcon icon21;
        icon21.addFile(QString::fromUtf8(":/resource/selectEdge.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectEdge->setIcon(icon21);
        actionGenerate = new QAction(MainWindow);
        actionGenerate->setObjectName(QString::fromUtf8("actionGenerate"));
        actionTest_1 = new QAction(MainWindow);
        actionTest_1->setObjectName(QString::fromUtf8("actionTest_1"));
        actionSelectFix = new QAction(MainWindow);
        actionSelectFix->setObjectName(QString::fromUtf8("actionSelectFix"));
        QIcon icon22;
        icon22.addFile(QString::fromUtf8(":/resource/selectFix.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectFix->setIcon(icon22);
        actionSelectHandle = new QAction(MainWindow);
        actionSelectHandle->setObjectName(QString::fromUtf8("actionSelectHandle"));
        QIcon icon23;
        icon23.addFile(QString::fromUtf8(":/resource/selectHandle.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectHandle->setIcon(icon23);
        actionSaveSelection = new QAction(MainWindow);
        actionSaveSelection->setObjectName(QString::fromUtf8("actionSaveSelection"));
        QIcon icon24;
        icon24.addFile(QString::fromUtf8(":/resource/SaveSelection.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSaveSelection->setIcon(icon24);
        actionReadSelection = new QAction(MainWindow);
        actionReadSelection->setObjectName(QString::fromUtf8("actionReadSelection"));
        QIcon icon25;
        icon25.addFile(QString::fromUtf8(":/resource/InputSelection.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionReadSelection->setIcon(icon25);
        actionSelectChamber = new QAction(MainWindow);
        actionSelectChamber->setObjectName(QString::fromUtf8("actionSelectChamber"));
        actionExport_to_Abaqus_model = new QAction(MainWindow);
        actionExport_to_Abaqus_model->setObjectName(QString::fromUtf8("actionExport_to_Abaqus_model"));
        actionExport_to_Abaqus_model->setCheckable(false);
        QIcon icon26;
        icon26.addFile(QString::fromUtf8(":/resource/abaqus logo.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionExport_to_Abaqus_model->setIcon(icon26);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        centralWidget->setMouseTracking(true);
        centralWidget->setAcceptDrops(true);
        horizontalLayout = new QHBoxLayout(centralWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        MainWindow->setCentralWidget(centralWidget);
        navigationToolBar = new QToolBar(MainWindow);
        navigationToolBar->setObjectName(QString::fromUtf8("navigationToolBar"));
        navigationToolBar->setMovable(false);
        navigationToolBar->setIconSize(QSize(25, 25));
        navigationToolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, navigationToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);
        selectionToolBar = new QToolBar(MainWindow);
        selectionToolBar->setObjectName(QString::fromUtf8("selectionToolBar"));
        selectionToolBar->setMovable(false);
        selectionToolBar->setIconSize(QSize(25, 25));
        selectionToolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, selectionToolBar);
        dockWidget = new QDockWidget(MainWindow);
        dockWidget->setObjectName(QString::fromUtf8("dockWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(dockWidget->sizePolicy().hasHeightForWidth());
        dockWidget->setSizePolicy(sizePolicy);
        dockWidget->setMinimumSize(QSize(800, 778));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        dockWidgetContents->setLayoutDirection(Qt::LeftToRight);
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label_MANY_3DP_CNC_CAM = new QLabel(dockWidgetContents);
        label_MANY_3DP_CNC_CAM->setObjectName(QString::fromUtf8("label_MANY_3DP_CNC_CAM"));
        QFont font1;
        font1.setPointSize(10);
        label_MANY_3DP_CNC_CAM->setFont(font1);

        verticalLayout_2->addWidget(label_MANY_3DP_CNC_CAM);

        line = new QFrame(dockWidgetContents);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line);

        line_4 = new QFrame(dockWidgetContents);
        line_4->setObjectName(QString::fromUtf8("line_4"));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_4);

        tabWidget = new QTabWidget(dockWidgetContents);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        verticalLayout_3 = new QVBoxLayout(tab_2);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        label_6 = new QLabel(tab_2);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        horizontalLayout_10->addWidget(label_6);

        comboBox_modelChoice3D = new QComboBox(tab_2);
        comboBox_modelChoice3D->addItem(QString());
        comboBox_modelChoice3D->addItem(QString());
        comboBox_modelChoice3D->addItem(QString());
        comboBox_modelChoice3D->setObjectName(QString::fromUtf8("comboBox_modelChoice3D"));
        comboBox_modelChoice3D->setEditable(true);

        horizontalLayout_10->addWidget(comboBox_modelChoice3D);

        label_18 = new QLabel(tab_2);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        horizontalLayout_10->addWidget(label_18);

        comboBox_hmpType = new QComboBox(tab_2);
        comboBox_hmpType->addItem(QString());
        comboBox_hmpType->addItem(QString());
        comboBox_hmpType->setObjectName(QString::fromUtf8("comboBox_hmpType"));
        comboBox_hmpType->setEditable(true);

        horizontalLayout_10->addWidget(comboBox_hmpType);


        verticalLayout_3->addLayout(horizontalLayout_10);

        horizontalLayout_20 = new QHBoxLayout();
        horizontalLayout_20->setSpacing(6);
        horizontalLayout_20->setObjectName(QString::fromUtf8("horizontalLayout_20"));
        label_19 = new QLabel(tab_2);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        horizontalLayout_20->addWidget(label_19);

        spinBox_checkWindow = new QSpinBox(tab_2);
        spinBox_checkWindow->setObjectName(QString::fromUtf8("spinBox_checkWindow"));
        spinBox_checkWindow->setMinimum(1);
        spinBox_checkWindow->setMaximum(999);
        spinBox_checkWindow->setValue(10);

        horizontalLayout_20->addWidget(spinBox_checkWindow);

        checkBox_globalSearch = new QCheckBox(tab_2);
        checkBox_globalSearch->setObjectName(QString::fromUtf8("checkBox_globalSearch"));

        horizontalLayout_20->addWidget(checkBox_globalSearch);

        label_21 = new QLabel(tab_2);
        label_21->setObjectName(QString::fromUtf8("label_21"));

        horizontalLayout_20->addWidget(label_21);

        spinBox_toolLength = new QSpinBox(tab_2);
        spinBox_toolLength->setObjectName(QString::fromUtf8("spinBox_toolLength"));
        spinBox_toolLength->setMinimum(2);
        spinBox_toolLength->setMaximum(200);
        spinBox_toolLength->setSingleStep(1);
        spinBox_toolLength->setValue(10);

        horizontalLayout_20->addWidget(spinBox_toolLength);


        verticalLayout_3->addLayout(horizontalLayout_20);

        horizontalLayout_22 = new QHBoxLayout();
        horizontalLayout_22->setSpacing(6);
        horizontalLayout_22->setObjectName(QString::fromUtf8("horizontalLayout_22"));
        checkBox_preProcess = new QCheckBox(tab_2);
        checkBox_preProcess->setObjectName(QString::fromUtf8("checkBox_preProcess"));
        checkBox_preProcess->setChecked(true);

        horizontalLayout_22->addWidget(checkBox_preProcess);

        checkBox_optAE3D = new QCheckBox(tab_2);
        checkBox_optAE3D->setObjectName(QString::fromUtf8("checkBox_optAE3D"));
        checkBox_optAE3D->setChecked(false);

        horizontalLayout_22->addWidget(checkBox_optAE3D);

        pushButton_accEro3D = new QPushButton(tab_2);
        pushButton_accEro3D->setObjectName(QString::fromUtf8("pushButton_accEro3D"));

        horizontalLayout_22->addWidget(pushButton_accEro3D);

        pushButton_accEroBatch = new QPushButton(tab_2);
        pushButton_accEroBatch->setObjectName(QString::fromUtf8("pushButton_accEroBatch"));

        horizontalLayout_22->addWidget(pushButton_accEroBatch);


        verticalLayout_3->addLayout(horizontalLayout_22);

        verticalSpacer_7 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_7);

        horizontalLayout_18 = new QHBoxLayout();
        horizontalLayout_18->setSpacing(6);
        horizontalLayout_18->setObjectName(QString::fromUtf8("horizontalLayout_18"));
        checkBox_forShowing = new QCheckBox(tab_2);
        checkBox_forShowing->setObjectName(QString::fromUtf8("checkBox_forShowing"));
        checkBox_forShowing->setChecked(true);

        horizontalLayout_18->addWidget(checkBox_forShowing);

        pushButton_inputHMP3D = new QPushButton(tab_2);
        pushButton_inputHMP3D->setObjectName(QString::fromUtf8("pushButton_inputHMP3D"));

        horizontalLayout_18->addWidget(pushButton_inputHMP3D);

        pushButton_getField = new QPushButton(tab_2);
        pushButton_getField->setObjectName(QString::fromUtf8("pushButton_getField"));

        horizontalLayout_18->addWidget(pushButton_getField);

        pushButton_outputPath3D = new QPushButton(tab_2);
        pushButton_outputPath3D->setObjectName(QString::fromUtf8("pushButton_outputPath3D"));

        horizontalLayout_18->addWidget(pushButton_outputPath3D);


        verticalLayout_3->addLayout(horizontalLayout_18);

        tabWidget->addTab(tab_2, QString());
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout = new QVBoxLayout(tab);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        pushButton_input3DMatrix = new QPushButton(tab);
        pushButton_input3DMatrix->setObjectName(QString::fromUtf8("pushButton_input3DMatrix"));

        verticalLayout->addWidget(pushButton_input3DMatrix);

        pushButton_preProcess = new QPushButton(tab);
        pushButton_preProcess->setObjectName(QString::fromUtf8("pushButton_preProcess"));

        verticalLayout->addWidget(pushButton_preProcess);

        horizontalLayout_21 = new QHBoxLayout();
        horizontalLayout_21->setSpacing(6);
        horizontalLayout_21->setObjectName(QString::fromUtf8("horizontalLayout_21"));
        spinBox_reduncheckWIndow = new QSpinBox(tab);
        spinBox_reduncheckWIndow->setObjectName(QString::fromUtf8("spinBox_reduncheckWIndow"));
        spinBox_reduncheckWIndow->setMinimum(1);
        spinBox_reduncheckWIndow->setMaximum(99999999);
        spinBox_reduncheckWIndow->setSingleStep(1);
        spinBox_reduncheckWIndow->setValue(10000);

        horizontalLayout_21->addWidget(spinBox_reduncheckWIndow);

        pushButton_redunCheck = new QPushButton(tab);
        pushButton_redunCheck->setObjectName(QString::fromUtf8("pushButton_redunCheck"));

        horizontalLayout_21->addWidget(pushButton_redunCheck);


        verticalLayout->addLayout(horizontalLayout_21);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        label_5 = new QLabel(tab);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_8->addWidget(label_5);

        comboBox_numOfCores = new QComboBox(tab);
        comboBox_numOfCores->addItem(QString());
        comboBox_numOfCores->addItem(QString());
        comboBox_numOfCores->addItem(QString());
        comboBox_numOfCores->setObjectName(QString::fromUtf8("comboBox_numOfCores"));
        comboBox_numOfCores->setEditable(true);

        horizontalLayout_8->addWidget(comboBox_numOfCores);


        verticalLayout->addLayout(horizontalLayout_8);

        tabWidget->addTab(tab, QString());

        verticalLayout_2->addWidget(tabWidget);

        treeView = new QTreeView(dockWidgetContents);
        treeView->setObjectName(QString::fromUtf8("treeView"));
        treeView->setEnabled(true);
        treeView->setMaximumSize(QSize(16777215, 20));
        treeView->setProperty("showDropIndicator", QVariant(true));
        treeView->setIndentation(5);
        treeView->header()->setVisible(false);

        verticalLayout_2->addWidget(treeView);

        pushButton_clearAll = new QPushButton(dockWidgetContents);
        pushButton_clearAll->setObjectName(QString::fromUtf8("pushButton_clearAll"));

        verticalLayout_2->addWidget(pushButton_clearAll);

        dockWidget->setWidget(dockWidgetContents);
        MainWindow->addDockWidget(Qt::RightDockWidgetArea, dockWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1343, 22));
        menuBar->setLayoutDirection(Qt::LeftToRight);
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuView = new QMenu(menuBar);
        menuView->setObjectName(QString::fromUtf8("menuView"));
        menuSelect = new QMenu(menuBar);
        menuSelect->setObjectName(QString::fromUtf8("menuSelect"));
        MainWindow->setMenuBar(menuBar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        toolBar->setMovable(false);
        toolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);

        navigationToolBar->addAction(actionFront);
        navigationToolBar->addAction(actionBack);
        navigationToolBar->addAction(actionTop);
        navigationToolBar->addAction(actionBottom);
        navigationToolBar->addAction(actionLeft);
        navigationToolBar->addAction(actionRight);
        navigationToolBar->addAction(actionIsometric);
        navigationToolBar->addSeparator();
        navigationToolBar->addAction(actionZoom_In);
        navigationToolBar->addAction(actionZoom_Out);
        navigationToolBar->addAction(actionZoom_All);
        navigationToolBar->addAction(actionZoom_Window);
        navigationToolBar->addSeparator();
        navigationToolBar->addAction(actionShade);
        navigationToolBar->addAction(actionMesh);
        navigationToolBar->addAction(actionNode);
        navigationToolBar->addAction(actionProfile);
        navigationToolBar->addAction(actionFaceNormal);
        navigationToolBar->addAction(actionNodeNormal);
        selectionToolBar->addAction(actionSaveSelection);
        selectionToolBar->addAction(actionReadSelection);
        selectionToolBar->addSeparator();
        selectionToolBar->addAction(actionSelectNode);
        selectionToolBar->addAction(actionSelectEdge);
        selectionToolBar->addAction(actionSelectFace);
        selectionToolBar->addAction(actionSelectFix);
        selectionToolBar->addAction(actionSelectHandle);
        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuView->menuAction());
        menuBar->addAction(menuSelect->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionSave);
        menuFile->addAction(actionSaveSelection);
        menuFile->addAction(actionReadSelection);
        menuView->addAction(actionFront);
        menuView->addAction(actionBack);
        menuView->addAction(actionTop);
        menuView->addAction(actionBottom);
        menuView->addAction(actionLeft);
        menuView->addAction(actionRight);
        menuView->addAction(actionIsometric);
        menuView->addSeparator();
        menuView->addAction(actionZoom_In);
        menuView->addAction(actionZoom_Out);
        menuView->addAction(actionZoom_All);
        menuView->addAction(actionZoom_Window);
        menuView->addSeparator();
        menuView->addAction(actionShade);
        menuView->addAction(actionMesh);
        menuView->addAction(actionNode);
        menuView->addAction(actionProfile);
        menuView->addSeparator();
        menuView->addAction(actionShifttoOrigin);
        menuSelect->addAction(actionSelectNode);
        menuSelect->addAction(actionSelectEdge);
        menuSelect->addAction(actionSelectFace);
        menuSelect->addSeparator();
        menuSelect->addAction(actionSelectFix);
        menuSelect->addAction(actionSelectHandle);
        menuSelect->addSeparator();
        toolBar->addAction(actionOpen);
        toolBar->addAction(actionSave);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        actionOpen->setText(QCoreApplication::translate("MainWindow", "Open", nullptr));
        actionFront->setText(QCoreApplication::translate("MainWindow", "Front", nullptr));
        actionBack->setText(QCoreApplication::translate("MainWindow", "Back", nullptr));
        actionTop->setText(QCoreApplication::translate("MainWindow", "Top", nullptr));
        actionBottom->setText(QCoreApplication::translate("MainWindow", "Bottom", nullptr));
        actionLeft->setText(QCoreApplication::translate("MainWindow", "Left", nullptr));
        actionRight->setText(QCoreApplication::translate("MainWindow", "Right", nullptr));
        actionIsometric->setText(QCoreApplication::translate("MainWindow", "Isometric", nullptr));
        actionZoom_In->setText(QCoreApplication::translate("MainWindow", "Zoom In", nullptr));
        actionZoom_Out->setText(QCoreApplication::translate("MainWindow", "Zoom Out", nullptr));
        actionZoom_All->setText(QCoreApplication::translate("MainWindow", "Zoom All", nullptr));
        actionZoom_Window->setText(QCoreApplication::translate("MainWindow", "Zoom Window", nullptr));
        actionShade->setText(QCoreApplication::translate("MainWindow", "Shade", nullptr));
        actionMesh->setText(QCoreApplication::translate("MainWindow", "Mesh", nullptr));
        actionNode->setText(QCoreApplication::translate("MainWindow", "Node", nullptr));
        actionSave->setText(QCoreApplication::translate("MainWindow", "Save", nullptr));
        actionSelectNode->setText(QCoreApplication::translate("MainWindow", "Node", nullptr));
        actionSelectFace->setText(QCoreApplication::translate("MainWindow", "Face", nullptr));
        actionShifttoOrigin->setText(QCoreApplication::translate("MainWindow", "Shift to Origin", nullptr));
        actionProfile->setText(QCoreApplication::translate("MainWindow", "Profile", nullptr));
        actionFaceNormal->setText(QCoreApplication::translate("MainWindow", "FaceNormal", nullptr));
        actionNodeNormal->setText(QCoreApplication::translate("MainWindow", "NodeNormal", nullptr));
        actionSelectEdge->setText(QCoreApplication::translate("MainWindow", "Edge", nullptr));
        actionGenerate->setText(QCoreApplication::translate("MainWindow", "Generate", nullptr));
        actionTest_1->setText(QCoreApplication::translate("MainWindow", "Test_1", nullptr));
        actionSelectFix->setText(QCoreApplication::translate("MainWindow", "Fix", nullptr));
        actionSelectHandle->setText(QCoreApplication::translate("MainWindow", "Handle & Rigid", nullptr));
        actionSaveSelection->setText(QCoreApplication::translate("MainWindow", "Save selection", nullptr));
        actionReadSelection->setText(QCoreApplication::translate("MainWindow", "Read selection", nullptr));
        actionSelectChamber->setText(QCoreApplication::translate("MainWindow", "Select Chamber (SORO)", nullptr));
        actionExport_to_Abaqus_model->setText(QCoreApplication::translate("MainWindow", "Export to Abaqus model", nullptr));
        navigationToolBar->setWindowTitle(QCoreApplication::translate("MainWindow", "navigationToolBar", nullptr));
        selectionToolBar->setWindowTitle(QCoreApplication::translate("MainWindow", "selectionToolBar", nullptr));
        label_MANY_3DP_CNC_CAM->setText(QCoreApplication::translate("MainWindow", "HybridManu", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "model: ", nullptr));
        comboBox_modelChoice3D->setItemText(0, QCoreApplication::translate("MainWindow", "fertility100", nullptr));
        comboBox_modelChoice3D->setItemText(1, QCoreApplication::translate("MainWindow", "bracket100", nullptr));
        comboBox_modelChoice3D->setItemText(2, QCoreApplication::translate("MainWindow", "TPMS50", nullptr));

        label_18->setText(QCoreApplication::translate("MainWindow", "inputHMPType:", nullptr));
        comboBox_hmpType->setItemText(0, QCoreApplication::translate("MainWindow", "AccEro", nullptr));
        comboBox_hmpType->setItemText(1, QCoreApplication::translate("MainWindow", "tool10_local10_AccEro", nullptr));

        label_19->setText(QCoreApplication::translate("MainWindow", "checkWindow", nullptr));
        checkBox_globalSearch->setText(QCoreApplication::translate("MainWindow", "globalSearch", nullptr));
        label_21->setText(QCoreApplication::translate("MainWindow", "toolLength", nullptr));
        checkBox_preProcess->setText(QCoreApplication::translate("MainWindow", "preProcess", nullptr));
        checkBox_optAE3D->setText(QCoreApplication::translate("MainWindow", "optInAccEro", nullptr));
        pushButton_accEro3D->setText(QCoreApplication::translate("MainWindow", "accEro", nullptr));
        pushButton_accEroBatch->setText(QCoreApplication::translate("MainWindow", "accEroBatch", nullptr));
        checkBox_forShowing->setText(QCoreApplication::translate("MainWindow", "For Showing", nullptr));
        pushButton_inputHMP3D->setText(QCoreApplication::translate("MainWindow", "input HMP", nullptr));
        pushButton_getField->setText(QCoreApplication::translate("MainWindow", "getField", nullptr));
        pushButton_outputPath3D->setText(QCoreApplication::translate("MainWindow", "outputPath", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("MainWindow", "Main Page", nullptr));
        pushButton_input3DMatrix->setText(QCoreApplication::translate("MainWindow", "input", nullptr));
        pushButton_preProcess->setText(QCoreApplication::translate("MainWindow", "preProces", nullptr));
        pushButton_redunCheck->setText(QCoreApplication::translate("MainWindow", "redundancyCheck", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "numOfCores", nullptr));
        comboBox_numOfCores->setItemText(0, QCoreApplication::translate("MainWindow", "15", nullptr));
        comboBox_numOfCores->setItemText(1, QCoreApplication::translate("MainWindow", "20", nullptr));
        comboBox_numOfCores->setItemText(2, QCoreApplication::translate("MainWindow", "30", nullptr));

        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("MainWindow", "Other Tools", nullptr));
        pushButton_clearAll->setText(QCoreApplication::translate("MainWindow", "Clear All", nullptr));
        menuFile->setTitle(QCoreApplication::translate("MainWindow", "File", nullptr));
        menuView->setTitle(QCoreApplication::translate("MainWindow", "View", nullptr));
        menuSelect->setTitle(QCoreApplication::translate("MainWindow", "Select", nullptr));
        toolBar->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
