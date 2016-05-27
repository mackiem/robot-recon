/********************************************************************************
** Form generated from reading UI file 'filteredstructlight.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FILTEREDSTRUCTLIGHT_H
#define UI_FILTEREDSTRUCTLIGHT_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_FilteredStructLightClass
{
public:
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QWidget *centralWidget;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *FilteredStructLightClass)
    {
        if (FilteredStructLightClass->objectName().isEmpty())
            FilteredStructLightClass->setObjectName(QStringLiteral("FilteredStructLightClass"));
        FilteredStructLightClass->resize(600, 400);
        menuBar = new QMenuBar(FilteredStructLightClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        FilteredStructLightClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(FilteredStructLightClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        FilteredStructLightClass->addToolBar(mainToolBar);
        centralWidget = new QWidget(FilteredStructLightClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        FilteredStructLightClass->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(FilteredStructLightClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        FilteredStructLightClass->setStatusBar(statusBar);

        retranslateUi(FilteredStructLightClass);

        QMetaObject::connectSlotsByName(FilteredStructLightClass);
    } // setupUi

    void retranslateUi(QMainWindow *FilteredStructLightClass)
    {
        FilteredStructLightClass->setWindowTitle(QApplication::translate("FilteredStructLightClass", "FilteredStructLight", 0));
    } // retranslateUi

};

namespace Ui {
    class FilteredStructLightClass: public Ui_FilteredStructLightClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FILTEREDSTRUCTLIGHT_H
