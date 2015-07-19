/********************************************************************************
** Form generated from reading UI file 'mrm_adviser_widget.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef MRM_ADVISER_WIDGET_H
#define MRM_ADVISER_WIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MrmAdviserWidget
{
public:
    QGridLayout *gridLayout;
    QLabel *adviser_text_label;

    void setupUi(QWidget *MrmAdviserWidget)
    {
        if (MrmAdviserWidget->objectName().isEmpty())
            MrmAdviserWidget->setObjectName(QString::fromUtf8("MrmAdviserWidget"));
        MrmAdviserWidget->resize(1047, 77);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MrmAdviserWidget->sizePolicy().hasHeightForWidth());
        MrmAdviserWidget->setSizePolicy(sizePolicy);
        gridLayout = new QGridLayout(MrmAdviserWidget);
        gridLayout->setSpacing(0);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 7, 70, 0);
        adviser_text_label = new QLabel(MrmAdviserWidget);
        adviser_text_label->setObjectName(QString::fromUtf8("adviser_text_label"));
        QFont font;
        font.setFamily(QString::fromUtf8("Hadasim CLM"));
        font.setPointSize(18);
        font.setBold(true);
        font.setWeight(75);
        adviser_text_label->setFont(font);
        adviser_text_label->setStyleSheet(QString::fromUtf8("background:\"transparent\";"));
        adviser_text_label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(adviser_text_label, 0, 0, 1, 1);


        retranslateUi(MrmAdviserWidget);

        QMetaObject::connectSlotsByName(MrmAdviserWidget);
    } // setupUi

    void retranslateUi(QWidget *MrmAdviserWidget)
    {
        MrmAdviserWidget->setWindowTitle(QApplication::translate("MrmAdviserWidget", "MRM:: Event Log", 0, QApplication::UnicodeUTF8));
        adviser_text_label->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class MrmAdviserWidget: public Ui_MrmAdviserWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // MRM_ADVISER_WIDGET_H
