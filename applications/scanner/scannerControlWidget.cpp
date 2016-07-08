/**
 * \file scannerControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "scannerControlWidget.h"
#include "ui_scannerControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


ScannerControlWidget::ScannerControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::ScannerControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->pathEdit, SIGNAL(textChanged(QString)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->fileTemplateEdit, SIGNAL(textChanged(QString)), this, SIGNAL(paramsChanged()));
}

ScannerControlWidget::~ScannerControlWidget()
{

    delete mUi;
}

void ScannerControlWidget::loadParamWidget(WidgetLoader &loader)
{
    Scanner *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void ScannerControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    Scanner *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}

 /* Composite fields are NOT supported so far */
void ScannerControlWidget::getParameters(Scanner& params) const
{

    params.setPath             (mUi->pathEdit->text().toStdString());
    params.setFileTemplate     (mUi->fileTemplateEdit->text().toStdString());

}

Scanner *ScannerControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    Scanner *result = new Scanner(
          mUi->pathEdit->text().toStdString()
        , mUi->fileTemplateEdit->text().toStdString()
    );
    return result;
}

void ScannerControlWidget::setParameters(const Scanner &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->pathEdit->setText(input.path().c_str());
    mUi->fileTemplateEdit->setText(input.fileTemplate().c_str());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void ScannerControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    Scanner *inputCasted = static_cast<Scanner *>(input);
    setParameters(*inputCasted);
}