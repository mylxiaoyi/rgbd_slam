#ifndef PROPERTIES_WINDOW_H
#define PROPERTIES_WINDOW_H

#include <vector>
#include <string>

#include <QDialog>
#include "ui_base_properties_widget.h"

namespace g2o
{
  class G2oQGLViewer;
}

class PropertiesWidget : public QDialog, public Ui::BasePropertiesWidget
{
  Q_OBJECT
  public:
    PropertiesWidget(QWidget * parent = 0, Qt::WindowFlags f = 0);
    ~PropertiesWidget();

    void setViewer(g2o::G2oQGLViewer* viewer);

  public slots:
    void on_btnApply_clicked();
    void on_btnOK_clicked();

  protected:
    g2o::G2oQGLViewer* _viewer;
    std::vector<std::string> _propNames;

    void updateDisplayedProperties();
    void applyProperties();
};

#endif
