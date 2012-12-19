#include "properties_widget.h"

#include "g2o_qglviewer.h"
#include "g2o/stuff/property.h"

#include <QLineEdit>

#include <iostream>
#include <cassert>

#ifdef __GNUC__
  #include <cxxabi.h>
#endif

using namespace std;

using namespace g2o;

/**
 * demangle the name of a type, depends on the used compiler
 */
std::string demangleName(const std::string& fullPropName)
{
#ifdef __GNUC__
  // find :: and extract the mangled class name from the whole string
  string mangledName;
  string propName;
  string::size_type found = fullPropName.rfind("::");
  if (found != string::npos) {
    mangledName = fullPropName.substr(0, found);
    propName    = fullPropName.substr(found);
  } else {
    mangledName = propName;
  }

  int status;
  char* s = abi::__cxa_demangle(mangledName.c_str(), 0, 0, &status);
  if (status != 0) {
    free(s);
    return fullPropName;
  } else {
    std::string demangled(s);
    free(s);
    return demangled + propName;
  }
#else
  // TODO for other compilers
  return fullPropName;
#endif
}

PropertiesWidget::PropertiesWidget(QWidget * parent, Qt::WindowFlags f) :
  QDialog(parent, f)
{
  setupUi(this);
}

PropertiesWidget::~PropertiesWidget()
{
}

void PropertiesWidget::updateDisplayedProperties()
{
  tableWidget->clear();
  _propNames.clear();

  assert(_viewer);
  PropertyMap* properties = _viewer->parameters();
  if (! properties)
    return;

  tableWidget->setRowCount(properties->size());
  tableWidget->setColumnCount(2);

  QStringList horizontalHeaders;
  horizontalHeaders.append("Name");
  horizontalHeaders.append("Value");
  tableWidget->setHorizontalHeaderLabels(horizontalHeaders);

  tableWidget->verticalHeader()->hide();

  int r = 0;
  for (PropertyMap::iterator it = properties->begin(); it != properties->end(); ++it, ++r) {

    QTableWidgetItem* textItem = new QTableWidgetItem;
    textItem->setText(QString::fromStdString(demangleName(it->first)));
    textItem->setFlags(textItem->flags() & ~Qt::ItemIsEditable);
    tableWidget->setItem(r, 0, textItem);
    _propNames.push_back(it->first);

    if (dynamic_cast<Property<bool>*>(it->second)) {
      Property<bool>* prop = static_cast<Property<bool>*>(it->second);
      QTableWidgetItem* checkItem = new QTableWidgetItem;
      checkItem->setText("enabled");
      checkItem->setFlags(checkItem->flags() | Qt::ItemIsUserCheckable);
      if (prop->value())
        checkItem->setCheckState(Qt::Checked);
      else
        checkItem->setCheckState(Qt::Unchecked);
      tableWidget->setItem(r, 1, checkItem);
    } else {
      QLineEdit* editor = new QLineEdit(tableWidget);
      editor->setText(QString::fromStdString(it->second->toString()));
      if (dynamic_cast<Property<int>*>(it->second)) {
        editor->setValidator(new QIntValidator(editor));
      }
      else if (dynamic_cast<Property<float>*>(it->second)) {
        editor->setValidator(new QDoubleValidator(editor));
      }
      else if (dynamic_cast<Property<double>*>(it->second)) {
        editor->setValidator(new QDoubleValidator(editor));
      }
      tableWidget->setCellWidget(r, 1, editor);
    }

  }
  tableWidget->resizeColumnToContents(0);
}

void PropertiesWidget::applyProperties()
{
  assert(tableWidget->rowCount() == (int) _propNames.size());
  PropertyMap* properties = _viewer->parameters();
  for (int r = 0; r < tableWidget->rowCount(); ++r) {
    const std::string& propName = _propNames[r];
    BaseProperty* baseProp = properties->getProperty<BaseProperty>(propName);
    if (! baseProp)
      continue;

    if (dynamic_cast<Property<bool>*>(baseProp)) {
      Property<bool>* prop = static_cast<Property<bool>*>(baseProp);
      QTableWidgetItem* checkItem = tableWidget->item(r, 1);
      prop->setValue(checkItem->checkState() == Qt::Checked);
    } else {
      QLineEdit* editor = dynamic_cast<QLineEdit*>(tableWidget->cellWidget(r, 1));
      bool status = baseProp->fromString(editor->text().toStdString());
      if (! status) {
        cerr << "Warning: unable to set property " << baseProp->name() << endl;
      }
    }
  }

  // draw with the new properties
  _viewer->setUpdateDisplay(true);
  _viewer->updateGL();
}

void PropertiesWidget::on_btnApply_clicked()
{
  applyProperties();
}

void PropertiesWidget::on_btnOK_clicked()
{
  applyProperties();
  close();
}

void PropertiesWidget::setViewer(g2o::G2oQGLViewer* viewer)
{
  _viewer = viewer;
  updateDisplayedProperties();
}
