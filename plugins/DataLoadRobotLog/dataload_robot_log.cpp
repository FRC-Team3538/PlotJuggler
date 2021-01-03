#include "dataload_robot_log.h"
#include <QDataStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <QSettings>
#include <QProgressDialog>
#include "proto/StatusFrame_generated.h"
#include <stdio.h>
#include "robot_log_parser.h"

DataLoadRobotLog::DataLoadRobotLog()
{
  _extensions.push_back("dat");
}

const std::vector<const char*>& DataLoadRobotLog::compatibleFileExtensions() const
{
  return _extensions;
}

bool DataLoadRobotLog::readDataFromFile(FileLoadInfo* info, PlotDataMapRef& plot_data)
{
  bool use_provided_configuration = false;

  if (info->plugin_config.hasChildNodes())
  {
    use_provided_configuration = true;
    xmlLoadState(info->plugin_config.firstChildElement());
  }

  const int TIME_INDEX_NOT_DEFINED = -2;

  int time_index = TIME_INDEX_NOT_DEFINED;

  QFile file(info->filename);
  file.open(QFile::ReadOnly);
  QDataStream inB(&file);
  inB.setByteOrder(QDataStream::BigEndian);

  Parser parser(plot_data);

  //-----------------

  while (!inB.atEnd())
  {
    quint32 len;

    inB >> len;
    len = _byteswap_ulong(len);
    char frame_bytes[len];

    inB.readRawData(frame_bytes, len);
    auto status_frame = rj::GetStatusFrameHolder(frame_bytes);

    if (parser.plot_frame(status_frame) != 0) {
      QString message = "Received a status frame with an unrecognized type\n"
          "Maybe you need to upgrade this plugin?";
      QMessageBox::warning(0, tr("Warning"), message);
    }
  }
  qDebug() << "Done!";
  file.close();

  return true;
}

DataLoadRobotLog::~DataLoadRobotLog()
{
}

bool DataLoadRobotLog::xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const
{
  QDomElement elem = doc.createElement("default");
  elem.setAttribute("time_axis", _default_time_axis.c_str());

  parent_element.appendChild(elem);
  return true;
}

bool DataLoadRobotLog::xmlLoadState(const QDomElement& parent_element)
{
  QDomElement elem = parent_element.firstChildElement("default");
  if (!elem.isNull())
  {
    if (elem.hasAttribute("time_axis"))
    {
      _default_time_axis = elem.attribute("time_axis").toStdString();
      return true;
    }
  }
  return false;
}
