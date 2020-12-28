#pragma once

#include <QObject>
#include <QtPlugin>
#include "PlotJuggler/messageparser_base.h"
#include "proto/StatusFrame_generated.h"
#include <QCheckBox>

class RobotLogParser : public PJ::MessageParser
{
public:
  RobotLogParser(const std::string& topic_name, PJ::PlotDataMapRef& data, bool use_msg_stamp):
    MessageParser(topic_name, data),
    _use_message_stamp(use_msg_stamp) {}

  bool parseMessage(const PJ::MessageRef msg, double timestamp) override;
  int plot_frame(const rj::StatusFrameHolder *status_frame);


private:
  bool _use_message_stamp;

  std::string get_id(const rj::CTREMotorStatusFrame* motor);

  std::string get_id(const rj::PDPStatusFrame* pdp);

  std::string get_id(const rj::PCMStatusFrame* pcm);

  int plot_frame(double time, const rj::CTREMotorStatusFrame* motor);

  int plot_frame(double time, const rj::PDPStatusFrame* pdp);

  int plot_frame(double time, const rj::PCMStatusFrame* pcm);

};

class RobotLogParserCreator: public PJ::MessageParserCreator
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.MessageParserCreator")
  Q_INTERFACES(PJ::MessageParserCreator)

public:
  RobotLogParserCreator()
  {
    _checkbox_use_timestamp = new QCheckBox("use field [monotonic_time]");
  }

  virtual QWidget* optionsWidget()
  {
    return _checkbox_use_timestamp;
  }

  PJ::MessageParserPtr createInstance(const std::string& topic_name, PJ::PlotDataMapRef& data) override {
    return std::make_shared<RobotLogParser>(topic_name, data, _checkbox_use_timestamp->isChecked());
  }
  const char* name() const override { return "RobotLog"; }

protected:
  QCheckBox* _checkbox_use_timestamp;
};
