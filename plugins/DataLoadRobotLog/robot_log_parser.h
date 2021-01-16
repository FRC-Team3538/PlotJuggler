#pragma once

#include "proto/StatusFrame_generated.h"
#include <stdio.h>
#include "PlotJuggler/dataloader_base.h"

class Parser
{
private:
  PJ::PlotDataMapRef& plot_data;

  std::string get_id(const rj::CTREMotorStatusFrame* motor);

  std::string get_id(const rj::PDPStatusFrame* pdp);

  std::string get_id(const rj::PCMStatusFrame* pcm);

  std::string get_id(const rj::REVMotorStatusFrame* pcm);

  std::string get_id(const rj::REVColorSensorStatusFrame* pcm);

  std::string get_id(const rj::NavXStatusFrame* pcm);

  std::string get_id(const rj::ADIS16470StatusFrame* pcm);

  std::string get_id(const rj::WPIDigitalInput* pcm);

  std::string get_id(const rj::WPIEncoder* pcm);

  std::string get_id(const rj::WPIDutyCycleEncoder* pcm);

  int plot_frame(double time, const rj::CTREMotorStatusFrame* motor);

  int plot_frame(double time, const rj::PDPStatusFrame* pdp);

  int plot_frame(double time, const rj::PCMStatusFrame* pcm);

  int plot_frame(double time, const rj::REVMotorStatusFrame* pcm);

  int plot_frame(double time, const rj::REVColorSensorStatusFrame* pcm);

  int plot_frame(double time, const rj::NavXStatusFrame* pcm);

  int plot_frame(double time, const rj::ADIS16470StatusFrame* pcm);

  int plot_frame(double time, const rj::WPIDigitalInput* pcm);

  int plot_frame(double time, const rj::WPIEncoder* pcm);

  int plot_frame(double time, const rj::WPIDutyCycleEncoder* pcm);

public:
  Parser(PJ::PlotDataMapRef &plot_data): plot_data(plot_data) {}

  int plot_frame(const rj::StatusFrameHolder *status_frame);
};
