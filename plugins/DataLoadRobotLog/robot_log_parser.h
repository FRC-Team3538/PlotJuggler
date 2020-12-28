#pragma once

#include "proto/StatusFrame_generated.h"
#include <endian.h>
#include <stdio.h>
#include "PlotJuggler/dataloader_base.h"

class Parser
{
private:
  PJ::PlotDataMapRef& plot_data;

  std::string get_id(const rj::CTREMotorStatusFrame* motor);

  std::string get_id(const rj::PDPStatusFrame* pdp);

  std::string get_id(const rj::PCMStatusFrame* pcm);

  int plot_frame(double time, const rj::CTREMotorStatusFrame* motor);

  int plot_frame(double time, const rj::PDPStatusFrame* pdp);

  int plot_frame(double time, const rj::PCMStatusFrame* pcm);

public:
  Parser(PJ::PlotDataMapRef &plot_data): plot_data(plot_data) {}

  int plot_frame(const rj::StatusFrameHolder *status_frame);
};