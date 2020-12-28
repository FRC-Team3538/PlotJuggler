#include "robot_log_parser.h"

std::string Parser::get_id(const rj::CTREMotorStatusFrame* motor)
{
  return "Motor/" + std::to_string(motor->deviceID()) + "::" + std::to_string(motor->baseID()) +
         "::" + std::to_string(motor->firmwareVersion());
}

std::string Parser::get_id(const rj::PDPStatusFrame* pdp)
{
  return "PDP/" + std::to_string(pdp->module_());
}

std::string Parser::get_id(const rj::PCMStatusFrame* pcm)
{
  return "PCM/" + std::to_string(pcm->module_());
}

int Parser::plot_frame(double time, const rj::CTREMotorStatusFrame* motor)
{
  // CTRE Motors - SRX AND SPX and eventually falcons
  auto id = get_id(motor);
  double data;
  PJ::PlotData::Point point;
  std::unordered_map<std::string, PJ::PlotData>::iterator series;

  data = motor->supplyCurrent();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/supplyCurrent");
  series->second.pushBack(point);

  data = motor->statorCurrent();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/statorCurrent");
  series->second.pushBack(point);

  data = motor->activeTrajectoryArbFeedFwd();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/activeTrajectoryArbFeedFwd");
  series->second.pushBack(point);

  data = motor->closedLoopTarget();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/closedLoopTarget");
  series->second.pushBack(point);

  data = motor->errorDerivative();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/errorDerivative");
  series->second.pushBack(point);

  data = motor->integralAccumulator();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/integralAccumulator");
  series->second.pushBack(point);

  data = motor->temperature();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/temperature");
  series->second.pushBack(point);

  data = motor->outputVoltage();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/outputVoltage");
  series->second.pushBack(point);

  data = motor->outputPercent();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/outputPercent");
  series->second.pushBack(point);

  data = motor->busVoltage();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/busVoltage");
  series->second.pushBack(point);

  data = motor->outputCurrent();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/outputCurrent");
  series->second.pushBack(point);

  data = motor->revLimitSwitchClosed();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/revLimitSwitchClosed");
  series->second.pushBack(point);

  data = motor->fwdLimitSwitchClosed();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/fwdLimitSwitchClosed");
  series->second.pushBack(point);

  data = motor->controlMode();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/controlMode");
  series->second.pushBack(point);

  data = motor->lastError();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/lastError");
  series->second.pushBack(point);

  data = motor->faults();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/faults");
  series->second.pushBack(point);

  data = motor->activeTrajectoryVelocity();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/activeTrajectoryVelocity");
  series->second.pushBack(point);

  data = motor->activeTrajectoryPosition();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/activeTrajectoryPosition");
  series->second.pushBack(point);

  data = motor->closedLoopError();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/closedLoopError");
  series->second.pushBack(point);

  data = motor->selectedSensorVelocity();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/selectedSensorVelocity");
  series->second.pushBack(point);

  data = motor->selectedSensorPosition();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/selectedSensorPosition");
  series->second.pushBack(point);

  data = motor->resetOccured();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/resetOccured");
  series->second.pushBack(point);

  return 0;
}

int Parser::plot_frame(double time, const rj::PDPStatusFrame* pdp)
{
  // PDP
  auto id = get_id(pdp);
  double data;
  PJ::PlotData::Point point;
  std::unordered_map<std::string, PJ::PlotData>::iterator series;

  data = pdp->totalEnergy();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/totalEnergy");
  series->second.pushBack(point);

  data = pdp->totalPower();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/totalPower");
  series->second.pushBack(point);

  data = pdp->totalCurrent();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/totalCurrent");
  series->second.pushBack(point);

  data = pdp->temperature();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/temperature");
  series->second.pushBack(point);

  data = pdp->voltage();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/voltage");
  series->second.pushBack(point);

  auto currents = pdp->channelCurrent();
  for (size_t i = 0; i < currents->size(); i++)
  {
    data = currents->Get(i);
    point = PJ::PlotData::Point(time, data);
    series = plot_data.addNumeric(id + "/channels/" + std::to_string(i) + "/current");
    series->second.pushBack(point);
  }

  return 0;
}

int Parser::plot_frame(double time, const rj::PCMStatusFrame* pcm)
{
  // PCM
  auto id = get_id(pcm);
  double data;
  PJ::PlotData::Point point;
  std::unordered_map<std::string, PJ::PlotData>::iterator series;

  data = pcm->enabled();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/enabled");
  series->second.pushBack(point);

  data = pcm->pressureSwitchValve();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/pressureSwitchValve");
  series->second.pushBack(point);

  data = pcm->compressorCurrent();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/compressorCurrent");
  series->second.pushBack(point);

  data = pcm->closedLoopControl();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/closedLoopControl");
  series->second.pushBack(point);

  data = pcm->compressorCurrentTooHighFault();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/compressorCurrentTooHighFault");
  series->second.pushBack(point);

  data = pcm->compressorShortedFault();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/compressorShortedFault");
  series->second.pushBack(point);

  data = pcm->compressorNotConnectedFault();
  point = PJ::PlotData::Point(time, data);
  series = plot_data.addNumeric(id + "/compressorNotConnectedFault");
  series->second.pushBack(point);

  return 0;
}

int Parser::plot_frame(const rj::StatusFrameHolder* status_frame)
{
  auto statusFrame_type = status_frame->statusFrame_type();
  auto time = status_frame->monotonicTime();
  if (statusFrame_type == rj::StatusFrame::StatusFrame_CTREMotorStatusFrame)
  {
    auto motor = status_frame->statusFrame_as<rj::CTREMotorStatusFrame>();
    return plot_frame(time, motor);
  }
  else if (statusFrame_type == rj::StatusFrame::StatusFrame_PDPStatusFrame)
  {
    auto pdp = status_frame->statusFrame_as<rj::PDPStatusFrame>();
    return plot_frame(time, pdp);
  }
  else if (statusFrame_type == rj::StatusFrame::StatusFrame_PCMStatusFrame)
  {
    auto pcm = status_frame->statusFrame_as<rj::PCMStatusFrame>();
    return plot_frame(time, pcm);
  }
  else if ((long)statusFrame_type == 4)
  {
    // This is initialize, skip
    return 0;
  }
  else
  {
    return -1;
  }
}