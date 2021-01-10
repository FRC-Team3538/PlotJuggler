#include "messageparser_robot_log.h"
#include "proto/StatusFrame_generated.h"

std::string RobotLogParser::get_id(const rj::CTREMotorStatusFrame* motor)
{
  return "Motor/" + std::to_string(motor->deviceID()) + "::" + std::to_string(motor->baseID()) +
         "::" + std::to_string(motor->firmwareVersion());
}

std::string RobotLogParser::get_id(const rj::PDPStatusFrame* pdp)
{
  return "PDP/" + std::to_string(pdp->module_());
}

std::string RobotLogParser::get_id(const rj::PCMStatusFrame* pcm)
{
  return "PCM/" + std::to_string(pcm->module_());
}

std::string RobotLogParser::get_id(const rj::REVMotorStatusFrame* motor)
{
  std::string serial(motor->serialNumber()->begin(), motor->serialNumber()->end());
  return "Motor/" + std::to_string(motor->deviceID()) + "::" + serial + "::" + std::to_string(motor->firmwareVersion()) + "::" + motor->firmwareString()->str();
}

std::string RobotLogParser::get_id(const rj::REVColorSensorStatusFrame* sensor)
{
  return "Sensor/Color";
}

std::string RobotLogParser::get_id(const rj::NavXStatusFrame* navx)
{
  return "Sensor/NavX::" + navx->firmwareVersion()->str();
}

std::string RobotLogParser::get_id(const rj::ADIS16470StatusFrame* imu)
{
  return "Sensor/ADIS16470";
}

int RobotLogParser::plot_frame(double time, const rj::CTREMotorStatusFrame* motor)
{
  // CTRE Motors - SRX AND SPX and eventually falcons
  auto id = get_id(motor);
  double data;
  PJ::PlotData::Point point;
  std::unordered_map<std::string, PJ::PlotData>::iterator series;

  data = motor->supplyCurrent();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/supplyCurrent");
  series->second.pushBack(point);

  data = motor->statorCurrent();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/statorCurrent");
  series->second.pushBack(point);

  data = motor->activeTrajectoryArbFeedFwd();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/activeTrajectoryArbFeedFwd");
  series->second.pushBack(point);

  data = motor->closedLoopTarget();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/closedLoopTarget");
  series->second.pushBack(point);

  data = motor->errorDerivative();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/errorDerivative");
  series->second.pushBack(point);

  data = motor->integralAccumulator();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/integralAccumulator");
  series->second.pushBack(point);

  data = motor->temperature();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/temperature");
  series->second.pushBack(point);

  data = motor->outputVoltage();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/outputVoltage");
  series->second.pushBack(point);

  data = motor->outputPercent();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/outputPercent");
  series->second.pushBack(point);

  data = motor->busVoltage();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/busVoltage");
  series->second.pushBack(point);

  data = motor->outputCurrent();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/outputCurrent");
  series->second.pushBack(point);

  data = motor->revLimitSwitchClosed();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/revLimitSwitchClosed");
  series->second.pushBack(point);

  data = motor->fwdLimitSwitchClosed();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/fwdLimitSwitchClosed");
  series->second.pushBack(point);

  data = motor->controlMode();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/controlMode");
  series->second.pushBack(point);

  data = motor->lastError();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/lastError");
  series->second.pushBack(point);

  data = motor->faults();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/faults");
  series->second.pushBack(point);

  data = motor->activeTrajectoryVelocity();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/activeTrajectoryVelocity");
  series->second.pushBack(point);

  data = motor->activeTrajectoryPosition();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/activeTrajectoryPosition");
  series->second.pushBack(point);

  data = motor->closedLoopError();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/closedLoopError");
  series->second.pushBack(point);

  data = motor->selectedSensorVelocity();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/selectedSensorVelocity");
  series->second.pushBack(point);

  data = motor->selectedSensorPosition();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/selectedSensorPosition");
  series->second.pushBack(point);

  data = motor->resetOccured();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/resetOccured");
  series->second.pushBack(point);

  return 0;
}

int RobotLogParser::plot_frame(double time, const rj::PDPStatusFrame* pdp)
{
  // PDP
  auto id = get_id(pdp);
  double data;
  PJ::PlotData::Point point;
  std::unordered_map<std::string, PJ::PlotData>::iterator series;

  data = pdp->totalEnergy();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/totalEnergy");
  series->second.pushBack(point);

  data = pdp->totalPower();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/totalPower");
  series->second.pushBack(point);

  data = pdp->totalCurrent();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/totalCurrent");
  series->second.pushBack(point);

  data = pdp->temperature();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/temperature");
  series->second.pushBack(point);

  data = pdp->voltage();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/voltage");
  series->second.pushBack(point);

  auto currents = pdp->channelCurrent();
  for (size_t i = 0; i < currents->size(); i++)
  {
    data = currents->Get(i);
    point = PJ::PlotData::Point(time, data);
    series = _plot_data.addNumeric(id + "/channels/" + std::to_string(i) + "/current");
    series->second.pushBack(point);
  }

  return 0;
}

int RobotLogParser::plot_frame(double time, const rj::PCMStatusFrame* pcm)
{
  // PCM
  auto id = get_id(pcm);
  double data;
  PJ::PlotData::Point point;
  std::unordered_map<std::string, PJ::PlotData>::iterator series;

  data = pcm->enabled();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/enabled");
  series->second.pushBack(point);

  data = pcm->pressureSwitchValve();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pressureSwitchValve");
  series->second.pushBack(point);

  data = pcm->compressorCurrent();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/compressorCurrent");
  series->second.pushBack(point);

  data = pcm->closedLoopControl();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/closedLoopControl");
  series->second.pushBack(point);

  data = pcm->compressorCurrentTooHighFault();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/compressorCurrentTooHighFault");
  series->second.pushBack(point);

  data = pcm->compressorShortedFault();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/compressorShortedFault");
  series->second.pushBack(point);

  data = pcm->compressorNotConnectedFault();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/compressorNotConnectedFault");
  series->second.pushBack(point);

  return 0;
}

int RobotLogParser::plot_frame(double time, const rj::REVMotorStatusFrame* motor)
{
  // Spark MAX
  auto id = get_id(motor);
  double data;
  PJ::PlotData::Point point;
  std::unordered_map<std::string, PJ::PlotData>::iterator series;

  auto encoder = motor->encoder();

  data = encoder->position();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/encoder/position");
  series->second.pushBack(point);

  data = encoder->velocity();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/encoder/velocity");
  series->second.pushBack(point);

  data = encoder->positionConversionFactor();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/encoder/positionConversionFactor");
  series->second.pushBack(point);

  data = encoder->velocityConversionFactor();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/encoder/velocityConversionFactor");
  series->second.pushBack(point);

  data = encoder->averageDepth();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/encoder/averageDepth");
  series->second.pushBack(point);

  data = encoder->measurementPeriod();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/encoder/measurementPeriod");
  series->second.pushBack(point);

  data = encoder->countsPerRevolution();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/encoder/countsPerRevolution");
  series->second.pushBack(point);

  data = encoder->inverted();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/encoder/inverted");
  series->second.pushBack(point);

  data = encoder->lastError();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/encoder/lastError");
  series->second.pushBack(point);


  auto altEncoder = motor->altEncoder();

  data = altEncoder->position();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/altEncoder/position");
  series->second.pushBack(point);
  
  data = altEncoder->velocity();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/altEncoder/velocity");
  series->second.pushBack(point);
  
  data = altEncoder->positionConversionFactor();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/altEncoder/positionConversionFactor");
  series->second.pushBack(point);
  
  data = altEncoder->velocityConversionFactor();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/altEncoder/velocityConversionFactor");
  series->second.pushBack(point);
  
  data = altEncoder->averageDepth();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/altEncoder/averageDepth");
  series->second.pushBack(point);
  
  data = altEncoder->measurementPeriod();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/altEncoder/measurementPeriod");
  series->second.pushBack(point);
  
  data = altEncoder->countsPerRevolution();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/altEncoder/countsPerRevolution");
  series->second.pushBack(point);
  
  data = altEncoder->inverted();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/altEncoder/inverted");
  series->second.pushBack(point);
  
  data = altEncoder->lastError();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/altEncoder/lastError");
  series->second.pushBack(point);


  auto analog = motor->analog();

  data = analog->voltage();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/analog/voltage");
  series->second.pushBack(point);

  data = analog->position();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/analog/position");
  series->second.pushBack(point);

  data = analog->velocity();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/analog/velocity");
  series->second.pushBack(point);

  data = analog->positionConversionFactor();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/analog/positionConversionFactor");
  series->second.pushBack(point);

  data = analog->velocityConversaionFactor();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/analog/velocityConversaionFactor");
  series->second.pushBack(point);

  data = analog->averageDepth();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/analog/averageDepth");
  series->second.pushBack(point);

  data = analog->measurementPeriod();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/analog/measurementPeriod");
  series->second.pushBack(point);

  data = analog->inverted();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/analog/inverted");
  series->second.pushBack(point);


  auto pid0 = motor->pid0();

  data = pid0->p();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid0/p");
  series->second.pushBack(point);

  data = pid0->i();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid0/i");
  series->second.pushBack(point);

  data = pid0->d();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid0/d");
  series->second.pushBack(point);

  data = pid0->dFilter();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid0/dFilter");
  series->second.pushBack(point);

  data = pid0->ff();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid0/ff");
  series->second.pushBack(point);

  data = pid0->iZone();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid0/iZone");
  series->second.pushBack(point);

  data = pid0->outputMin();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid0/outputMin");
  series->second.pushBack(point);

  data = pid0->outputMax();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid0/outputMax");
  series->second.pushBack(point);

  data = pid0->smartMotionMaxVelocity();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid0/smartMotionMaxVelocity");
  series->second.pushBack(point);

  data = pid0->smartMotionMaxAccel();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid0/smartMotionMaxAccel");
  series->second.pushBack(point);

  data = pid0->smartMotionMinOutputVelocity();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid0/smartMotionMinOutputVelocity");
  series->second.pushBack(point);

  data = pid0->smartMotionAllowedClosedLoopError();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid0/smartMotionAllowedClosedLoopError");
  series->second.pushBack(point);

  data = pid0->iMaxAccum();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid0/iMaxAccum");
  series->second.pushBack(point);

  data = pid0->iAccum();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid0/iAccum");
  series->second.pushBack(point);


  auto pid1 = motor->pid1();

  data = pid1->p();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid1/p");
  series->second.pushBack(point);

  data = pid1->i();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid1/i");
  series->second.pushBack(point);

  data = pid1->d();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid1/d");
  series->second.pushBack(point);

  data = pid1->dFilter();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid1/dFilter");
  series->second.pushBack(point);

  data = pid1->ff();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid1/ff");
  series->second.pushBack(point);

  data = pid1->iZone();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid1/iZone");
  series->second.pushBack(point);

  data = pid1->outputMin();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid1/outputMin");
  series->second.pushBack(point);

  data = pid1->outputMax();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid1/outputMax");
  series->second.pushBack(point);

  data = pid1->smartMotionMaxVelocity();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid1/smartMotionMaxVelocity");
  series->second.pushBack(point);

  data = pid1->smartMotionMaxAccel();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid1/smartMotionMaxAccel");
  series->second.pushBack(point);

  data = pid1->smartMotionMinOutputVelocity();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid1/smartMotionMinOutputVelocity");
  series->second.pushBack(point);

  data = pid1->smartMotionAllowedClosedLoopError();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid1/smartMotionAllowedClosedLoopError");
  series->second.pushBack(point);

  data = pid1->iMaxAccum();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid1/iMaxAccum");
  series->second.pushBack(point);

  data = pid1->iAccum();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid1/iAccum");
  series->second.pushBack(point);


  auto pid2 = motor->pid2();

  data = pid2->p();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid2/p");
  series->second.pushBack(point);

  data = pid2->i();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid2/i");
  series->second.pushBack(point);

  data = pid2->d();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid2/d");
  series->second.pushBack(point);

  data = pid2->dFilter();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid2/dFilter");
  series->second.pushBack(point);

  data = pid2->ff();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid2/ff");
  series->second.pushBack(point);

  data = pid2->iZone();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid2/iZone");
  series->second.pushBack(point);

  data = pid2->outputMin();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid2/outputMin");
  series->second.pushBack(point);

  data = pid2->outputMax();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid2/outputMax");
  series->second.pushBack(point);

  data = pid2->smartMotionMaxVelocity();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid2/smartMotionMaxVelocity");
  series->second.pushBack(point);

  data = pid2->smartMotionMaxAccel();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid2/smartMotionMaxAccel");
  series->second.pushBack(point);

  data = pid2->smartMotionMinOutputVelocity();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid2/smartMotionMinOutputVelocity");
  series->second.pushBack(point);

  data = pid2->smartMotionAllowedClosedLoopError();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid2/smartMotionAllowedClosedLoopError");
  series->second.pushBack(point);

  data = pid2->iMaxAccum();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid2/iMaxAccum");
  series->second.pushBack(point);

  data = pid2->iAccum();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid2/iAccum");
  series->second.pushBack(point);


  auto pid3 = motor->pid3();

  data = pid3->p();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid3/p");
  series->second.pushBack(point);

  data = pid3->i();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid3/i");
  series->second.pushBack(point);

  data = pid3->d();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid3/d");
  series->second.pushBack(point);

  data = pid3->dFilter();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid3/dFilter");
  series->second.pushBack(point);

  data = pid3->ff();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid3/ff");
  series->second.pushBack(point);

  data = pid3->iZone();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid3/iZone");
  series->second.pushBack(point);

  data = pid3->outputMin();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid3/outputMin");
  series->second.pushBack(point);

  data = pid3->outputMax();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid3/outputMax");
  series->second.pushBack(point);

  data = pid3->smartMotionMaxVelocity();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid3/smartMotionMaxVelocity");
  series->second.pushBack(point);

  data = pid3->smartMotionMaxAccel();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid3/smartMotionMaxAccel");
  series->second.pushBack(point);

  data = pid3->smartMotionMinOutputVelocity();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid3/smartMotionMinOutputVelocity");
  series->second.pushBack(point);

  data = pid3->smartMotionAllowedClosedLoopError();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid3/smartMotionAllowedClosedLoopError");
  series->second.pushBack(point);

  data = pid3->iMaxAccum();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid3/iMaxAccum");
  series->second.pushBack(point);

  data = pid3->iAccum();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pid3/iAccum");
  series->second.pushBack(point);


  auto fwdLimit = motor->fwdLimitSwitch();

  data = fwdLimit->value();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/fwdLimitSwitch/value");
  series->second.pushBack(point);

  data = fwdLimit->enabled();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/fwdLimitSwitch/enabled");
  series->second.pushBack(point);


  auto revLimit = motor->revLimitSwitch();

  data = revLimit->value();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/revLimitSwitch/value");
  series->second.pushBack(point);

  data = revLimit->enabled();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/revLimitSwitch/enabled");
  series->second.pushBack(point);


// end other shit

  data = motor->idleMode();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/idleMode");
  series->second.pushBack(point);

  data = motor->voltageCompensationNominalVoltage();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/voltageCompensationNominalVoltage");
  series->second.pushBack(point);

  data = motor->openLoopRampRate();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/openLoopRampRate");
  series->second.pushBack(point);

  data = motor->closedLoopRampRate();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/closedLoopRampRate");
  series->second.pushBack(point);

  data = motor->busVoltage();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/busVoltage");
  series->second.pushBack(point);

  data = motor->appliedOutput();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/appliedOutput");
  series->second.pushBack(point);

  data = motor->outputCurrent();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/outputCurrent");
  series->second.pushBack(point);

  data = motor->temperature();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/temperature");
  series->second.pushBack(point);

  data = motor->softLimitForwardEnabled();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/softLimitForwardEnabled");
  series->second.pushBack(point);

  data = motor->softLimitReverseEnabled();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/softLimitReverseEnabled");
  series->second.pushBack(point);

  data = motor->softLimitForwardValue();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/softLimitForwardValue");
  series->second.pushBack(point);

  data = motor->softLimitReverseValue();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/softLimitReverseValue");
  series->second.pushBack(point);

  data = motor->lastError();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/lastError");
  series->second.pushBack(point);


  return 0;
}

int RobotLogParser::plot_frame(double time, const rj::REVColorSensorStatusFrame* sensor)
{
  // REV Color Sensor
  auto id = get_id(sensor);
  double data;
  PJ::PlotData::Point point;
  std::unordered_map<std::string, PJ::PlotData>::iterator series;

  data = sensor->reset();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/reset");
  series->second.pushBack(point);

  data = sensor->proximity();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/proximity");
  series->second.pushBack(point);


  auto color = sensor->color();

  data = color->red();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/color/red");
  series->second.pushBack(point);

  data = color->green();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/color/green");
  series->second.pushBack(point);

  data = color->blue();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/color/blue");
  series->second.pushBack(point);

  data = color->ir();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/color/ir");
  series->second.pushBack(point);


  return 0;
}

int RobotLogParser::plot_frame(double time, const rj::NavXStatusFrame* navx)
{
  // NavX
  auto id = get_id(navx);
  double data;
  PJ::PlotData::Point point;
  std::unordered_map<std::string, PJ::PlotData>::iterator series;

  data = navx->pitch();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pitch");
  series->second.pushBack(point);

  data = navx->roll();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/roll");
  series->second.pushBack(point);

  data = navx->yaw();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/yaw");
  series->second.pushBack(point);

  data = navx->compassHeading();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/compassHeading");
  series->second.pushBack(point);

  data = navx->isCalibrating();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/isCalibrating");
  series->second.pushBack(point);

  data = navx->isConnected();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/isConnected");
  series->second.pushBack(point);

  data = navx->byteCount();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/byteCount");
  series->second.pushBack(point);

  data = navx->updateCount();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/updateCount");
  series->second.pushBack(point);

  data = navx->lastSensorTimestamp();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/lastSensorTimestamp");
  series->second.pushBack(point);

  data = navx->worldLinearAccelX();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/worldLinearAccelX");
  series->second.pushBack(point);

  data = navx->worldLinearAccelY();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/worldLinearAccelY");
  series->second.pushBack(point);

  data = navx->worldLinearAccelZ();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/worldLinearAccelZ");
  series->second.pushBack(point);

  data = navx->isMoving();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/isMoving");
  series->second.pushBack(point);

  data = navx->isRotating();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/isRotating");
  series->second.pushBack(point);

  data = navx->barometricPressure();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/barometricPressure");
  series->second.pushBack(point);

  data = navx->altitude();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/altitude");
  series->second.pushBack(point);

  data = navx->isAltitudeValid();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/isAltitudeValid");
  series->second.pushBack(point);

  data = navx->fusedHeading();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/fusedHeading");
  series->second.pushBack(point);

  data = navx->isMagneticDisturbance();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/isMagneticDisturbance");
  series->second.pushBack(point);

  data = navx->isManetometerCalibrated();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/isManetometerCalibrated");
  series->second.pushBack(point);

  data = navx->quaternionW();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/quaternionW");
  series->second.pushBack(point);

  data = navx->quaternionX();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/quaternionX");
  series->second.pushBack(point);

  data = navx->quaternionY();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/quaternionY");
  series->second.pushBack(point);

  data = navx->quaternionZ();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/quaternionZ");
  series->second.pushBack(point);

  data = navx->velocityX();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/velocityX");
  series->second.pushBack(point);

  data = navx->velocityY();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/velocityY");
  series->second.pushBack(point);

  data = navx->velocityZ();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/velocityZ");
  series->second.pushBack(point);

  data = navx->displacementX();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/displacementX");
  series->second.pushBack(point);

  data = navx->displacementY();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/displacementY");
  series->second.pushBack(point);

  data = navx->displacementZ();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/displacementZ");
  series->second.pushBack(point);

  data = navx->angle();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/angle");
  series->second.pushBack(point);

  data = navx->rate();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/rate");
  series->second.pushBack(point);

  data = navx->angleAdjustment();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/angleAdjustment");
  series->second.pushBack(point);

  data = navx->rawGyroX();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/rawGyroX");
  series->second.pushBack(point);

  data = navx->rawGyroY();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/rawGyroY");
  series->second.pushBack(point);

  data = navx->rawGyroZ();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/rawGyroZ");
  series->second.pushBack(point);

  data = navx->rawAccelX();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/rawAccelX");
  series->second.pushBack(point);

  data = navx->rawAccelY();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/rawAccelY");
  series->second.pushBack(point);

  data = navx->rawAccelZ();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/rawAccelZ");
  series->second.pushBack(point);

  data = navx->rawMagX();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/rawMagX");
  series->second.pushBack(point);

  data = navx->rawMaxY();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/rawMaxY");
  series->second.pushBack(point);

  data = navx->rawMagZ();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/rawMagZ");
  series->second.pushBack(point);

  data = navx->pressure();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/pressure");
  series->second.pushBack(point);

  data = navx->tempC();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/tempC");
  series->second.pushBack(point);


  auto yawAxis = navx->boardYawAxis();

  data = yawAxis->boardAxis();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/boardYawAxis/boardAxis");
  series->second.pushBack(point);

  data = yawAxis->up();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/boardYawAxis/up");
  series->second.pushBack(point);


  data = navx->actualUpdateRate();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/actualUpdateRate");
  series->second.pushBack(point);

  data = navx->requestedUpdateRate();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/requestedUpdateRate");
  series->second.pushBack(point);

  data = navx->isBoardLevelYawResetEnabled();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/isBoardLevelYawResetEnabled");
  series->second.pushBack(point);

  data = navx->gyroFullScaleRangeDPS();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/gyroFullScaleRangeDPS");
  series->second.pushBack(point);

  data = navx->accelFullScaleRangeG();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/accelFullScaleRangeG");
  series->second.pushBack(point);


  return 0;
}

int RobotLogParser::plot_frame(double time, const rj::ADIS16470StatusFrame* imu)
{
  // IMU
  auto id = get_id(imu);
  double data;
  PJ::PlotData::Point point;
  std::unordered_map<std::string, PJ::PlotData>::iterator series;

  data = imu->angle();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/angle");
  series->second.pushBack(point);

  data = imu->rate();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/rate");
  series->second.pushBack(point);

  data = imu->gyroInstantX();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/gyroInstantX");
  series->second.pushBack(point);

  data = imu->gyroInstantY();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/gyroInstantY");
  series->second.pushBack(point);

  data = imu->gyroInstantZ();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/gyroInstantZ");
  series->second.pushBack(point);

  data = imu->accelInstantX();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/accelInstantX");
  series->second.pushBack(point);

  data = imu->accelInstantY();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/accelInstantY");
  series->second.pushBack(point);

  data = imu->accelInstantZ();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/accelInstantZ");
  series->second.pushBack(point);

  data = imu->xComplimentaryAngle();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/xComplimentaryAngle");
  series->second.pushBack(point);

  data = imu->yComplimentaryAngle();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/yComplimentaryAngle");
  series->second.pushBack(point);

  data = imu->xFilteredAccelAngle();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/xFilteredAccelAngle");
  series->second.pushBack(point);

  data = imu->yFilteredAccelAngle();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/yFilteredAccelAngle");
  series->second.pushBack(point);

  data = imu->yawAxis();
  point = PJ::PlotData::Point(time, data);
  series = _plot_data.addNumeric(id + "/yawAxis");
  series->second.pushBack(point);

  return 0;
}

int RobotLogParser::plot_frame(const rj::StatusFrameHolder* status_frame)
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
  else if (statusFrame_type == rj::StatusFrame::StatusFrame_InitializeStatusFrame)
  {
    // This is initialize, skip
    return 0;
  }
  else if (statusFrame_type == rj::StatusFrame::StatusFrame_REVMotorStatusFrame)
  {
    auto spark = status_frame->statusFrame_as<rj::REVMotorStatusFrame>();
    return plot_frame(time, spark);
  }
  else if (statusFrame_type == rj::StatusFrame::StatusFrame_REVColorSensorStatusFrame)
  {
    auto sensor = status_frame->statusFrame_as<rj::REVColorSensorStatusFrame>();
    return plot_frame(time, sensor);
  }
  else if (statusFrame_type == rj::StatusFrame::StatusFrame_NavXStatusFrame)
  {
    auto navx = status_frame->statusFrame_as<rj::NavXStatusFrame>();
    return plot_frame(time, navx);
  }
  else if (statusFrame_type == rj::StatusFrame::StatusFrame_ADIS16470StatusFrame)
  {
    auto imu = status_frame->statusFrame_as<rj::ADIS16470StatusFrame>();
    return plot_frame(time, imu);
  }
  else
  {
    return -1;
  }
}

bool RobotLogParser::parseMessage(const PJ::MessageRef msg,
                                      double timestamp)
{
  // TODO: gotta split this up because our messages can be sent in bulk
  int ind = 0;
  int res;
  while (ind < msg.size()) {
    uint32_t len = msg.data()[ind + 0] | msg.data()[ind + 1] << 8 | msg.data()[ind + 2] << 16 | msg.data()[ind + 3] << 24;

    uint8_t buf[len];

    memcpy(buf, msg.data() + ind, len + 4);

    auto status_frame = rj::GetSizePrefixedStatusFrameHolder(buf);

    ind += 4 + len;
    res += plot_frame(status_frame);
  }
  return res == 0;
}