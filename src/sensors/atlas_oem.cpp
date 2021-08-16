#include "atlas_oem.h"
#include "system/monitor.h"

EC_OEM *oem_ec;

void setupEC_OEM(TwoWire *wire)
{

  Monitor::instance()->writeDebugMessage(F("EC I2C setup"));
  Monitor::instance()->writeDebugMessage(F("Setting up EC OEM board"));

  oem_ec = new EC_OEM(wire, NONE_INT, ec_i2c_id);
  bool awoke = oem_ec->wakeUp();

  char message[300];
  sprintf(message, "Device addr EC: %x\nDevice type EC: %x\nFirmware EC: %x\nAwoke: %i\nHibernating: %i",
          oem_ec->getStoredAddr(), oem_ec->getDeviceType(), oem_ec->getFirmwareVersion(), awoke, oem_ec->isHibernate());
  Monitor::instance()->writeDebugMessage(message);

  /*
  oem_ec->singleReading();
  struct param_OEM_EC parameter;
  parameter = oem_ec->getAllParam();

  Monitor::instance()->writeDebugMessage(F("test:"));
  sprintf(message, "salinity= %f\nconductivity= %f\ntds= %f\nSalinity stable = %s",
          parameter.salinity, parameter.conductivity, parameter.tds, (oem_ec->isSalinityStable() ? "yes" : "no"));
  Monitor::instance()->writeDebugMessage(message);
  */

  oem_ec->setLedOn(true);
  //oem_ec->setLedOn(false);
  //oem_ec->setLedOn(true);
  oem_ec->setProbeType(1.0);

  Monitor::instance()->writeDebugMessage(F("Done with EZO I2C setup"));
}

void hibernateEC_OEM()
{
  oem_ec->setHibernate();
  delete  oem_ec;
}

void clearECCalibrationData()
{
  oem_ec->clearCalibrationData();
}

void setECDryPointCalibration()
{
  oem_ec->setCalibration(DRY_CALIBRATION);
}

void setECLowPointCalibration(float lowPoint)
{
  oem_ec->setCalibration(LOW_POINT_CALIBRATION, lowPoint);
}

void setECHighPointCalibration(float highPoint)
{
  oem_ec->setCalibration(HIGH_POINT_CALIBRATION, highPoint);
}

bool readECDataIfAvailable(float *ecValue)
{
  bool newDataAvailable = oem_ec->singleReading();

  if (newDataAvailable)
  {
    *ecValue = oem_ec->getConductivity();
    oem_ec->clearNewDataRegister();
  }
  return true;
}