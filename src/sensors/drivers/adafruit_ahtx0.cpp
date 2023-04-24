/* 
 *  RRIV - Open Source Environmental  ata Logging Platform
 *  Copyright (C) 20202  Zaven Arra  zaven.arra@gmail.com
 *  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include "adafruit_ahtx0.h"
#include "system/logs.h"
#include "system/measurement_components.h"
#include "system/eeprom.h" // TODO: ideally not included in this scope
#include "system/clock.h"  // TODO: ideally not included in this scope

AdaAHTX0::AdaAHTX0()
{
  // debug("allocation AtlasECDriver");
}
AdaAHTX0::~AdaAHTX0(){}

const char * AdaAHTX0::getSensorTypeString()
{
  return sensorTypeString;
}

configuration_bytes_partition AdaAHTX0::getDriverSpecificConfigurationBytes()
{
  configuration_bytes_partition partition;
  memcpy(&partition, &configuration, sizeof(driver_configuration));
  return partition;
}


void AdaAHTX0::configureSpecificConfigurationsFromBytes(configuration_bytes_partition configurationPartition)
{
  memcpy(&configuration, &configurationPartition, sizeof(driver_configuration));
}

void AdaAHTX0::appendDriverSpecificConfigurationJSON(cJSON * json)
{
  // debug("getting AdaAHTX0 json");
  //driver specific config, customize
  addCalibrationParametersToJSON(json);
}



void AdaAHTX0::setup()
{
  // notify("aht setup");
  aht = new Adafruit_AHTX0();
  if(!aht->begin(wire,1,AHTX0_I2CADDR_DEFAULT)){
    // notify("aht setup fail");
  }
}

// void AdaAHTX0::stop()
// {
//   //empty
// }

bool AdaAHTX0::takeMeasurement()
{
  // notify("in aht sensor driver take measurement\n");
  sensors_event_t hum, temp;
  bool measurementTaken = false;
  aht->getEvent(&hum, &temp);
  humidity = hum.relative_humidity;
  temperature = temp.temperature;
  if(isnan(humidity)){
    notify("Error reading humidity");
  } else{
    measurementTaken = true;
  }
  if(isnan(temperature)){
    notify("Error reading temperature");
  } else{
    measurementTaken = true;
  }
  if(measurementTaken)
  {
    // notify("measurement read");
    addValueToBurstSummaryMean("temperature", temperature);
    addValueToBurstSummaryMean("humidity", humidity);
    lastSuccessfulReadingMillis = millis();
    // Serial2.print("temp t %d\n",temp.timestamp);
  }

  return measurementTaken;
}
const char *AdaAHTX0::getSummaryDataString()
{
  // debug("configuring AdaAHTX0 dataString");
  // process data string for .csv
  // TODO: just reporting the last value, not a true summary
  sprintf(dataString, "%.2f", getBurstSummaryMean("aht"));
  return dataString;
}

const char *AdaAHTX0::getBaseColumnHeaders()
{
  // for debug column headers defined in the .h
  // debug("getting AdaAHTX0 base column headers");
  return baseColumnHeaders;
}
void AdaAHTX0::initCalibration()
{
  // debug("init AdaAHTX0 calibration");
}
void AdaAHTX0::calibrationStep(char *step, int arg_cnt, char ** args)
{
  // for intermediary steps of calibration process
  // debug("AdaAHTX0 calibration steps");
}
bool AdaAHTX0::configureDriverFromJSON(cJSON *json)
{
  return true;
}
void AdaAHTX0::setDriverDefaults()
{
  // debug("setting AdaAHTX0 driver defaults");
  // set default values for driver struct specific values
  configuration.cal_timestamp = 0;
}

void AdaAHTX0::addCalibrationParametersToJSON(cJSON *json)
{
  // follows structure of calibration parameters in .h
  // debug("add AdaAHTX0 calibration parameters to json");
  cJSON_AddNumberToObject(json, CALIBRATION_TIME_STRING, configuration.cal_timestamp);
}

const char * AdaAHTX0::getRawDataString()
{
  sprintf(dataString, "%.2f,%.2f", temperature, humidity);
  return dataString;
}
unsigned int AdaAHTX0::millisecondsUntilNextReadingAvailable()
{
  
  return (30000 - (millis() - lastSuccessfulReadingMillis)); // return min by default, a larger number in driver implementation causes correct delay
}
