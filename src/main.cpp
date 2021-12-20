#include <Arduino.h>
#include "datalogger.h"

// Setup and Loop

void setup(void)
{
  // disable unused components and hardware pins //
  componentsAlwaysOff();
  hardwarePinsAlwaysOff();

  startSerial2();

  setupSwitchedPower();
  enableSwitchedPower();

  setupHardwarePins();

  //blinkTest();

  // Don't respond to interrupts during setup
  disableClockInterrupt();
  disableUserInterrupt();

  clearClockInterrupt();
  clearUserInterrupt();

  //  Prepare I2C
  i2c_bus_reset(I2C1);
  //i2c_disable(I2C1);
  //i2c_master_enable(I2C1, 0);
  Wire.begin();
  delay(1000);
  scanIC2(&Wire);

  // Clear the alarms so they don't go off during setup
  clearAllAlarms();

  initializeFilesystem();

  //initBLE();

  readUniqueId(uuid);

  allocateMeasurementValuesMemory();

  setupWakeInterrupts();

  powerUpSwitchableComponents();

  setNotBursting(); // prevents bursting during first loop

  //awakeTime = timestamp(); // Push awake time forward and provide time for user interation during setup()

  /* We're ready to go! */
  Monitor::instance()->writeDebugMessage(F("done with setup"));
  Serial2.flush();
}

/* main run loop order of operation: */
void loop(void)
{
  bool bursting = checkBursting();
  bool debugLoop = checkDebugLoop();
  bool awakeForUserInteraction = checkAwakeForUserInteraction(debugLoop);
  bool takeMeasurement = checkTakeMeasurement(bursting, awakeForUserInteraction);

  // Should we sleep until a measurement is triggered?
  bool awaitMeasurementTrigger = false;
  if (!bursting && !awakeForUserInteraction)
  {
    Monitor::instance()->writeDebugMessage(F("Not bursting or awake"));
    awaitMeasurementTrigger = true;
  }

  if (awaitMeasurementTrigger) // Go to sleep
  {
    stopAndAwaitTrigger();
    return; // Go to top of loop
  }

  if (WaterBear_Control::ready(Serial2))
  {
    handleControlCommand();
    return;
  }

  if (WaterBear_Control::ready(getBLE()))
  {
    Monitor::instance()->writeDebugMessage(F("BLE Input Ready"));
    awakeTime = timestamp(); // Push awake time forward
    WaterBear_Control::processControlCommands(getBLE());
    return;
  }

  if (takeMeasurement)
  {
    takeNewMeasurement();
    trackBurst(bursting);
    if (DEBUG_MEASUREMENTS)
    {
      monitorValues();
    }
  }

  if (configurationMode)
  {
    monitorConfiguration();
  }

  if (debugValuesMode)
  {
    if (burstCount == burstLength) // will cause loop() to continue until mode turned off
    {
      prepareForTriggeredMeasurement();
    }
    monitorValues();
  }

  if (tempCalMode)
  {
    monitorTemperature();
  }
}
