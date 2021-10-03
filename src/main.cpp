#include <Arduino.h>
#include <libmaple/libmaple.h>
#include <libmaple/pwr.h> // necessary?
#include <string.h>
#include <Ezo_i2c.h>

//DHT22 humidity sensor stuff
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN PC11     // Digital pin connected to the DHT sensor 
#define DHTTYPE DHT22     // DHT 22 (AM2302)

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;
//end DHT22 stuff

#include "datalogger.h"
#include "scratch/dbgmcu.h"
#include "system/watchdog.h"
#include "sensors/atlas_rgb.h"
#include "utilities/qos.h"
#include "system/hardware.h"


// Setup and Loop

void setupSensors(){

  // read sensors types from EEPROM
  // malloc configuration structs
  // read configuration structs from EEPROM for each sensor type
  // run setup for each sensor

  
  // Setup RGB Sensor
  //AtlasRGB::instance()->setup(&WireTwo);

}

void setup(void)
{
  startSerial2();

  startCustomWatchDog();
  
  // disable unused components and hardware pins //
  componentsAlwaysOff();
  //hardwarePinsAlwaysOff(); // TODO are we turning off I2C pins still, which is wrong

  setupSwitchedPower();

  //Serial2.println("hello");
  enableSwitchedPower();

  setupHardwarePins();
  //Serial2.println("hello");
  //Serial2.flush();

  //Serial2.println(atlasRGBSensor.get_name());
  // digitalWrite(PA4, LOW); // turn on the battery measurement

  //blinkTest();

  // Set up the internal RTC
  RCC_BASE->APB1ENR |= RCC_APB1ENR_PWREN;
  RCC_BASE->APB1ENR |= RCC_APB1ENR_BKPEN;
  PWR_BASE->CR |= PWR_CR_DBP; // Disable backup domain write protection, so we can write
  

  // delay(20000);

  allocateMeasurementValuesMemory();

  setupManualWakeInterrupts();

  powerUpSwitchableComponents();
  delay(2000);

   // Don't respond to interrupts during setup
  disableManualWakeInterrupt();
  clearManualWakeInterrupt();

  // Clear the alarms so they don't go off during setup
  clearAllAlarms();

  initializeFilesystem();

  //initBLE();

  readUniqueId(uuid);

  setNotBursting(); // prevents bursting during first loop
  setNotBurstLooping(); // prevent bursting during first loop

  /* We're ready to go! */
  Monitor::instance()->writeDebugMessage(F("done with setup"));
  Serial2.flush();

  //setupSensors();
  Monitor::instance()->writeDebugMessage(F("done with sensor setup"));
  Serial2.flush();
  
  print_debug_status(); 

  //DHT22 Stuff
    // Initialize device.
  dht.begin();
  Serial.println(F("DHT22 Initializing"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  
}

/* main run loop order of operation: */
void loop(void)
{
  startCustomWatchDog();
  printWatchDogStatus();

  // Get reading from RGB Sensor
  // char * data = AtlasRGB::instance()->mallocDataMemory();
  // AtlasRGB::instance()->takeMeasurement(data);
  // free(data);
 
  checkMemory();

  // allocate and free the ram to test if there is enough?
  //nvic_sys_reset - what does this do?
  char debugBuffer[100];
  sprintf(debugBuffer, "burstLoopCount=%d, burstCount=%d, timestamp=%d", burstLoopCount,burstCount, (int)timestamp());
  Monitor::instance()->writeDebugMessage(F(debugBuffer));

  bool burstLooping = checkBurstLoop();
  bool bursting = checkBursting();
  bool debugLoop = checkDebugLoop();
  bool awakeForUserInteraction = checkAwakeForUserInteraction(debugLoop);
  bool takeMeasurement = checkTakeMeasurement(bursting, awakeForUserInteraction);

  //Adding DHT test here
   // Delay between measurements.
  //delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  //if (isnan(event.temperature)) {
  //  Serial.println(F("Error reading temperature!"));
  //}
  //else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  //}
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
  //End of DHT test code


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
    if (figMethane == true)
    {
      Serial2.println("turning on 5v boost");
      Serial2.flush();
      digitalWrite(GPIO_PIN_3, HIGH);
    }
    checkFirstBurst(bursting, awakeForUserInteraction);
    if (burstLooping && burstCount == 0) // delay once at the start of each new burst
    {
      Serial2.print("delay (min):");
      Serial2.println(burstDelay);
      Serial2.flush();
      warmup(burstDelay);
    }
    takeNewMeasurement();
    trackBurst(bursting);
    trackBurstLoop(burstLooping);
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
