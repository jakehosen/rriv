# waterteam-embedded

- Install the file COLUMNS.TXT into the root directory of your SD Card, and change field names 5-10 to reflect the sensors attached to analog inputs 0-5 (A0 - A5).
  - Current columns.txt: duuid,uuid,time.s,time.h,battery.V,temperature.V,data3,data4,data5,data6,conductivity.mS,time.TC,C1,V1,C2,V2,M,B,temperature.C,Burst,UserValue,UserNote
  - Deployment uuid, uuid, epoch time in seconds, time in yyyy-mm-dd hh:mm:ss GMT, battery voltage, thermistor voltage, n/a, n/a, n/a, n/a, conductivity in microsiemens, timestamp temperature calibration, Calibration temp 1, Voltage 1, Calibration temp 2, Voltage 2, Slope(M), Intercept(B), Burst count, User input value, User input note


### CONNECTING VIA SERIAL/PROGRAMMING:
1. Use a severed development board to program STM32F103RB via usb:
2. Remove both ST-LINK jumpers
3. Connect using jumper wires[severed board -> main board]:
	1. SWD 2 -> CN7 PA14 [SWD clock] (labeled as 2 on programming header)
	2. SWD 3 -> CN7 GND [ground] (labeled as 3 on programming header)
	3. SWD 4 -> CN7 PA13 [SWD data input/output] (labeled as 4 on programming header)
	4. CN3 TX -> CN10 RX
	5. CN3 RX -> CN10 TX
4. Use a serial monitor @ baud rate 115200 [if "monitor_speed = 115200" is included in the platformio.ini this will not be necessary for vscode]
5. You may need to restart the nucleo board by hitting the Black reset button to see beginning setup output
6. If the output gets stuck at 'scanning...' then you will need to power cycle the waterbear shield, this can be done by unplugging the power pack then plugging it back in.
	1. If still stuck, there's a line in setup() called i2c_bus_reset(I2C1); that might help, uncomment, rebuild, reupload and wait for it to run. you won't see any output for a moment or any lights on. I also found that unconnecting the USB along with the power helped. If still stuck and you see a blue light on the Atlas Scientific OEM Development board, you may need to let it sit for a period, with power still plugged in, and then reset again.
7. Some linux systems will require the correct USB device permission to program.  These can be installed using the below commands:

```
sudo apt -y install stlink-tools
sudo systemctl restart udev
```

### CURRENT FUNCTIONING SERIAL COMMAND LIST [ >< angle brackets are signifiers for start and end of commands and will need to be incorporated]:
- `>WT_CONFIG[flag]<`						--- outputs readings on serial based on [flag]=time/conduct/therm
- `>CAL_DRY<`							--- step 1 of conductivity calibration
- `>CAL_LOW:12880<`						--- step 2 of conductivity calibration [match to low point fluid]
- `>CAL_HIGH:80000<`						--- step 3 of conductivity calibration [match to high point fluid]
- `>WT_DEBUG_VALUES<`						--- output values to be logged to sdcard on to serial each second
- `>WT_SET_RTC:[insert epoch timestamp]<`			--- set the real time clock [https://www.epochconverter.com/]
- `>WT_DEPLOY:[insert 25 characters exact]<`			--- give a custom name to the device
- `>WT_CLEAR_MODES<`						--- clears WT_CONFIG, WT_DEPLOY, and WT_CAL_TEMP (note: this might be combined with WT_CONFIG)
- `>WT_CAL_TEMP<`						--- display calibration data and readings (note: this might be combined with WT_CONFIG:therm)
- `>TEMP_CAL_LOW:[xxx.xxC]<`					--- low point temperature(C) calibration input
- `>TEMP_CAL_HIGH:[xxx.xxC]<`					--- high point temperature(C) calibration input
- `>WT_USER_VALUE:[10 character max value]<`				--- user serial input value
- `>WT_USER_NOTE:[30 character max note]<`				--- user serial input note
- `>WT_USER_INPUT:[10 character max value]&[30 character max note]<`	--- user serial input value & note
  - Note: I use vscode's serial monitor, and copy paste complete commands into the terminal
  - Note: WT_DEBUG_VALUES can be used to see all logged data and is useful for any calibrations

### CALIBRATION OF CONDUCTIVITY PROBE VIA SERIAL:
Note: You should only see serial output from initial setup steps (if reset/first start)
1. Use serial command: `>WT_CONFIG:conduct<` Which will queue to enter config mode. The serial output should immediately change to conductivity readings.
	1. Next we'll be giving commands for a two-point calibration using the recommended Atlas Scientific K1.0 calibration solutions which are Low:12880uS, and High:80000uS, if you are using custom calibration points, change the numerical value in the Low and High commands accordingly.
	2. These commands are based off of Atlas Scientific's EC_EZO_Datasheet pg 13/74 two point calibration theory for K 1.0
2. Have the probe in open air and let it stabilize at 0, then use serial command: `>CAL_DRY<`
	1. Note: Any output should change to 0 at this point
3. Place probe in low calibration solution, and wait for readings to stabilize. You may want to shake the probe a little to dislodge any air bubbles that may be in the sensing area. Then use serial command: `>CAL_LOW:12880<` The readings will not change at this point.
4. Clean probe (I rinse in water and dry to prevent contamination of calibration fluids), then place into High calibration solution, again shaking it to dislodge air bubbles, and waiting for readings to stabilize. Then use serial command: `>CAL_HIGH:80000<` At this point you should see the results change to ~80000
5. It would be wise to test the probe at various dilutions to confirm that it is working correctly. If anything seems awry, start over from step 1, as new data will overwrite the old.
6. Reset to reenter normal operations or use command: `>WT_CLEAR_MODES<`

### TO CONFIGURE TIME(Real Time Clock - DS3231):
1. Enter time config mode via serial command: `>WT_CONFIG:time<` Which will begin to print current timestamp in epoch
2. Enter serial command: `>WT_SET_RTC:[insert epoch time]<`
	1. example: `>WT_SET_RTC:1605578020<`
	2. Change should be immediate or after the second output line, human readable time will also change to reflect this.
	3. I use https://www.epochconverter.com/ and add a few seconds to try to maintain accuracy of timestamp, given how long it takes to copy the command
3. (optional) Enter serial command to clear output: `>WT_CLEAR_MODES<`

### TO SET THE DEPLOYMENT UUID (Universally Unique IDentifier)
1. Enter debug values mode via serial command: `>WT_DEBUG_VALUES<` Which will begin to print all logged data values
2. Enter serial command: `>WT_DEPLOY:[25 characters exact]<`
	1. examples:
		1. `>WT_DEPLOY:0123456789012345678901234<`
		2. `>WT_DEPLOY:__bench_tester_01_vDeploy<`
		3. `>WT_DEPLOY:__fridge_#01_vOEM_EC_DBG_<`
		4. `>WT_DEPLOY:__bench_#00_vRefactored__<`
	2. Change should be noticeable at second output line.
3. (optional) Enter serial command to clear output: `>WT_CLEAR_MODES<`

### TWO POINT LINEAR CALIBRATION OF THERMISTOR VIA SERIAL:
1. Boil water for high temperature, and prepare ice water bath for low temperature points.
2. Enter temperature calibration mode via serial command: `>WT_CONFIG:therm<` Which will begin to print out any relevant stored calibration info as well as raw voltage readings
3. Place thermistor and calibration thermometer(C) in ice bath, wait for 30-60s to equalize, then use serial command: `>TEMP_CAL_LOW:[xxx.xxC]<` with temperature reading off of thermometer. You should see confirmation via serial that command has been read.
4. Move thermistor and calibration thermometer(C) to hot water, wait for 30-60s to equalize, then use serial command `>TEMP_CAL_HIGH:[xxx.xxC]<` with temperature reading off of thermometer. You should see confirmation via serial, and there should now be calibrated output.
5. Reset to reenter normal operations or use command: `>WT_CLEAR_MODES<`
Note: If at any point voltage reading show as 0, that indicates an error, check connection of thermistor to Waterbear shield

### USER INPUT VALUES AND NOTES VIA SERIAL:
1. When you're in a config/debug mode you may wish to log what you're doing directly to the sdcard for future reference
2. To log a value up to 10 characters long use `>WT_USER_VALUE:[10 character max value]<`
3. To log a note up to 30 characters long use `>WT_USER_NOTE:[30 character max note]<`
4. To log both a value and note use `>WT_USER_INPUT:[10 character max value]&[30 character max note]<`

### NOTES:
- Check version of Maple is at least: framework-arduinoststm32-maple 2.10000.200103 (1.0.0)
	- This impacts some commands in the platform.ini [build flag, board build]
