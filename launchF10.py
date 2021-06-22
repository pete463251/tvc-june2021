import logging
import sys
import math
import time
import pigpio
import RPi.GPIO as GPIO

from Adafruit_BNO055 import BNO055
import F10thrustcurve as thrust
import H13thrustcurve as thrustH13

F10 = thrust.F10
#H13 = thrustH13.H13

forceFactorLookup = [10./x for x in F10]
#forceFactorLookup = [10./x for x in H13]

#print len(F10)
#print len(forceFactorLookup)
#print forceFactorLookup

#### initiate servos #####
servo2 = 18 #### servo1 is the X-servo
servo1 = 23 #### servo2 is the Y-servo
motorFWD = 25 ### GPIO25 is pin 22
motorREV = 8  ### GPIO8 is pin 24

pwm = pigpio.pi()
pwm.set_mode( servo1, pigpio.OUTPUT)
pwm.set_mode( servo2, pigpio.OUTPUT)
pwm.set_mode( motorFWD, pigpio.OUTPUT)
pwm.set_mode( motorREV, pigpio.OUTPUT)
 
pwm.set_PWM_frequency( servo1, 50 )    #rotation servos use 50Hz PWM
pwm.set_PWM_frequency( servo2, 50 )
pwm.set_PWM_frequency( motorFWD, 500 )
pwm.set_PWM_frequency( motorREV, 500 )


# servo1 moves to +x by 0.033deg/count
# servo2 moves to +y by -0.03deg/count
maxDegreesX = 3.5  #how far in degrees are servos allowed to move
maxDegreesY = 3.5

# define min/max/CenterVal2
centerVal1 = 1640
servo1DegPerCount = 0.021
servo1RadPerCount = servo1DegPerCount*3.14/180
minVal1 = int( centerVal1 - maxDegreesX/servo1DegPerCount)  # mapping is 0.021 degrees/servo count
maxVal1 = int( centerVal1 + maxDegreesX/servo1DegPerCount)  #
rangeVal1 = (maxVal1-minVal1)/2
pwm.set_servo_pulsewidth(servo1, centerVal1)

centerVal2 = 1600
servo2DegPerCount = 0.021
servo2RadPerCount = servo2DegPerCount*3.14/180
minVal2 = int( centerVal2 - maxDegreesY/servo2DegPerCount)  #
maxVal2 = int( centerVal2 + maxDegreesY/servo2DegPerCount)  #
rangeVal2 = (maxVal2-minVal2)/2
pwm.set_servo_pulsewidth(servo2, centerVal2)

print "minVal1 = " + str(minVal1) + ", maxVal1 = " + str(maxVal1) + ", servo1RadPerCount = " + str(servo1RadPerCount)
print "minVal2 = " + str(minVal2) + ", maxVal2 = " + str(maxVal2) + ", servo2RadPerCount = " + str(servo2RadPerCount)
###### done initaiting servos #######

f = open("IMU.txt", "a")
f.write("-------------------------")
f.write(time.strftime("      %a %d-%m-%Y @ %H:%M:%S") )
f.write("\n")

g = open("IMUbackup.txt", "a")
g.write("-------")
g.write(time.strftime("      %a %d-%m-%Y @ %H:%M:%S") )
g.write("\n")


current_milli_time = lambda: int(round(time.time() * 1000))  #prints out time in milliseconds; current_milli_time()
print("current_milli_time= " + str(current_milli_time() ) )

bno = BNO055.BNO055(serial_port='/dev/serial0', rst=6)


###### This section changes the accelerometer range to be 16G #######
### based on https://forums.adafruit.com/viewtopic.php?f=1&t=85097
### Notes:
### 0x08 = BNO055_ACC_CONFIG_ADDR
### 0x07 = BNO055_PAGE_ID_ADDR
### default accelerometer setting is 4G, last two bits of register 08 is 01
### change to last two bits being 11. Data sheet here, look for Table 3-8
### https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf 
###
time.sleep(0.1)
savePageID = bno._read_byte(0x07)
time.sleep(0.1)
bno._write_byte(0x07, 0x01)     
time.sleep(0.1)
print bno._read_byte(0x07)      
time.sleep(0.1)
print bno._read_byte(0x08)   
time.sleep(0.1)
bno._write_byte(0x08, 0xFF)       
time.sleep(0.1)
print bno._read_byte(0x08) 
time.sleep(0.1)
print "got"
bno._write_byte(0x07, savePageID & 0xFF)
print "here"
######## End section on changing accelerometer range #########


# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
    f.write('Failed toinitialize BNO055')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
#f.write('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
#f.write('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
#f.write('\n')

# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')
    #f.write('System error: {0}'.format(error))
    #f.write('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

print('starting 60 seconds with no logging')
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)    #if using same script as IMU, this must be BCM. Can be Board for Pin number otherwise
GPIO.setup(20, GPIO.OUT, initial=GPIO.LOW)  #pin 38=GPIO20, MOSFET gate for ejection charge
print('Ejection Charge Set Low')

#make n circles. when viewed from below, this should trace out a counterclockwise circle
for n in range(3):
	#blah
	pwm.set_PWM_dutycycle( motorFWD, 0)
        pwm.set_PWM_dutycycle( motorREV, 0)
	num_points = 40
	for increment in range(num_points):
		angle = 2*3.14*increment/num_points
		servoXval = centerVal1 + rangeVal1*math.cos( angle )
		servoYval = centerVal2 + rangeVal2*math.sin( angle )
		pwm.set_servo_pulsewidth( servo1, servoXval)
		pwm.set_servo_pulsewidth( servo2, servoYval)
		angle_deg = angle*180./3.14
		#print("servoXval={0}, servoYval={1}, angle={2}".format( servoXval, servoYval, angle_deg ) )
		time.sleep(0.03)

pwm.set_PWM_dutycycle( motorFWD, 0)
pwm.set_PWM_dutycycle( motorREV, 0)
pwm.set_servo_pulsewidth( servo1, centerVal1)
pwm.set_servo_pulsewidth( servo2, centerVal2)
############# done n circles, stress test #######################

###  calibrate loop #########
for n in range(55):  #should be range 85 or so
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()
    # Print everything out.
    print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
    		heading, roll, pitch, sys, gyro, accel, mag))

    time.sleep(1)


f.write("Latest Calibration Data\n")
f.write('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}\n'.format(
                heading, roll, pitch, sys, gyro, accel, mag))
g.write("Latest Calibration Data\n")
g.write('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}\n'.format(
                heading, roll, pitch, sys, gyro, accel, mag))

print("We should be calibrated on the IMU now, if not restart")
time.sleep(1)
print("You should be putting the rocket on the launchpad now with igniter installed")


while( True ):
	print("Confirm calibration or kill the script. Rocket should be on the launchpad at the proper angle now")
        userInput = str( input("Enter 2 to continue:  ") )
        if( userInput =='2' ):
                break

#if we want to offset any IMU angle bias this is where we would do it....
heading, roll, pitch = bno.read_euler()
#######
if( pitch > 0 ):
    pitchVal = 180 - pitch
else:
    pitchVal = -180 - pitch
desiredPitch = pitchVal      #this is where we want to point
desiredPitch = 0   ## HARDWIRED TO 0
desiredRoll = 0
print("Desired pitch = {0:0.2F} deg, and Desired roll = {1:0.2F} degrees".format(desiredPitch, desiredRoll) )
print("This is the F10 script, confirm F10 motor")
while( True ):
        print("Confirm orientation or kill the script")
        userInput = str( input("Enter 2 to continue:  ") )
        if( userInput =='2' ):
                break



print("This is the F10 script, confirm F10 motor")
print("F10: Fire when ready - no more messages will be given.")

# wait for launch and then do control loop
maxAZ = -12      #-9.8 is the value at rest; this is not a light touch, but is vulnerable to shock
gx = 0
gy = 0
gz = 0  #gyroscope values
while True:
	time.sleep(0.01)
	ax,ay,az = bno.read_accelerometer()
	#f.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2) )
	#g.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2) )
	if( az < maxAZ):
		# there is an event happening. to make sure its not jitter, check it again
		time_start = current_milli_time()
		time.sleep(0.01)
		ax,ay,az = bno.read_accelerometer()
		f.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2) )
		#g.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2) )
		if( az < maxAZ ):
			#sustained. check it again
			time.sleep(0.01)
			ax,ay,az = bno.read_accelerometer()
			if( az<maxAZ):
				f.write('Launch Detected\n')
				f.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2) )
                                g.write('Launch Detected\n')
                                #g.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2) )
				
				#ok calling it a launch now
				maxAZ = az
				#print('MaxAZ = {0}'.format(maxAZ) )
				break

f.close()
f = open("IMU.txt", "a")
g.close()
g = open("IMUbackup.txt", "a")
##################### end of launch loop ########


#### main control loop
#print('starting main control loop')
time_old = current_milli_time()
#time_start = time_old           #we are about 0.2seconds into the burn at this point
Kp = 0.48*3.14/180.     #offset by  3.14/180 to convert to radians
Kd = 0.15            #no need to offset because return value for gx, gy, gz is in rad/s
Ki = 0.1*3.14/180.   #offset to convert to radians
roll_integral = 0
servoYval = centerVal2
pitch_integral = 0
ejectionfirecheck = True      #this is a flag to see if we have fired the ejection charge
motorburnoutcheck = True      #this flag determines if rocket motor is still burning
forwardSpinFlag = True        #once I spin forward, I force it to never spin backward again [battery issue]
reverseSpinFlag = True       #once I spin backward, I will set the other flag to false so can't spin forward
while True:
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    #sys, gyro, accel, mag = bno.get_calibration_status()
    # Print everything out.

    #print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
    #	heading, roll, pitch, sys, gyro, accel, mag))
    # Other values you can optionally read:
    # Orientation as a quaternion:
    #x,y,z,w = bno.read_quaternion()
    # Sensor temperature in degrees Celsius:
    #temp_c = bno.read_temp()
    #print('Temperature = {0:0.2F}'.format(temp_c) )
    # Magnetometer data (in micro-Teslas):
    #mx,my,mz = bno.read_magnetometer()
    #print('Magnetometer x,y,z = {0:0.2F}, {1:0.2F}, {2:0.2F}'.format(x,y,z) )
    # Gyroscope data (in degrees per second):
    gx,gy,gz = bno.read_gyroscope()
    #print('Gyroscope x,y,z = {0:0.2F}, {1:0.2F}, {2}'.format(x,y,z) )
    # Accelerometer data (in meters per second squared):
    #ax,ay,az = bno.read_accelerometer()
    #print('Accelerometer x,y,z = {0:0.2F}, {1:0.2F}, {2}'.format(x,y,z) )
    # Linear acceleration data (i.e. acceleration from movement, not gravity--
    # returned in meters per second squared):
    lx,ly,lz = bno.read_linear_acceleration()
    #print('Linear Acceleration x,y,z = {0:0.2F}, {1:0.2F}, {2}'.format(x,y,z) )
    # Gravity acceleration data (i.e. acceleration just from gravity--returned
    # in meters per second squared):
    #grx,gry,grz = bno.read_gravity()
    #print('Gravity x,y,z = {0:0.2F}, {1:0.2F}, {2}'.format(x,y,z) )

    #f.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}, {16}, {17}, {18}, {19}, {20}, {21}, {22}, {23}'.format(heading, roll, pitch, sys, gyro, accel, mag, temp_c, mx, my, mz, gx, gy, gz, ax, ay, az, lx, ly, lz, grx, gry, grz, current_milli_time() ) )
    #f.write('\n')
    # Sleep for a second until the next reading.
    
    #this loop is on the order of 25mS, measured 18-37.
    
    time_now = current_milli_time()
    deltaT = (time_now - time_old)/1000.
    #print deltaT
    if( (time_now - time_start)> 7100 ):  # motor burns for 7.1s, don't need to update servos after that
    	motorburnoutcheck = False    
	if( (time_now - time_start)> 9500 ):  #for weight=21oz=590g, optimal delay is about 2.35-2.9s after burn out depending on nose cone shape, ~9.6s burn time after detecting launch get here; include sleep delay
		if( ejectionfirecheck ):
			#turn off spin motor to reduce battery loading for firing
        		pwm.set_PWM_dutycycle( motorREV, 0)
        		pwm.set_PWM_dutycycle( motorFWD, 0)
			time.sleep( 0.1 )     # 100mS delay;  some time to make sure spin motor isn't pulling current
			#print("fired ejection charge - delete this line") 
			ejectionfirecheck = False
			#fire!
			f.write("About to fire Ejection Charge - {0}\n".format( current_milli_time() ) )
			f.close()
        		f = open("IMU.txt", "a") 
        		g.write("About to fire Ejection Charge - {0}\n".format( current_milli_time() ) )
        		g.close()
        		g = open("IMUbackup.txt", "a") 
			###### MAKE SURE THIS IS ON FOR AN ACTUAL FLIGHT !!!! #####
			GPIO.output(20, GPIO.HIGH)   #Firing ejection charge
			f.write("Fired Ejection Charge\n - {0}\n".format( current_milli_time() ) )
        		f.close()
        		f = open("IMU.txt", "a") 
        		g.write("Fired Ejection Charge\n - {0}\n".format( current_milli_time() ) )
        		g.close()
        		g = open("IMUbackup.txt", "a") 
			#motor has burned out
	
		#this if statement takes 1 mS
		if( time_now - time_start > 62000 ):  #flight time expected to be 52.5s approximately
			break
    
    if( motorburnoutcheck ):    #after motor burns out, no need to update Servos 
    	# pitch rotates about the x-axis gyroX is d(pitch)/dt
    	# roll rotates about the y-axis. gyroY is d(roll)/dt
    
    	roll_derivative = gy  #don't multiply by 57.32 since already in radians/sec       #180/pi = 57.32. convert to degrees/s
    	roll_integral = roll_integral + (roll-desiredRoll)*deltaT
    	#print "roll_integral = " + str(roll_integral)

    	forceIndex = (time_now-time_start)/50   #should be an integer >= 0
    	forceFactor = forceFactorLookup[ forceIndex ]
	#print "F10[ forceIndex ] = " +  str(F10[forceIndex]) + ", and forceFactor = " + str( forceFactor )

    	rollSetpoint = -Kp*(roll-desiredRoll) - Kd*(roll_derivative) - Ki*(roll_integral)
	#print "roll = " + str( roll )
	#print "roll_derivative=gy= " + str( gy )
	
    	#print "time= " + str( (time_now-time_start)/1000. ) + " and force = " + str( F10[ forceIndex ] ) + " Newtons"
    	rollSetpoint = rollSetpoint*forceFactor
    	servoYval = centerVal2 + rollSetpoint/servo2RadPerCount   #map desired radians to servo counts

    	#roll corresponds to rotation about y-axis per my rocket
    	#servoYval = centerVal2 + roll*(maxVal2-minVal2)/2/45.
    	if( servoYval > maxVal2 ):
		servoYval = maxVal2
		#print('max value for servo2 reached + {0}'.format(maxVal2))
    	if( servoYval < minVal2 ):
		servoYval = minVal2
		#print('min value for servo2 reached + {0}'.format(minVal2))

	#print "servoYval = " + str( servoYval )
    
    	#pitch corresponds to rotation about x-axix on my rocket
    	if( pitch > 0 ):
		pitchVal = 180 - pitch
    	else:
		pitchVal = -180 - pitch
    
    	pitch_derivative = gx     #already in radians/sec *57.32        #180/pi=57.32
    	pitch_integral = pitch_integral + (pitchVal-desiredPitch)*deltaT
    	pitchSetpoint = -Kp*(pitchVal-desiredPitch) - Kd*(pitch_derivative) - Ki*pitch_integral
        #print "pitch_integral = " + str( pitch_integral )
	#print "pitch_derivative=gx = " + str( pitch_derivative )
	#print "pitchVal = " + str( pitchVal )

    	pitchSetpoint = pitchSetpoint*forceFactor
    	servoXval = centerVal1 - pitchSetpoint/servo1RadPerCount
    	if( servoXval > maxVal1 ):
		servoXval = maxVal1
    	elif( servoXval < minVal1 ):
		servoXval = minVal1
    
        #print "servoXval = " + str(servoXval)

    	pwm.set_servo_pulsewidth( servo2, servoYval )
    	pwm.set_servo_pulsewidth( servo1, servoXval )
	
    if( ejectionfirecheck ):   #after ejection charge is fired, don't update servos
	#spin control algorithm here
	# we update the spin motor even while coasting to prevent spin torque
	#gz_deg = gz*57.3  #180/pi=57.3
    	if( gz > 0.174 and forwardSpinFlag):
        	pwm.set_PWM_dutycycle( motorFWD, 255) #resets occured at 255 when battery not full charge (3.69V).
        	pwm.set_PWM_dutycycle( motorREV, 0)
        	spinSetpoint = 255
		reverseSpinFlag = False         #this means motor can't be reversed
    	elif( gz < -0.174 and reverseSpinFlag ):
        	pwm.set_PWM_dutycycle( motorREV, 255)
        	pwm.set_PWM_dutycycle( motorFWD, 0)
        	spinSetpoint = -255
		forwardSpinFlag = False     #motor is going backwards, can't go forward
    	else:
        	pwm.set_PWM_dutycycle( motorREV, 0)  ##!!! update this to zero!!!!
        	pwm.set_PWM_dutycycle( motorFWD, 0)
        	spinSetpoint = 0
    else:
	#after ejection charge is fired, just turn everything off
        pwm.set_servo_pulsewidth( servo2, centerVal1 )
        pwm.set_servo_pulsewidth( servo1, centerVal2 )                
	pwm.set_PWM_dutycycle( motorREV, 0)
        pwm.set_PWM_dutycycle( motorFWD, 0)
        spinSetpoint = 0
	
    
    
    time_old = time_now
    
    #debug portion for looking at servo angles - needs deleted
    #pitchSetAngle = (servoXval - centerVal1)*0.033
    #rollSetAngle = (servoYval - centerVal2)*0.03
    #print('pitch={0} deg, roll={1} deg, heading={2} deg, pitchSetAngle={3}, rollSetAngle={4}'.format(pitchVal, roll, heading, pitchSetAngle, rollSetAngle) )
    #time.sleep( 0.5 )
    ####### should be deleted #####

    #print("pitchSetpoint = {0}, rollSetpoint = {1}, deltaT = {2}".format(pitchSetpoint, rollSetpoint, deltaT) )
    #f.write('{0:0.2F}, {1:0.2F}, {2:0.2F}, {3:0.2F}, {4:0.2F}, {5:0.2F}, {6:0.2F}, {7:0.2F}, {8:0.2F}, {9:0.2F}, {10:0.2F}, {11:0.2F}, {12:0.2F}, {13:0.2F}, {14:0.2F}, {15:0.2F}, {16:0.2F}, {17:0.2F}, {18:0.2F}, {19:0.2F}, {20:0.2F}, {21:0.2F}, {22:0.2F}, {23:0.2F}, {24:0.2F}, {25:0.2F}, {26:0.2F}, {27:0.2F}, {28:0.2F}, {29:0.2F}, {30:0.2F}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, lx, ly, lz, grx, gry, grz, mx, my, mz, x, y, z, w, temp_c, sys, gyro, accel, mag, time_old, rollSetpoint, pitchSetpoint, spinSetpoint) )
    f.write('{0:0.2F}, {1:0.2F}, {2:0.2F}, {3:0.2F}, {4:0.2F}, {5:0.2F}, {6:0.2F}, {7:0.2F}, {8:0.2F}, {9:0.2F}, {10:0.2F}, {11:0.2F}, {12:0.2F}\n'.format(heading, roll, pitchVal, gx, gy, gz, lx, ly, lz, time_old, rollSetpoint, pitchSetpoint, spinSetpoint) )
    #f.close()
    #f = open("IMU.txt", "a") 
    #g.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, time_old, rollSetpoint, pitchSetpoint) )
    #g.close()
    #g = open("IMUbackup.txt", "a")
# exited the main control loop - motor is exhausted



f.write("-------------------------")
f.close()
g.write("-------------------------")
g.close()
