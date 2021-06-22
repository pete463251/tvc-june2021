#!/usr/bin/env python

from pylab import * 
import csv

print "hello"
close('all')


f1 = csv.reader(  open( "/Users/pete/Desktop/Launches/Flight06-F10-061821/IMU-flight6.txt" , "rU") )
data = [ this_row for this_row in f1]

# pitch rotates about the x-axis gyroX is d(pitch)/dt
# roll rotates about the y-axis. gyroY is d(roll)/dt
roll = []
pitch = []
heading = []
time = []
timedelta = []
gyroX = []
gyroY = []
gyroZ = []
lx = []
ly = []
lz = []
pitchVal = []
pitchSetPoint = []
rollSetPoint = []
spinSetPoint = []
rollOld = 0
pitchOld = 0
rollSetPoint = []
time_offset = 18000
maxDeviation = 3.5 #degrees of allowable gimbaling
speedZ = []
altitude = []
for index, x in enumerate(data):
	
	if( index == 0):
		start_time = x[9]
		old_time = float(start_time)-1
		
	heading.append( float(x[0]) )
	roll.append( float(x[1]) )
	#rollDerivative.append( 1000*(float(x[1]) - rollOld)/(float(x[9])-old_time) ) 
	rollOld = float(x[1])
	pitch.append( float(x[2]) )
	gyroX.append( 180/3.14*float( x[3] ) )
	gyroY.append( 180/3.14*float( x[4] ) )
	gyroZ.append( 180/3.14*float( x[5] ))
	lx.append( float(x[6]))
	ly.append( float(x[7]))
	lz.append( float(x[8]))	
	
	time.append( float(x[9]) - float(start_time) )
	timedelta.append( float(x[9]) - old_time)
	old_time = float(x[9])
	
	if( index== 0):
		speedZ.append( 0 )
		altitude.append(0 )
	else:
		speedZ.append( speedZ[index-1] - float(x[8])*timedelta[index]/1000. )
		altitude.append( altitude[index-1] + speedZ[index]*timedelta[index]/1000.)
    #pitch corresponds to rotation about x-axix on my rocket
	if( float(x[2]) > 0 ):
		pitchValCurrent = 180 - float(x[2])	
	else:
		pitchValCurrent = -180 - float(x[2])
		
	pitchValCurrent = float( x[2] )
	pitchVal.append( pitchValCurrent  )
	#pitchDerivative.append( 1000*( pitchValCurrent-pitchOld)/(float(x[9])-old_time) )
	pitchOld = pitchValCurrent
	

	currentPitchVal = 180/pi*float( x[11] )
	currentRollVal = 180/pi*float( x[10] )
	if( currentPitchVal  > maxDeviation ):
		currentPitchVal = maxDeviation
	elif( currentPitchVal < -maxDeviation ):
		currentPitchVal = -maxDeviation

	if( currentRollVal  > maxDeviation ):
		currentRollVal = maxDeviation
	elif( currentRollVal < -maxDeviation ):
		currentRollVal = -maxDeviation
			
	# pitch/roll vals are at 0 prior to ignition
	#if( float(x[9])< (1614800415651-4500)):
	#	currentPitchVal = 0
	#	currentRollVal = 0

	rollSetPoint.append( currentRollVal )
	pitchSetPoint.append( currentPitchVal )
	spinSetPoint.append( float( x[12] ) )
	



# shifts start time from start of ignition wait sequence to just before launch
#time = [(x-time_offset)/1000 for x in time]
time = [x/1000 for x in time]
#print len(x)
#print len(gyroZ)


# ejection charge fired ####
plot( [7.1, 7.1], [-500, 500], 'r' )   # ejection charge fired
plot( [9.6, 9.6], [-500, 500], 'g' )   # motor burnout

'''
p = plot( time, rollSetPoint, 'k')
p = plot( time, roll, 'b')
title('Roll and Roll Setpoint')
ylabel('roll (degrees)')
ylim([-15, 30])
'''

'''

p = plot( time, pitchSetPoint, 'k')
p = plot( time, pitchVal, 'b')
title('Pitch and Pitch Setpoint')
ylabel('Pitch (degrees)')
ylim([-65, 35])
'''

'''
# this is the linear acceleration section
p = plot(time, lx, 'k')
p = plot(time, ly, 'b')	
p = plot( time, lz, 'g')
title('Accelerations: X, Y, Z')
ylabel('m/s/s')
ylim([-10, 20])
'''

# heading and motorspin setpoint
p = plot(time, gyroZ, 'k')
p = plot(time, spinSetPoint, 'b')
title('Spin and SpinSetPpoint')
ylim([-400, 400])

plt.setp(p, linewidth=2)
grid()
xlabel('Seconds')

xlim([-0.2, 10])


#plot([  (float(1614800415651)-float(start_time)- time_offset)/1000, (float(1614800415651)-float(start_time)-time_offset)/1000], [-500, 500], 'r')
# estimated motor burnout
#plot([  (float(1614800415651-1900)-float(start_time)-time_offset)/1000, (float(1614800415651-1900)-float(start_time)-time_offset)/1000], [-500, 500], 'm')
# estimated motor start
#plot([  (float(1614800415651-4500)-float(start_time)-time_offset)/1000, (float(1614800415651-4500)-float(start_time)-time_offset)/1000], [-500, 500], 'g')

plt.show()