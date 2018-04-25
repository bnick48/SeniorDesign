#Rep Counter 
#GPIO Imports
import RPi.GPIO as GPIO
import time

## Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Hardware SPI configuration:
SPI_PORT   = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

#Set GPIO Pins
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(11,GPIO.OUT)

GPIO.setup(12,GPIO.OUT)
pwm = GPIO.PWM(12, 100)

# Was used for button
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP) 
inputState = GPIO.input(18)



#Initialize variables
toggle = 0
dc = 0
GPIO.output(11, GPIO.LOW)
rampInterval = 5
pwm.start(0) #start sending signal 
repCount = 0
intervalReset = 0
i=0
running = True
dcArray = [0]
dcSpeed = []

#Functions ########################################################

#Ramp Down Motor Function
def rampDown(power = dc):
    global pwm
    global rampInterval
    
    print "Starting power : ", power
    while power > 0:
        power = power - rampInterval
        if power < 0:
            power = 0
        pwm.ChangeDutyCycle(float(power))
        time.sleep(0.01)
        print "Power Changing: ", power

#Ramp Up Motor Function
def rampUp(power = dc):
    global pwm
    global rampInterval
    currentLvl = 0
    print "Ending power: ", power
    while currentLvl < power:
        currentLvl = currentLvl + rampInterval
        if currentLvl > power:
            currentLvl = power
        pwm.ChangeDutyCycle(float(currentLvl))
        time.sleep(0.01)
        print "Power Changing: ", currentLvl

#Toggle Direction of motor with ramp up/down
def dir_toggle():
    global toggle
    global pwm
    global dc
    if toggle == 0:
        rampDown(dc)
        GPIO.output(11, GPIO.HIGH)
        rampUp(dc)
        toggle = 1
        time.sleep(0.1)
        print "Toggle Value: ", toggle
    else :
        rampDown(dc)
        GPIO.output(11, GPIO.LOW)
        rampUp(dc)
        toggle = 0
        time.sleep(0.1)
        print "Toggle Value: ", toggle
        

#Changeable Parameters #########################################
repWaitTime = .75
repInterval = 2
percentTurnPerRep = 30
maxReps = 6
motorSpeedPercent =float(98)
motorRunTime = 0.01
nextRep = 2

#Actual Program #################################################
potVal = mcp.read_adc(0)
#dcStart = float(10)         #Hardcode start val
dcStart = float(potVal) / 1050 * 100
if dcStart >= float(15):
    dcStart = float(15)

while running:
    inputState = GPIO.input(18)         #Initialize for button
    potVal = mcp.read_adc(0)            #Get value from pot
    dc = float(potVal) / 1050 * 100     #Turns potVal into percentage turned

    dcArray.append(dc)
    #print(dcArray)
    dcSpeed.append(abs(dcArray[i+1] - dcArray[i]))
    i=i+1
    
    if (dc > float(dcStart + percentTurnPerRep)):
        time.sleep(repWaitTime)
        repCount = repCount + 1
        print "repCount: ", repCount

    if dc > (float(percentTurnPerRep + dcStart )):
        if repCount == nextRep:
            dc = motorSpeedPercent
            rampUp(dc)
            time.sleep(motorRunTime)
            rampDown(dc)
            nextRep = nextRep + repInterval
            

    if repCount >= maxReps:
        print "Max Reps Reached: ", maxReps
        dc = 0
        dir_toggle()
        dc = 98.0
        rampUp(dc)
        time.sleep(1.25)
        rampDown(dc)
        running = False

    if  inputState == False:
        print('Button Pressed')
        time.sleep(0.2)
        running = False

           
    time.sleep(0.02)
    print(dc)

### Calculating Speed and Outputting
print "Program Finished"
dcSpeed.remove(dcSpeed[0])              #Remove first value
avgSpeed = sum(dcSpeed)/len(dcSpeed)    #Calculate Average (%turn / Unit of time)
avgSpeed = avgSpeed * 3.7699
maxSpeed = max(dcSpeed) * 3.7699

#print "dcArray: ", dcArray
#print "dcSpeed: ", dcSpeed

print "Average speed: ", avgSpeed , "in/s"
print "Average speed: ", avgSpeed * 0.0254 , "m/s"
#print "Max speed: ", maxSpeed , "in/s"
#print "Max speed: ", maxSpeed * 0.0254 , "m/s"

    



