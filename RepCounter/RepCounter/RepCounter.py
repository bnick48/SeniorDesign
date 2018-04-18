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




#Changeable Parameters #########################################
repInterval = 2
percentTurnPerRep = 50
maxReps = 10
motorSpeedPercent = float(50)


#Initialize variables
toggle = 0
dc = 0
GPIO.output(11, GPIO.LOW)
rampInterval = 5
pwm.start(0) #start sending signal 
repCount = 0;

#Actual Program #################################################
potVal = mcp.read_adc(0)
dcStart = float(potVal) / 1050 * 100

while True:
    inputState = GPIO.input(18)
    potVal = mcp.read_adc(0)
    dc = float(potVal) / 1050 * 100
    if (dc > float(dcStart + percentTurnPerRep)):
        time.sleep(0.3)
        repCount = repCount + 1

    if repCount == repInterval:
        dc = motorSpeedPercent
        rampUp(dc)
        time.sleep(0.2)
        rampDown(dc)

    if repCount == maxReps:
        print "Max Reps Reached: ", maxReps
        dc = 50
        rampDown(dc)
        rampUp(dc)
        time.sleep(0.5)
        dc = 0
        rampDown(dc)

    if  inputState == False:
        print('Button Pressed')
        time.sleep(0.2)
        dir_toggle()
    time.sleep(0.05)


    #print(dc)
    #pwm.ChangeDutyCycle(dc)

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

