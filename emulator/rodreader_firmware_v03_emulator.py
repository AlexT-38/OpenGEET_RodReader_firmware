# -*- coding: utf-8 -*-
"""
Created on Tue Jul 19 22:23:54 2022

Emulate the behaviour of the rodreader arduino firmware v03 (April 2014)

version 0.1:
    communications with RodReaderGUI2D
version 0.2:
    all firmware functions (v 0.3) implemented,
    rudimentary simulation of data using random amplitude sine waves
    random amplitude selected at scan start

@author: Washu
"""
import serial
import os
import time
import math
import random

cursor_char = '>'
device_id_str = 'RODREADER EMULATOR v0.1 (target: 03, APR 2014)\n'

def echo(string):
    print(string)
    try:
        #debugging coms - I suspect I'm accidentally transmitting multiple linefeeds?
        if string.endswith('\n'):
            print("string already has line feed! :", string)
        else:
            string = string + '\n'
        ser.write(bytes(string,'UTF-8'))
    except serial.SerialTimeoutException:
        print('Timed out waiting to send')
    
def getParam(param_str, min_value, max_value):
    try:
        value = int(param_str)
        if value >= min_value and value <= max_value:
            echo('OK')
            return value
        else:
            echo('Out of range')
            return None
            
    except ValueError:
        echo('Not a real number')
        return None
    
base_xyz = [1,1,1]
def simulateMagXYZ():
    """
     simulate sensor reading with sinusoids over scan range
     
     output is 12 bit, and represents a milliGauss range set by SensorGain, roughly 1-8 Gauss
     resolution is dependent on gain and averaging:
         gain \ samples (approx) = LSB
             0      2       4       8
         0   3.0    2.5     2.0     1.5
         1   2.5    2.0     1.5     1.7
         2   2.0    1.5     1.25    1.95
         3   1.5    1.25    1.0     0.75
         4   1.25   1.0     0.75    0.6
         5   1.0    0.75    0.6     0.5
         6   0.9    0.6     0.55    0.45
         7   0.7    0.55    0.5     0.4
         
    suitable functions will need to be derived to limit the synthesized value to the given resolution and add appropriate noise
    as well as simulate the sensor's averaging function
    """
    
    #get the base field strength
    base_x = base_xyz[0]
    base_y = base_xyz[1]
    base_z = base_xyz[2]
    
    #calculate an angle base on c
    theta = 2 * math.pi * (status['motorPosition'] ) / (params['scanPoints'] * params['scanSteps'])
    
    #initialise the sample
    value_x = 0;
    value_y = 0;
    value_z = 0;
    
    #perform n samples
    avg_samples = 1 #should be from sensor config
    for n in range(avg_samples):
        #generate sine curve
        x = base_x * math.sin(theta)
        y = base_y * math.cos(theta)
        z = base_z * math.sin(theta)
        
        #add some random noise (should be gain dependant)
        x = x + random.gauss(0,0.01)
        y = y + random.gauss(0,0.01)
        z = z + random.gauss(0,0.01)
        
        #convert those values to 9 bit (should be gain dependent)
        value_x = value_x + int(x * 512)
        value_y = value_y + int(y * 512)
        value_z = value_z + int(z * 512)
        
        #this should give us about the right range for MSb(111b) = 8Gauss
    
        #device measurement time
        time.sleep(0.006)
        
    magData = [value_x//avg_samples, 
               value_y//avg_samples, 
               value_z//avg_samples]
    
    status['magData'] = magData
    
    
def printMagXYZ():
    magData = status['magData']
    line = f"{status['motorPosition']}: {magData[0]}, {magData[1]}, {magData[2]}"
    echo(line)
    
def printDeviceID():
    echo(cursor_char+device_id_str)

def reset():
    pass

base_scan = False
def scan():
    #set a random field strength for this pass
    base_xyz[0] = random.gauss(1,0.1)
    base_xyz[1] = random.gauss(0.5,0.1)
    base_xyz[2] = random.gauss(0.1,0.05)
    #configure HMC5883
    time.sleep(0.001)# simulate i2c protocol
    
    #set stepper speed to rapid
    setMotorSpeed(params['scanRapid'])
    #move to start position
    sMoveTo(params['scanStart'])
    #set stepper speed to scan
    setMotorSpeed(params['scanSpeed'])
    #report scan is starting
    echo('SCAN')
    #loop through points
    for n in range(params['scanPoints']-1):
        echo(f'LOOP: {n}')
        #simulate data
        simulateMagXYZ()
        #write data to serial
        printMagXYZ()
        #move to next point
        sMove(params['scanSteps'])
        time.sleep(params['scanDelay']/1000)
    #final sample
    echo(f"LOOP: {params['scanPoints']}")
    #simulate data
    simulateMagXYZ()
    #write data to serial
    printMagXYZ()
    echo('END')
    
    time.sleep(  0.5  ) #short delay before retruning to start
    
    setMotorSpeed(params['scanRapid'])
    sMoveTo(0)
    
    #put sensor into idle mode
    
    #set the base scan, indicating to the HMC5883L emulator that at least one scan has been made
    base_scan = True    #not sure how much use this is- the firmware is agnostic of calibration pass vs scan

def tilt():
    #not implemented in the firmware being emulated
    pass

def setSteps(steps):
    value = getParam(steps, 1, config['scanStepsMax'])
    if value is not None:
        params['scanSteps'] = value

def setPoints(points):
    value = getParam(points, 1, config['scanPointsMax'])
    if value is not None:
        params['scanPoints'] = value

def setSamples(samples):
    value = getParam(samples, 1, config['samplesMax'])
    if value is not None:
        params['samples'] = value

def setSpeed(speed):
    value = getParam(speed, 1, config['scanSpeedMax'])
    if value is not None:
        params['scanSpeed'] = value

def setRapid(rapid):
    value = getParam(rapid, 1, config['scanRapidMax'])
    if value is not None:
        params['scanRapid'] = value

def setDelay(delay):
    value = getParam(delay, 1, config['scanDelayMax'])
    if value is not None:
        params['scanDelay'] = value

def getParams():
    #It's unknown at this point if the order is important to the GUI. It should not be, but...
    echo('PARAMETERS')
    for key, value in params.items():
        if key.startswith('sensor'):
            #use hexadecimal for printing sensor config bytes
            echo(f"{param_names[key]} {value:x}")
        else:
            echo(f"{param_names[key]} {value}")
    echo('END')

def saveParams():
    with open(param_filepath, 'w') as param_file:
        for key, value in params.items():
            param_file.write(f'{key}:{value}\n')

def loadParams():
    try:
        with open(param_filepath, 'r') as param_file:
            lines = param_file.readlines()
            for line in lines:
                param_name, param_val = line.split(':')
                params[param_name] = int(param_val)
        echo('LOADED')
    except FileNotFoundError:
        pass
    #display the current parameters after load attempt
    getParams()

def resetParams():
    global params
    params = params_default.copy()

def clearParams():
    os.remove(param_filepath)
    echo('CLEARED')

def move(distance):
    value = getParam(distance, -config['moveMax'], config['moveMax'])
    if value is not None:
        setMotorSpeed(params['scanRapid'])
        sMove(value)
        

def moveTo(position):
    value = getParam(position, -config['moveMax'], config['moveMax'])
    if value is not None:
        setMotorSpeed(params['scanRapid'])
        sMoveTo(value)

def setStartPos(position):
    if len(position) > 0:
        value = getParam(position, 0, config['scanTravelMax'])
    else:
        value = status['motorPosition']
    params['scanStart'] = value

def setZeroPos():
    #optionally could add position relative to current zero, or current pos?
    status['motorPosition'] = 0

def getPos():
    echo(f"POSITION {status['motorPosition']}")

def setSensorGain(gain):
    """
    set sensor gain on HMC5883L 3axis magnetometer: 

    Parameters
    ----------
    gain : INT
            gain, field range (+/- Ga)
            0     0.88
            1     1.3
            2     1.9
            3     2.5
            4     4.0
            5     4.7
            6     5.6
            7     8.1

    """
    value = getParam(gain, 0, 7)
    if value is not None:
        regB = value << 2
        params['sensorConfigB'] = regB
        

def setSensorRate(rate):
    """
    set sampling rate on HMC5883L 3axis magnetometer:

    Parameters
    ----------
    rate : INT
        rate, sample rate (Hz)
        0     0.75
        1     1.5
        2     3
        3     7.5
        4     15
        5     30
        6     75
    """
    value = getParam(rate, 0, 6)
    if value is not None:
        regA = params['sensorConfigA']
        regA = regA & 0xE3
        regA = regA | (value<<2)
        params['sensorConfigA'] = regA

def setSensorAvg(avg):
    """
    set averaging samples on HMC5883L 3axis magnetometer: 

    Parameters
    ----------
    avg : INT
        avg,  samples
        0     1
        1     2
        2     4
        3     8
    """
    value = getParam(avg, 0, 3)
    if value is not None:
        regA = params['sensorConfigA']
        regA = regA & 0x9F
        regA = regA | (value<<5)
        params['sensorConfigA'] = regA
        
    
command_list = {'RESET':reset,
                'SCAN':scan,
                'TILT':tilt,
                'STEPS':setSteps,
                'POINTS':setPoints,
                'SAMPLES':setSamples,
                'SPEED':setSpeed,
                'RAPID':setRapid,
                'DELAY':setDelay,
                'GETPARAMS':getParams,
                'SAVE':saveParams,
                'LOAD':loadParams,
                'DEFAULT':resetParams,
                'CLEAR':clearParams,
                'MOVE':move,
                'MOVETO':moveTo,
                'START':setStartPos,
                'ZERO':setZeroPos,
                'GETPOS':getPos,
                'SENSORGAIN':setSensorGain,
                'SENSORRATE':setSensorRate,
                'SENSORAVG':setZeroPos,
                'U':printDeviceID}

param_filepath = './params.cfg'

params_default = {  'scanPoints': 160,      # number of points to sample
            'scanSteps' :  10,      # motor steps between points
            'scanSpeed' :  25,      # motor speed when moving to a sample point
            'scanDelay' :   0,      # delay after moving, before sampling
            'scanRapid' :  40,      # motor speed when jogging to a position, eg start pos
            'samples'   :   1,      # number of readings to average at each point
            'scanStart' :   0,      # scan start position in motor steps
            'sensorConfigA' : 0x18,  #no oversampling, max data rate, no bias
            'sensorConfigB' : 0x20}
params = params_default.copy()

#parameter names as per firmware 'GETPARAMS'
param_names = {'scanPoints': 'POINTS',
            'scanSteps' :  'STEPS',
            'scanSpeed' :  'SPEED',
            'scanDelay' :  'DELAY',
            'scanRapid' :  'RAPID',
            'samples'   :  'SAMPLES',
            'scanStart' :  'START',
            'sensorConfigA' : 'SENSOR CONFIG A',  #no oversampling, max data rate, no bias
            'sensorConfigB' : 'SENSOR CONFIG B'}

config = {'scanTravelMax': 2200,
          'moveMax'      : 5000,
          'scanPointsMax': 2100,
          'scanStepsMax' : 2100,
          'scanSpeedMax' :  100,
          'scanDelayMax' : 1000,
          'scanRapidMax' :  100,
          'samplesMax'   :  255,
          'motorSteps'   :  200,
          'motorStepSize':0.125
          }

#todo store sensor config and use to condition simulated readings
sensor_gain = []
sensor_rate = []
sensor_avg = []
sensor_config = {'gain':0,
                 'rate':0,
                 'avg':0}

status = {'motorPosition': 0,
          'motorSpeed'   : 0,
          'magData'      : [0.0,0.0,0.0]}


def motorStep(steps):
    """
    Sleeps for a time equivalent to rotating x steps at speed y rpm

    """
    rpm = status['motorSpeed']
    rps = rpm/60
    stpr = config['motorSteps']
    stps = rps*stpr
    spst = 1/stps
    #print(f'sleeping for {steps} steps of {spst} seconds each, totaling {steps*spst} seconds')
    print(f'Moving for {steps*spst} seconds')
    for n in range(steps):
        time.sleep(  spst  )
    
def setMotorSpeed(rpm):
    status['motorSpeed'] = rpm
    
def sMove(steps):
    print("pos:", status['motorPosition'])
    print("moving:", steps)
    motorStep(steps)
    status['motorPosition'] = status['motorPosition'] + steps
    print("pos:", status['motorPosition'])
    
def sMoveTo(position):
    print("pos:", status['motorPosition'])
    print("moving to:", position)
    steps = position - status['motorPosition']
    print("moving:", steps)
    motorStep(steps)
    status['motorPosition'] = position

def do_loop():
    """
    emulates the 'loop' function in the arduiono source code
    waits for a '\n' terminated string and compares against the available commands
    """
    ser_input = ser.readline()
    #command = str(ser.read_until('\n', serial_buffer_size),'UTF-8')
    command = str(ser_input, 'UTF-8')
    
    
    #check the string started
    if len(command) > 0:
        #check the string ended
        if command[-1] != '\n':
            raise ValueError('no termination character')
        else:
            #strip the line terminator
            command = command[:-1]
            #echo the command
            echo('>'+command)
        
        #go through each command in the command dictionary and compare to the start of the string
        for key, func in command_list.items():
            if command.startswith(key):
                #this is the correct command, strip the command to get the param
                param = command[len(key):]
                
                #attempt to call the function with the parameter
                try:
                    func(param)
                except TypeError:
                    #type error is raised when attempting to pass a parameter into a function that doesnt accept it
                    #call the function with no params instead
                    func()
                return
                

if __name__ == "__main__":
    
    #ser = serial.Serial('COM3')
    with serial.Serial('COM3', timeout=0.050, write_timeout=1) as ser:
        print(ser.name)
        
        print('waiting for sync...')
        #non-blocking attempt to pick up the sync /display ID command
        while True:
            ser_input = ser.readline()
            if len(ser_input) > 0:
                                
                if ser_input[0] == 0x55:
                    #respond appropriately to the sync command
                    printDeviceID()
                    
                    #attempt to load params from file, otherwise save defaults
                    if os.path.isfile(param_filepath):
                        loadParams()
                    else:
                        saveParams()
                        
                    #begin executing the main loop
                    while True:
                        do_loop()
                else:
                    print('read:',ser_input)
            