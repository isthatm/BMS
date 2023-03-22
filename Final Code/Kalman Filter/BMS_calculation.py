import numpy as np
import serial
import time 
import pandas as pd

# ================= Set up ====================
serialPort = 'COM4'
baudRate = 9600
arduino = serial.Serial(serialPort, baudRate, timeout=0.1) #timeout for response on readline()
time.sleep(2) #allow time for serial port to open

field_0 = "OCV"
field_1 = "OCV_SOC"
field_2 = "Coulomb Counting"
field_3 = "Kalman Filter"

file_path = "Discharge_March_20_2023.csv"
file = open(file_path, "w")
file.write("Time" + ","+ field_0 + "," + field_1 + "," + field_2 + "," + field_3 + "\n")

# ================= Constants =====================
READ_OCV = '1'
READ_SAMPLING_CURRENT = '2'
READ_AVG_CURRENT = '3'

maxCapacity = 2.1
initSOC = 100
fitOrder = 8

# ================= Variables =====================
# Sensors, initialization
var_OCV = 0
var_currentCounter = 0
var_current = 0

# CC
totalC = 0

# KF
SOC1 = 100
SOC3 = 100
last_x = np.array([[100], [-1]])
last_P = np.array([[5, 0], [0, 0.3]])

#Time
currentTime = 0
prev_SOC_time = 0
prev_SamplingCurrent = 0

interval_SamplingCurrent = 0.5
interval_SOC = 20

# ================= Program =======================
def getData(task):
    global var_OCV, var_currentCounter, var_current
    encodedTask = str.encode(task)
    arduino.write(encodedTask) #send to Arduino
    while (arduino.in_waiting == 0): #not receiving any byte
        time.sleep(0.001) # delay while there is no incoming data
    data = arduino.readline() #the last bit gets rid of the new-line char
    if (data):
        decodedData = data.decode()
    match encodedTask:
        case b'1':
            var_OCV = float(decodedData)
        case b'2':
            var_currentCounter = float(decodedData)
        case b'3':
            var_current = float(decodedData)

def OCVtoSOC(OCV, order):
    coef = [80894.4152491213, -2258735.08625908, 27549325.9927546, -191710032.497609,	
            832512591.643289, -2310229414.41262, 4000777326.66182, -3953187759.74077, 1706426238.54662]
    ocvSOC = 0
    i = 0
    if (OCV < 2.80):
        OCV = 2.80
    elif (OCV > 4.0126):
        OCV = 4.0126
    while (i <= order):
        ocvSOC += coef[i] * OCV**(order - i) 
        i+=1
    return ocvSOC

def CC_SOC (deltaT, current):
    global totalC
    totalC += current * (deltaT) 
    CCSoC = initSOC + (totalC / (maxCapacity*3600) ) * 100
    return CCSoC

def KF( deltaT , measuredSOC, measuredCurrent): 
    global last_x, last_P
    x_measurement = np.array([[measuredSOC], [measuredCurrent]])   # State vector
    R = np.array([[1, 0], [0, 0.2]])  # measurement noise
    H = ([[1, 0], [0, 1]])    # Observation matrix
    I = ([[1, 0], [0, 1]])    # Identity matrix

    # Prediction
    F = np.array([[1, 100*(deltaT / (maxCapacity*3600) )], [0, 1]])  # Transformation matrix
    x = np.matmul(F, last_x) 
    P = np.matmul(np.matmul(F, last_P), np.transpose(F))      # Predict uncertainty
    P[0, 1] = 0
    P[1, 0] = 0

    # Correction
    S = P + R
    invS = np.linalg.inv(S)

    # Update
    K = np.matmul(P, invS)  # Kalman gain
    last_x = x + np.matmul(K, (x_measurement - x))
    #last_P = np.matmul((I - K), P)
    last_P = np.matmul(np.matmul((I - K), P), np.transpose(I - K)) + np.matmul(np.matmul(K, R), np.transpose(K))
    return last_x[0]

def calculate_SOC():
    getData(READ_AVG_CURRENT)
    getData(READ_OCV)
    SOC1 = OCVtoSOC(var_OCV, fitOrder) 
    KF_res = KF(interval_SOC, SOC1, var_current)
    SOC3 = float('.'.join(str(ele) for ele in KF_res)) 
    SOC2 = CC_SOC(interval_SOC, var_current)
    csvData = ( str(round(currentTime, 2)) + "," + str(round(var_OCV, 2)) + 
              "," + str(SOC1) + "," + str(SOC2) + "," + str(SOC3) + "\n" )
    file.write(csvData)
    print("Time: %.2f, OCV: %.2f, OCV_SOC: %.2f, CC_SOC: %.2f, KF_SOC: %.2f" % (currentTime, var_OCV , SOC1, SOC2, SOC3))

# ================= MAIN ===================

# initialization 
getData(READ_OCV)
print("OCV = %.2f" % var_OCV )

while True:
    startLoop = time.time()
    if (var_OCV < 2.8):
        file.close()
        break
    if ( (currentTime - prev_SamplingCurrent) >= interval_SamplingCurrent ):
        getData(READ_SAMPLING_CURRENT)
        prev_SamplingCurrent = currentTime
    if ( (currentTime - prev_SOC_time) >= interval_SOC ):
        calculate_SOC()
        prev_SOC_time = currentTime
    endLoop = time.time()
    currentTime += (endLoop - startLoop)
print("Out of battery!!!")

