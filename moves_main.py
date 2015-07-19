import pandas as pd
import numpy as np
import os, re
import math

# class Vehicle(object):
#   """
#   VehNr: (int) The ID number of Vehicle
#   Type: (int) The Type of Vehicle
#   Weight: (float) Vehicle weight in ton
#   v: (float) speed of Vehicle in mph
#   a: (float) acceleration of Vehicle in ft/s^2
#   grad: (float) road gradient at current location in %
#   A: (float) rolling coefficient in kW*sec/meter
#   B: (float) rotation coefficient in kW*sec^2/meter^2
#   C: (float) drag coefficient in kW*sec^3/meter^3
#   M: (float) vehicle source mass in metric tons
#   f: (float) fixed mass factor in metric tons
#   """
#   def __init__(self, VehNr, Type, Weight, v, a, grad):
#       super(Vehicle, self).__init__()
#       self.VehNr = VehNr
#       self.Type = Type
#       self.Weight = Weight
#       self.v = v
#       self.a = a
#       self.grad = grad

def findSourceType(VehType, M):
    '''
    Map VISSIM vehicle type into MOVES source type
    VehType: VISSIM vehicle type number
    M: vehicle weight in metric tons
    '''
    if 100 == VehType:
        return 21
    if 200 ==VehType:
        if 15 < M < 25:
            return 41
        if 25 < M < 36:
            return 62
        if 3.8 < M < 6.3:
            return 53
        if 1.5 < M < 3.8:
            return 32
    if 300 == VehType:
        return 42


    # elif 200 == VehType and 15<M<22:
    #     sourceType = 41
    # elif 200 == VehType and 27<M<36:
    #     sourceType = 62
    # elif 200 == VehType and 3.8<M<6.3:
    #     elif 
    #     sourceType = 53
    # elif 300 == VehType:
    #     sourceType = 42
    # return sourceType

def findParameters(sourceType):
    '''Find vehicle parameters for specific source type'''
    if 21 == sourceType:
        A = 0.156461
        B = 0.002002
        C = 0.000493
        M = 1.4788
        f = 1.4788
    elif 32 == sourceType:
        A = 0.235008
        B = 0.003039
        C = 0.000748
        M = 2.05979
        f = 2.05979        
    elif 41 == sourceType:
        A = 1.29515
        B = 0        
        C = 0.003715
        M = 19.5937
        f = 17.1
    elif 42 == sourceType:
        A = 1.0944
        B = 0
        C = 0.003587
        M = 16.556
        f = 17.1
    elif 53 == sourceType:
        A = 0.498699
        B = 0
        C = 0.001474
        M = 6.25047
        f = 17.1        
    elif 62 == sourceType:
        A = 2.08126
        B = 0
        C = 0.004188
        M = 31.4038
        f = 17.1
    return A, B, C, M, f

def findVSP(A, B, C, M, f, v, a, grad):
    '''
    Find VSP of vehicle for current time
    '''
    tan_theta = grad/100
    sin_theta = math.sqrt(tan_theta**2 / (1 + tan_theta**2))
    VSP = (A*v + B*(v**2) + C*(v**3) + M*(a + 9.8*sin_theta)*v) / f
    return VSP


def findOperationMode(VSP, v, a):
    '''
    Find operating mode of vehicle for current time
    The operating mode number returned by this function is not MOVES operation mode ID, but the link ID 1~23, corresponding to 23 operation modes, instead.
    This is for convenience in table lookup
    '''
    if a <= -2:
        return 1
    elif v < 1 and v > -1:
        return 2
    elif v < 25:
        if VSP < 0: return 3
        if VSP < 3: return 4
        if VSP < 6: return 5
        if VSP < 9: return 6
        if VSP < 12: return 7
        return 8
    elif v < 50:
        if VSP < 0: return 9
        if VSP < 3: return 10
        if VSP < 6: return 11
        if VSP < 9: return 12
        if VSP < 12: return 13
        if VSP < 18: return 14
        if VSP < 24: return 15
        if VSP < 30: return 16
        return 17
    else:
        if VSP < 6: return 18
        if VSP < 12: return  19
        if VSP < 18: return 20
        if VSP < 24: return 21
        if VSP < 30: return 22
        return 23




    pass

def moves_main():
    '''
    Evaluate the total emission with VISSIM output *.fzp file and MOVES emission rate
    '''
    #GRAVITY = 9.8   #gravitational acceleration in m/s^2
    mph2ms = 0.44704    #convert mph to m/s
    fts2ms = 0.3048 #convert ft/s^2 to m/s^2
    vehData = pd.read_csv('i710.fzp', skiprows = 21, sep = ';', skipinitialspace = True)
    # abandon strings in the last column
    vehData = vehData[list(vehData.columns[:-1])]
    vehData.dropna(inplace = True)
    vehData.sort(['VehNr', 't'], inplace = True)
    nVeh = int(max(vehData.VehNr)) #Largest vehicle ID number, used in vehicle iteration
    throughput = 0 #Total number of vehicles
    VMT = 0 #vehicle miles travelled
    travelTime = 0
    HC = 0
    CO = 0
    NOX = 0
    CO2 = 0
    energy = 0
    PM25 = 0

    emissionRates = pd.read_csv('LosAngeles_201507_emissionrate_lookup.csv')
    pass
    #emissions = {}



    for i in range(1, nVeh + 1):
        veh = vehData[i == vehData.VehNr]
        if 0 == len(veh):
            continue
        throughput += 1
        print throughput
        VehType = veh.Type.iloc[0]
        M = veh.Weight.iloc[0] #Vehicle weight in metric tons
        print VehType, M
        sourceType = findSourceType(VehType, M)
        A, B, C, M, f = findParameters(sourceType)
        emissions = emissionRates[emissionRates.sourceTypeID == sourceType]
        for sec in range(len(veh)):
            v = veh.iloc[sec]['v'] * mph2ms
            a = veh.iloc[sec]['a'] * fts2ms
            grad = veh.iloc[sec]['Grad']
            VSP = findVSP(A, B, C, M, f, v, a, grad)
            operationMode = findOperationMode(VSP, v/mph2ms , a/mph2ms)
            VMT += veh.iloc[sec]['v'] / 3600
            travelTime += 1 / 60
            emission = emissions[emissions.linkID == operationMode]
            HC += float(emission[emission.pollutantID == 1]['ratePerDistance']) / 3600
            CO += float(emission[emission.pollutantID == 2]['ratePerDistance']) / 3600
            NOX += float(emission[emission.pollutantID == 3]['ratePerDistance']) / 3600
            CO2 += float(emission[emission.pollutantID == 90]['ratePerDistance']) / 3600
            energy += float(emission[emission.pollutantID == 91]['ratePerDistance']) / 3600
            PM25 += float(emission[emission.pollutantID == 110]['ratePerDistance']) / 3600
            pass
    rates = [HC, CO, NOX, CO2, energy, PM25]
    for i in len(rates):
        rates[i] /= VMT
    print rates, throughput


        #travelTime += (veh.iloc[-1]['t'] - veh.iloc[0]['t']) / 60

        #VMT +=
    pass




# This is the standard boilerplate that calls the main() function.
if __name__ == '__main__':
    moves_main()
