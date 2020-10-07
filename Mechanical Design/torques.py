import pandas as pd
import numpy as np
print('Running torques...')
data = pd.read_excel('/home/jason/Documents/MQP/Arm Measurements.xlsx')
print(data)

lengths = data['Length (mm)'].tolist()
masses = data['Mass (grams)'].tolist()
print(lengths)
print(masses)
factor = 1000.0
scaledLength = [x / factor for x in lengths]
scaledMass = [x / factor for x in masses]
arrayLength = len(scaledMass)

print(scaledLength)

massArray = np.array(scaledMass)
massArrayP1 = np.delete(massArray, [0, len(massArray)-2,len(massArray)-1])
massArrayP = np.flip(massArray)

lengthArray = np.array(scaledLength)
lengthArrayP1 = np.delete(lengthArray, [0, len(lengthArray)-2,len(lengthArray)-1])
lengthArrayP = np.flip(lengthArray)
print('\nLengthArrayP:')
print(lengthArrayP)
print('\nMassArrayP')
print(massArrayP)

# Moment of Intertia J = mL^2
motorMass = []
J = []
massAdd = np.copy(massArrayP)
massAdd[0] = massArrayP[0]
prevMass = 0
# print('Running moments of Intertia...')
# for i in range(len(massArrayP-1)):
#     totalMass = massArrayP[i] + prevMass + motorMass[i]
#     #print(totalMass)
#     temp = totalMass*((lengthArrayP[i])**2)
#     prevMass = massArrayP[i] + prevMass
#     J.append(temp)
#print('\nMoments of Inertia: ')
#print(J)

# Torque Calculations T = F*R
joint = ['Rotate at 1', 'Pivot at 2', 'Rotate at 3', 'Pivot at 4', 'Rotate at 5', 'Pivot at 6']
jointP = np.flip(joint)
T = []
Tozin = []
g = 9.8
print('Running Torques...')
prevMass = 0
prevLength = 0
for i in range(len(massArrayP-1)):
    # Input mass of components needed for previous joint
    motorMass = (input("Mass of New Components in grams: "))

    # Calculate total mass of all parts the motor will be rotating
    totalMass = massArrayP[i] + prevMass + (motorMass/1000.0)

    # Calculate total length of all parts motor will be rotating
    totalLength = (lengthArrayP[i] + prevLength)

    # Print new masses and lengths
    print('\nNew Total Mass:')
    print(totalMass)
    print('New Total Length')
    print(totalLength)

    # Temp is the required torque for the motor in question
    temp = ((totalMass)*g*(totalLength*.75))

    # Convert to oz-in for motor selection
    tempOzin = temp * 141.6119;
    T.append(temp)
    Tozin.append(tempOzin)

    # Print needed torque
    print("Torque needed for " + jointP[i])
    print(str(tempOzin) + " oz-in")

    # Update for next interation
    prevMass= totalMass
    prevLength = totalLength

print('\nTorques (Nm)')
print(T)

print('\nTorques (oz-in)')
print(Tozin)
