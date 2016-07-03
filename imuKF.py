import numpy as np 
import matplotlib.pyplot as plt 
import csv

data_array = []

with open("output.txt", 'r') as file:
	column = zip(*[line for line in csv.reader(file, dialect="excel-tab")])
	for i in range(7):
		data_array.append(np.array(column[i], dtype='float'))
	for j in range(len(column[0])):
		data_array[0][j] /= 1000.

if __name__=='__main__':
	#show(raw_Accelero_position())
	




#def Kalman_position():


def raw_Accelero_position(): 
	Pre_time = 5.744
	Accbuff = []
	for i in range(3):
		Accbuff.append(data_array[i+1][0])
	Velbuff = [0., 0., 0.]
	Posbuff = [0., 0., 0.]
	position = [[0.], [0.], [0.]]
	newAccdata = [0.,0.,0.]
	newVeldata = [0.,0.,0.]
	#print "0,0", Accbuff[0], Velbuff[0], position[0][0]

	for phase in range(len(column[0])-1):
		for i in range(3):
			newAccdata[i] = data_array[i+1][phase+1]
		dt = data_array[0][phase+1] - Pre_time

		for i in range(3):
			newVeldata[i] += (newAccdata[i]+Accbuff[i])/2.*dt
			Posbuff[i] += (newVeldata[i]+Velbuff[i])/2.*dt
			position[i].append(Posbuff[i])
			#data positioning
			Accbuff[i] = newAccdata[i]
			Velbuff[i] = newVeldata[i]
		Pre_time += dt
		#print dt, Accbuff[0], Velbuff[0], position[0][phase+1]

	return position

def show(data):
	plt.figure()
	for i in range(len(data)):
		plt.subplot(len(data),1,i+1)
		plt.plot(data_array[0], data[i])
	plt.show()


