import numpy as np 
import matplotlib.pyplot as plt 
import csv
import uda_matrix as umx

data_array = []

with open("output.txt", 'r') as file:
	column = zip(*[line for line in csv.reader(file, dialect="excel-tab")])
	for i in range(7):
		data_array.append(np.array(column[i], dtype='float'))
	for j in range(len(column[0])):
		data_array[0][j] /= 1000.
		data_array[1][j] *= 0.001197509765625
		data_array[2][j] *= 0.001197509765625
		data_array[3][j] *= 0.001197509765625
		data_array[4][j] *= 0.00106526421440972222222
		data_array[5][j] *= 0.00106526421440972222222
		data_array[6][j] *= 0.00106526421440972222222


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

def graph(dataAxis, timeAxis=data_array[0]):
	plt.figure()
	for i in range(len(dataAxis)):
		plt.subplot(len(dataAxis),1,i+1)
		plt.plot(timeAxis, dataAxis)
	plt.show()


if __name__=='__main__':
	#graph(raw_Accelero_position())
	QuaternionX = []
	x = umx.matrix([[1.,0.,0.,0.]]).transpose()
	P = umx.matrix([[]]); P.diagonal(4,1000.)
	z = [1., 0., 0., 0.]
	dt = .2
	#xP[0.,0.]
	for phase in range(len(column[0])):
		# w: gyro, z: measured Quaternion
		w = [float(column[4][phase]), float(column[5][phase]), float(column[6][phase])]
		x,P = umx.kalman_filter(x, P, w, dt)
		QuaternionX.append(x.transpose().normalize()) #data collecting
		print QuaternionX[phase]
	#print QuaternionX
