import csv
import numpy

def loadDataset(filename):
	with open(filename, 'rb') as csvfile:
		lines = csv.reader(csvfile)
		dataset = list(lines)
		numCols = len(dataset[0])
		for x in range(len(dataset)):
		    for y in range(1,numCols): # every col except first, which contains string classification
		        dataset[x][y] = float(dataset[x][y])
		return dataset

def cleanDataAvg(dataset, clean_filename):
	clean_file = open(clean_filename, 'w')
	wr = csv.writer(clean_file)
	second_to_last_ind = len(dataset[0])-2 # one for index, one for second to last
	# set zeros to average of nearest non zero neighbors
	for row in dataset:
		if row[1] == 0:
			num_subseq_zeros = 1;
			next_ind = 2
			while next_ind <= second_to_last_ind and row[next_ind] == 0:
				num_subseq_zeros +=1
				next_ind +=1
			for j in range(1,1+num_subseq_zeros):
				row[j] = row[1+ num_subseq_zeros]
		for i in range(2,second_to_last_ind+1): # not gonna do anything if first or last are 0, also skip string at beginning
			if row[i] == 0:
				num_subseq_zeros = 1;
				next_ind = i+1
				while next_ind <= second_to_last_ind and row[next_ind] == 0:
					num_subseq_zeros +=1
					next_ind +=1
				if row[next_ind] != 0:
					for j in range(i,i+num_subseq_zeros):
						row[j] = (row[i-1] + row[i+num_subseq_zeros]) / 2
		if row[second_to_last_ind+1] == 0:
			num_subseq_zeros = 1;
			last_ind = second_to_last_ind
			while last_ind >= 1 and row[last_ind] == 0:
				num_subseq_zeros +=1
				last_ind -=1
			for j in range(second_to_last_ind+2-num_subseq_zeros, second_to_last_ind+2):
				row[j] = row[second_to_last_ind+1-num_subseq_zeros]
		wr.writerow(row)

def getKNeighbors(k, dataset, testPoint):
	distances = []
	numDims = len(dataset[0]) - 1
	for x in range(len(dataset)):
		classification = dataset[x][0]
		#print dataset[x][1:]
		dist = euclideanDistance(k, dataset[x][1:], testPoint, distances, numDims)
		#print dist
		if not isinstance(type(dist), bool) or dist != False: # is greater than at least one of current distances
			if len(distances) == k:
				#print distances
				dist_list = list(map(lambda x: x[1], distances))
				max_dist_index = dist_list.index(max(dist_list))
				distances[max_dist_index] = (classification, dist)
			else:
				distances.append((classification, dist))
	return distances

def greater_than_all_k(distances, dist):
	for k_distance in enumerate(distances):
		if dist < k_distance:
			return False;
	return True

def euclideanDistance(k, dataPoint, testPoint, distances, numDims):
	distance = 0
	for x in range(numDims):
		#print "dataPoint", type(dataPoint[x])
		#print "testPoint", type(testPoint[x])
		distance += pow((dataPoint[x] - testPoint[x]), 2)
		if len(distances) == k and greater_than_all_k(distances, distance):
			return False
	return distance

def discretizeData(dataset, filename):
	discrete_file = open(filename, 'w')
	wr = csv.writer(discrete_file)

	discretized = []
	for row in dataset:
		name = row[0]
		data = row[1:]
		array_data = numpy.array(data)
		median = numpy.median(array_data)
		thresh = 0.5*numpy.std(array_data)
		new_row = [name]
		for i in range(0,len(data)):
			if data[i] > median + thresh:
				new_row.append(1)
			elif data[i] < median - thresh:
				new_row.append(-1)
			else:
				new_row.append(0)
		discretized.append(new_row)
		wr.writerow(new_row)
	return discretized

def classifyObjects(training_dataset,test_dataset):
	class_dict = {'starbucks' : "Cylinder", 'Toblerone' : "Triangle", 'Rubiks' : 'Cube'}
	num_correct = {'Triangle' : 0, 'Cube' : 0, 'Cylinder' : 0}
	k = 3
	num_test_points = 50.0
	for i in range(0,len(test_dataset)):
		correct_class = class_dict[ test_dataset[i][0] ]
	 	distances = getKNeighbors(k,training_dataset,test_dataset[i][1:])
	 	print distances
	 	#distances.sort(key=lambda x: x[1])
	 	#print "sorted", distances
	 	#print " "
	 	neighbors = {'Triangle' : 0, 'Cube' : 0, 'Cylinder' : 0}
	 	for j in range(0,k):
	 		neighbors[ distances[j][0] ] +=1
	 	guessed_class = max(neighbors, key=neighbors.get)
	 	if neighbors[guessed_class] == 1:
	 		guessed_class = min(distances)[0]

	 	print "I'm guessing that's a" , guessed_class
	 	if guessed_class == correct_class:
	 		num_correct[correct_class] +=1

	for c in num_correct:
		print (num_correct[c] / num_test_points) * 100 , "percent correct at guessing object of type", c


training_dataset = loadDataset('training_data.csv')
cleanDataAvg(training_dataset,'clean_training_data.csv')
discrete_training = discretizeData(training_dataset, 'discrete_training_data.csv')

test_dataset = loadDataset('test_data.csv')
cleanDataAvg(test_dataset,'clean_test_data.csv')
discrete_test = discretizeData(test_dataset, 'discrete_test_data.csv')

classifyObjects(discrete_training, discrete_test)

"""
class_dict = {'starbucks' : "Cylinder", 'Toblerone' : "Triangle", 'Rubiks' : 'Cube'}
num_correct = {'Triangle' : 0, 'Cube' : 0, 'Cylinder' : 0}
k = 2
num_test_points = 50.0
for i in range(0,len(test_dataset)):
	correct_class = class_dict[ test_dataset[i][0] ]
 	distances = getKNeighbors(k,training_dataset,test_dataset[i][1:])
 	print distances
 	#distances.sort(key=lambda x: x[1])
 	#print "sorted", distances
 	#print " "
 	neighbors = {'Triangle' : 0, 'Cube' : 0, 'Cylinder' : 0}
 	for j in range(0,k):
 		neighbors[ distances[j][0] ] +=1
 	guessed_class = max(neighbors, key=neighbors.get)
 	if neighbors[guessed_class] == 1:
 		guessed_class = min(distances)[0]

 	print "I'm guessing that's a" , guessed_class
 	if guessed_class == correct_class:
 		num_correct[correct_class] +=1

for c in num_correct:
	print (num_correct[c] / num_test_points) * 100 , "percent correct at guessing object of type", c
"""


