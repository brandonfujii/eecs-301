from sklearn.neighbors import BallTree
import numpy as numpy
import csv
import math

NUM_ROWS = 320

def loadDataset(filename):
	with open(filename, 'rb') as csvfile:
		lines = csv.reader(csvfile)
		dataset = list(lines)
		numCols = len(dataset[0])
		for x in range(len(dataset)):
		    for y in range(1,numCols): # every col except first, which contains string classification
		        dataset[x][y] = float(dataset[x][y])
		return dataset

def cleanData2(dataset):
	row = 0
	while row < len(dataset):
		#print "range", len(dataset[row][1:])
		for col in range(len(dataset[row][1:])):
			#print row
			if dataset[row][col] == 0:
				try:
					dataset = dataset[:row] + dataset[row + 1:]
				except IndexError:
					dataset = dataset[:row]
				break
			else:
				row += 1

	return dataset

def cleanData(dataset):
	cleanOne = []
	for row in dataset:
		if 0 not in row[1:]:
			#print row[0], row[1]
			cleanOne.append([row[0]] + normalizeDistances(row[1:]))
	return cleanOne

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

def normalizeDistances(point):
	normalized = []
	max_val = max(point)
	min_val = min(point)
	range_val = max_val - min_val
	for component in point:
		normalized.append((component - min_val) / range_val)
	return normalized



dataset = []
dataset = loadDataset('testdata.csv')
cleanDataset = cleanData(dataset)
with open('testpoint.csv', 'rb') as csvfile:
 	line = csv.reader(csvfile)
 	testPoint = list(line)
 	numCols = len(testPoint[0])
 	for y in range(0, numCols):
 		testPoint[0][y] = float(testPoint[0][y])
 	norm_test_point = normalizeDistances(testPoint[0])
 	distances = getKNeighbors(7,cleanDataset,normalizeDistances(testPoint[0]))
 	print distances

