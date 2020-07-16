'''
Saves each topic in a bagfile as a csv.

Accepts a filename as an optional argument. Operates on all bagfiles in current directory if no argument provided

Usage1 (for one bag file):
	python bag2csv.py filename.bag
Usage 2 (for all bag files in current directory):
	python bag2csv.py

Written by Nick Speal in May 2013 at McGill University's Aerospace Mechatronics Laboratory
www.speal.ca. Supervised by Professor Inna Sharf, Professor Meyer Nahon 

Additional edits: Ryan Cosner 
'''

import rosbag, sys, csv
import time
import string
import os 
import shutil 

# Case Structure: 
#	Determine the number of inputs
#	- >2 inputs: throw error
#	- 2 inputs: convert second argument to a csv
#	- 1 input: convert all bag files to 
if (len(sys.argv) > 2):
	# Input Error
	print "Invalid number of arguments:   " + str(len(sys.argv))
	print "There should be 2: 'bag2csv.py' and 'bagName'"
	print "or just 1  : 'bag2csv.py'"
	sys.exit(1)
elif (len(sys.argv) == 2):
	# Find 1 bag file given
	listOfBagFiles = [sys.argv[1]]
	numberOfFiles = str(len(listOfBagFiles))
	print "Reading only 1 bagfile: " + str(listOfBagFiles[0])
elif (len(sys.argv) == 1):
	# Find all bag files
	listOfBagFiles = [f for f in os.listdir(".") if f[-4:] == ".bag"]	
	numberOfFiles = str(len(listOfBagFiles))
	print "Reading all " + numberOfFiles + " bagfiles in current directory: \n"
	for f in listOfBagFiles:
		print f
else:
	# Exception
	print "Bad argument(s): " + str(sys.argv)
	sys.exit(1)


count = 0
for bagFile in listOfBagFiles:
	count += 1
	print "Reading file " + str(count) + " of  " + numberOfFiles + ": " + bagFile
	# Access bagFile
	bag = rosbag.Bag(bagFile)
	bagContents = bag.read_messages()
	bagName = bag.filename

	# Create a new directory for each .bag 
	folder = string.rstrip(bagName, ".bag")
	try:	# Test file existence
		os.makedirs(folder)
	except:
		pass
	shutil.copyfile(bagName, folder + '/' + bagName)


	# Get list of topics from the bag
	listOfTopics = []
	for topic, msg, t in bagContents:
		if topic not in listOfTopics:
			listOfTopics.append(topic)


	for topicName in listOfTopics:
		# Create a new .csv file for each topic in .bag
		filename = folder + '/' + folder + '.csv' #folder + '/' + string.replace(topicName, '/', '_slash_') + '.csv'
		with open(filename, 'w+') as csvfile:
			filewriter = csv.writer(csvfile, delimiter = ',')
			firstIteration = True	# Allows header row

			processing_iter = 0 
			print '\t Number of messages parsed: '
			for subtopic, msg, t in bag.read_messages(topicName):	
				# For each instant in time that has data for topicName
				# parse data from this instant, which is of the form of 
				# multiple lines of "Name: value\n"
				#	- put it in the form of a list of 2-element lists
				if processing_iter%100==0: 
					print '\t\t'+ str(processing_iter)

				msgString = str(msg)
				msgList = string.split(msgString, '\n')
				instantaneousListOfData = []
				for nameValuePair in msgList:
					splitPair = string.split(nameValuePair, ':')
					for i in range(len(splitPair)):	#should be 0 to 1
						splitPair[i] = string.strip(splitPair[i])
					instantaneousListOfData.append(splitPair)
				# Write the first row from the first element of each pair
				if firstIteration:	# Create Header
					headers = ["rosbagTimestamp"]	# First column header
					for pair in instantaneousListOfData:
						headers.append(pair[0])
					filewriter.writerow(headers)
					firstIteration = False
				# Write the value from each pair to the file
				values = [str(t)]	# First column will have rosbag timestamp
				for pair in instantaneousListOfData:
					values.append(pair[1])
				filewriter.writerow(values)

				processing_iter+=1 
	os.remove(bag.filename)
	bag.close()


print "Done reading all " + numberOfFiles + " bag files."