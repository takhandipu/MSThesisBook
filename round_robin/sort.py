import csv
from collections import Counter
import os
import sys
import operator

def changeToFloat(list):
	retList = []
	for l in list:
		retList.append([i.lstrip(' ') for i in l])
	return retList

folderName = "."
for file in os.listdir(folderName):
    if file.endswith(".csv"):
        sys.stdout = open(folderName+'/'+"sorted"+file, 'w')
        reader = csv.reader(open(folderName+'/'+file), delimiter=",")
        reader = changeToFloat(reader)	
        firstlist = sorted(reader, key=lambda row: float(row[2]), reverse=False)
        secondlist = sorted(firstlist, key=lambda row: float(row[1]), reverse=False)
        for item in secondlist:
            for value in item:
                print value+",",
            print 
