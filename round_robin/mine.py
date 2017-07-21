import csv
from collections import Counter
import os
import sys

def average_column (csv_filepath):
    column_totals = Counter()
    with open(csv_filepath,"rb") as f:
        reader = csv.reader(f)
        row_count = 0.0
        for row in reader:
            for column_idx, column_value in enumerate(row):
                try:
                    n = float(column_value)
                    column_totals[column_idx] += n
                except ValueError:
                    # print "Error -- ({}) Column({}) could not be converted to float!".format(column_value, column_idx)         
                    pass
            row_count += 1.0            

    # row_count is now 1 too many so decrement it back down
    # row_count -= 1.0

    # make sure column index keys are in order
    column_indexes = column_totals.keys()
    column_indexes.sort()

    # calculate per column averages using a list comprehension
    averages = [column_totals[idx]/row_count for idx in column_indexes]
    return averages

folderName = "."
for folder in os.listdir(folderName):
     sys.stdout = open(folderName+'/'+folder+'.csv', 'w')
     for file in os.listdir(folderName+'/'+folder):
	    # if file.endswith(".csv"):
		ls = file.split("_")        
		print ''+ls[1]+',',
		averages = average_column(folderName+'/'+folder+'/'+file)
		for value in averages:
			print ''+`value`+',',
			pass
		print 
