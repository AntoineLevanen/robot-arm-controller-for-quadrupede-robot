import sys, getopt
import os

# folder path
dir_path = r'/home/alevanen/Documents/StageM1/sassa'

# Iterate directory
for file in os.listdir(dir_path):
    # check only STL files
    if file.endswith('.obj'):
        print(file)
        with open(dir_path + "/" + file, 'r') as my_file:
            data = my_file.read()
            data = data.replace("/", "")
        
        with open(dir_path + "/" + file, 'w') as my_file:
            my_file.write(data)