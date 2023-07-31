import sys, getopt
import os

"""
Remove the '//' charater in .OBJ file after conversion from .STL ( in the face definition part in the file)
"""

# folder path
dir_path = os.path.abspath("sassa")

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