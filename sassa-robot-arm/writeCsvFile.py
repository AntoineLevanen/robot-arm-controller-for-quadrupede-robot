import csv

class CsvWriter:

    def __init__(self, file_path="/home/antoine/Documents/StageM1/joint_angle.csv"):
        self.file_path = file_path
        self.file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.file)

    def writeToCsvFile(self, vector):
        self.writer.writerow(vector)

    def closeFile(self):
        self.file.close()