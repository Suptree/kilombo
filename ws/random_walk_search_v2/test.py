import csv
import matplotlib.pyplot as plt
f = open('random_walk.dat')
csv_f = csv.reader(f)
for row in csv_f:
  plt.scatter(row[0],row[1])
