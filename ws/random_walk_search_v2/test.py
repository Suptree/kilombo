#%%
import csv
import matplotlib.pyplot as plt
f = open('random_walk.dat')
csv_f = csv.reader(f)
for row in csv_f:
  plt.plot(float(row[0]),float(row[1]), marker='.', color='red')
plt.xlim(-500,500)
plt.ylim(-500,500)
plt.show()