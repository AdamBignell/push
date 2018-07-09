# Generates a figure indicating the distance of each box to the goal over time
import math
import numpy as np
import matplotlib.pyplot as plt

def myreadlines(f, newline):
  buf = ""
  while True:
    while newline in buf:
      pos = buf.index(newline)
      yield buf[:pos]
      buf = buf[pos + len(newline):]
    chunk = f.read(4096)
    if not chunk:
      yield buf
      break
    buf += chunk

perfDataStr = "_PerfData.txt"
prefixStr = "Results/100 Robots 1000 Boxes/circle/circle"

fileName = prefixStr + str(0) + perfDataStr
with open(fileName) as fHead:
  line = ""
  while ("#Boxes" not in line):
    line = fHead.readline()
  numBoxes = int(line.split()[1])
  while ("Maxsteps:" not in line):
    line = fHead.readline()
  numSteps = int(line.split()[1])
  interval = numSteps / 100

# Need 100 buckets each with 20 avg lengths for ALL tests
# change 20 to n where n is the amount of trials
# this shows the change over time
avgDists = [[] for j in range(101)]
avgOutsideDists = [[] for j in range(101)]
allDists = [[] for j in range(101)]
stepNumbers = [j*interval + 1 for j in range(101)]

allSteps = []

for i in range(0, 20):
    del allSteps[:]
    fileName = prefixStr + str(i) + perfDataStr
    with open(fileName) as f:
      for line in myreadlines(f, "$"):
        allSteps.append(line)
      for j in range (1, len(allSteps)):
            stepInfo = allSteps[j].split("!")
            if (len(stepInfo) <= 1):
                continue
            bucket = int(math.floor(int(stepInfo[1].split()[1])/1000))
            # This isn't strictly needed right now
            allThisDists = map(float, stepInfo[2].strip().split("\n"))
            allDists[bucket].extend(allThisDists)
            avgDist = stepInfo[3]
            avgOutsideDists[bucket].append(float(avgDist.strip().split("\n")[3].split()[1]))
            avgDists[bucket].append(float(avgDist.strip().split("\n")[2].split()[1]))

avgAvgDists = []
stdDevs = []

# This gives 1000 lists of items 101 items long (one box dist for each step)
perBoxDists = [[] for i in range(1000)]
for i in range(1000):
  for j in range(101):
    perBoxDists[i].append(allDists[j][i]);

for i in range(0, 101):
    print(sum(avgDists[i]))
    print(float(len(avgDists[i])))
    print(sum(avgDists[i]) / float(len(avgDists[i])))
    avgAvgDists.append(sum(avgDists[i]) / float(len(avgDists[i])))
    stdDevs.append(np.std(avgDists[i]))

print(avgAvgDists)
#lines = plt.plot(stepNumbers, avgAvgDists)
lines = plt.errorbar(stepNumbers, avgAvgDists, stdDevs, marker='s', mfc='red',
      mec='red', ms=0.5, mew=0.5)
plt.setp(lines, color='r', linewidth=0.5)
plt.grid()
ax = plt.gca()
#ax.set_yticks(np.arange(0, math.ceil(max(avgAvgDists)), 0.50))
plt.xlabel('World Steps')
plt.ylabel('Mean Box Distance from Goal Shape')
plt.title("Convergence of Boxes Towards Goal Circle")
plt.show()
