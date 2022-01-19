import math
import random
import numpy as np
import matplotlib.pyplot as plt


def calc_angle(x1,x2,y1,y2):
 	n1 = x2 - x1
 	n2 = y2 - y1
	angle = math.atan2(n1,n2)

	if angle < 0.0:
		angle += 2*math.pi

	return math.degrees(angle)

for i in range(100):
	plt.clf()
	x1 = random.randint(1,4)
	y1 = random.randint(-4,4)

	x2 = random.randint(1,4)
	y2 = random.randint(-4,4)

	plt.plot([x1,x2], [y1, y2], 'ro-')
	plt.annotate("Finish", (x2, y2))
	plt.plot(x2, y2, 'bo')
	plt.plot(0, 0, 'ko')

	# Calculate angle:
	THETA = calc_angle(x1,x2,y1,y2)
	plt.annotate(str(THETA), (x1, y1))

	plt.xticks(np.arange(0, 4+1, 1.0))

	plt.xlim(0, 4)
	plt.ylim(-4, 4)
	plt.grid()
	plt.show()
