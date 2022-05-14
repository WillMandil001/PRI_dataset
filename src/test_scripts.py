import os
import numpy as np
from PIL import Image


data_new = np.load("data/color_images.npy")
data_old = np.load("data_old/color_images.npy")
data_oldest = np.load("data_oldest/color_images.npy")

print(data_new.shape)
print(data_old.shape)

import matplotlib.pyplot as plt

data_new = Image.fromarray(data_new[20], 'RGB')
data_old = Image.fromarray(data_old[20], 'RGB')
data_oldest = Image.fromarray(data_oldest[20], 'RGB')

plt.figure()

f, axarr = plt.subplots(3,1)

axarr[0].imshow(data_new)
axarr[1].imshow(data_old)
axarr[2].imshow(data_oldest)

plt.show()