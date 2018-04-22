"""
@file hough_lines.py
@brief This program demonstrates line finding with the Hough transform
https://docs.opencv.org/3.4.0/d9/db0/tutorial_hough_lines.html
"""
import sys
import math
import numpy as np

from skimage import io
from skimage.util import invert
from skimage.filters import threshold_otsu
from skimage.transform import probabilistic_hough_line
from skimage.morphology import skeletonize_3d

import matplotlib.pyplot as plt
from matplotlib import cm

def main(argv):
    
    # Loads an image
    image = io.imread(argv[0], True)
    
    binary = image > threshold_otsu(image)
    skel = skeletonize_3d(invert(binary))
    
    # Copy edges to the images that will display the results in BGR
    lines = probabilistic_hough_line(skel, threshold=5, line_length=20,
                                 line_gap=50)

    plot(image, skel, lines)

def plot(raw, processed, lines):
    fig, axes = plt.subplots(1, 3, figsize=(20, 6), sharex=True, sharey=True)
    ax = axes.ravel()

    ax[0].imshow(raw, cmap=cm.gray)
    ax[0].set_title('Input image')

    ax[1].imshow(processed, cmap=cm.gray)
    ax[1].set_title('Processed image')

    ax[2].imshow(processed * 0)
    for line in lines:
        p0, p1 = line
        ax[2].plot((p0[0], p1[0]), (p0[1], p1[1]))
    ax[2].set_xlim((0, raw.shape[1]))
    ax[2].set_ylim((raw.shape[0], 0))
    ax[2].set_title('Probabilistic Hough')

    for a in ax:
        a.set_axis_off()

    plt.tight_layout()
    plt.show()
    
if __name__ == "__main__":
    main(sys.argv[1:])