from matplotlib import pyplot as plt
import sys
from skimage.morphology import skeletonize_3d
from skimage import io
from skimage.feature import corner_harris, corner_subpix, corner_peaks
from skimage.filters import threshold_otsu, gaussian
from skimage.util import invert

def main(argv):
    image = io.imread(argv[0], True)
    smooth = gaussian(image, sigma=4, mode='reflect')   
    binary = image > threshold_otsu(smooth)

    
    skeleton = skeletonize_3d(invert(binary))    

    coords = corner_peaks(corner_harris(binary, k=0.2, sigma=4), min_distance=5)
    coords_subpix = corner_subpix(binary, coords, window_size=13)

    fig, ax = plt.subplots()
    ax.imshow(binary, interpolation='nearest', cmap=plt.cm.gray)
    ax.plot(coords[:, 1], coords[:, 0], '+r', markersize=15)
    # ax.plot(coords_subpix[:, 1], coords_subpix[:, 0], '+r', markersize=15)
    ax.axis((0, 600, 600, 0))
    plt.show()

if __name__ == "__main__":
    main(sys.argv[1:])