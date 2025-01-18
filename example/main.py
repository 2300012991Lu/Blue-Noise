import numpy as np
from matplotlib import pyplot as plt

import argparse


#   Type
#       python main.py --file="Noise R=2.000 , n=3 , l=25.txt"          (or any other file)
#           in shell


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('--file', '-f', type=str, help='Your File Here!')
    args = parser.parse_args()

    if args.file is None:
        FileName = "Noise R=1.067 , n=3 , l=25.txt" # Or any other file
    else:
        FileName = args.file
    print(FileName)

    arr = np.loadtxt(FileName, dtype=np.float64).reshape((-1, 3))
    print(arr.shape)

    fig = plt.figure()

    ax = fig.add_subplot(111, projection="3d")
    fig.add_axes(ax)
    ax.scatter(arr[:,0], arr[:,1], arr[:,2], c='b', s=0.5)

    plt.show() # You can drag the axes to rotate
    return


if __name__ == '__main__':
    main()