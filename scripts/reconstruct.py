import cv2
import sys
import numpy as np
import subprocess


ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''


def write_pcd(fn, verts, colors):
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])

    with open('/tmp/%s.ply' % fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')

    subprocess.call('/usr/local/pcl/build/bin/pcl_converter /tmp/cloud.ply /tmp/cloud.pcd',
                    shell=True)


def main(leftpath, rightpath, filename):

    imgL = cv2.imread(leftpath)
    imgR = cv2.imread(rightpath)

    imgL = cv2.pyrDown(imgL)
    imgR = cv2.pyrDown(imgR)

    window_size = 3
    min_disp = 16
    num_disp = 112 - min_disp

    stereo = cv2.StereoSGBM_create(minDisparity=min_disp,
                                   numDisparities=num_disp,
                                   blockSize=5,
                                   P1=8 * 3 * window_size ** 2,
                                   P2=32 * 3 * window_size ** 2,
                                   disp12MaxDiff=1,
                                   uniquenessRatio=10,
                                   speckleWindowSize=100,
                                   speckleRange=32
                                   )

    disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0

    h, w = imgL.shape[:2]
    f = 0.8 * w  # guess for focal length
    Q = np.float32([[1, 0, 0, -0.5 * w],
                    [0, -1, 0, 0.5 * h],  # turn points 180 deg around x-axis,
                    [0, 0, 0, -f],  # so that y-axis looks up
                    [0, 0, 1, 0]])

    points = cv2.reprojectImageTo3D(disp, Q)
    colors = cv2.cvtColor(imgL, cv2.COLOR_BGR2RGB)
    mask = disp > disp.min()
    out_points = points[mask]
    out_colors = colors[mask]

    write_pcd(filename, out_points, out_colors)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3])
