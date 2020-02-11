# python implementation of << Adaptive Logarithmic Mapping For Displaying High Contrast Scenes >>
# created: LIU Jianbang @ RPAI Lab, 2020.02.11

import cv2
import numpy as np


def tone_mapper(rgb, factor=85, base=0.95):
    """ adjust the brightness of the input rgb image
    :param rgb: rgb image input
    :param factor: the illumination factor, the larger, the brighter
    :param base: the dynamic range scale
    :return: processed rgb image
    """
    XYZ = cv2.cvtColor(rgb, cv2.COLOR_RGB2XYZ) / 255.0

    # normalize to 0 - 1, and convert to XYZ color space
    # rgb = rgb / 255.0
    # XYZ = np.zeros(rgb.shape, dtype=np.float)
    # XYZ[:, :, 0] = 0.4124 * rgb[:, :, 0] + 0.3576 * rgb[:, :, 1] + 0.1805 * rgb[:, :, 2]
    # XYZ[:, :, 1] = 0.2126 * rgb[:, :, 0] + 0.7152 * rgb[:, :, 1] + 0.0722 * rgb[:, :, 2]
    # XYZ[:, :, 2] = 0.0193 * rgb[:, :, 0] + 0.1192 * rgb[:, :, 1] + 0.9505 * rgb[:, :, 2]

    Y = XYZ[:, :, 1]
    lwmax = max(-1, Y.max())

    xyz_sum = XYZ[:, :, 0] + XYZ[:, :, 1] + XYZ[:, :, 2] + np.finfo(float).eps
    xx = XYZ[:, :, 0] / xyz_sum
    yy = XYZ[:, :, 1] / xyz_sum

    XYZ_proc = np.zeros(rgb.shape)
    Y_proc = 0.01 * factor * np.log(Y + 1 + np.finfo(float).eps) / \
             np.log(2 + 8.0 * np.power((Y / lwmax), np.log(base) / np.log(0.5))) / \
             np.log10(lwmax + 1)
    XYZ_proc[:, :, 0] = Y_proc / yy * xx
    XYZ_proc[:, :, 1] = Y_proc
    XYZ_proc[:, :, 2] = Y_proc / yy * (1 - xx - yy)

    # convert to RGB color space
    rgb_proc = np.zeros(rgb.shape)
    # rgb_proc = cv2.cvtColor(XYZ_proc.astype(np.uint8), cv2.COLOR_XYZ2RGB)
    rgb_proc[:, :, 0] =   3.2410 * XYZ_proc[:, :, 0] - 1.5374 * XYZ_proc[:, :, 1] - 0.4986 * XYZ_proc[:, :, 2]
    rgb_proc[:, :, 1] = - 0.9692 * XYZ_proc[:, :, 0] + 1.8760 * XYZ_proc[:, :, 1] + 0.0416 * XYZ_proc[:, :, 2]
    rgb_proc[:, :, 2] =   0.0556 * XYZ_proc[:, :, 0] - 0.2040 * XYZ_proc[:, :, 1] + 1.0570 * XYZ_proc[:, :, 2]

    # transform the color value
    rgb_proc[rgb_proc < 0] = 0
    rgb_proc[rgb_proc > 1] = 1
    mask1 = rgb_proc <= 0.05
    mask2 = np.logical_not(rgb_proc)
    rgb_proc[mask1] = rgb_proc[mask1] * 2.64
    rgb_proc[mask2] = 1.099 * np.power(rgb_proc[mask2], 0.9 / 2.2) - 0.099

    # scale to 0 - 255
    rgb_proc = (rgb_proc * 255.0).astype(np.uint8)

    return rgb_proc


if __name__ == "__main__":
    # load the test image file
    raw_img = cv2.imread("/home/henry/catkin_ws/src/angular_servo/data/imgs_new_4/6.jpg")

    # call the tone_mapper
    proc_img = tone_mapper(raw_img, 90, 0.95)

    # display the images
    cv2.imshow("raw", raw_img)
    cv2.imshow("proc", proc_img)
    cv2.waitKey(0)
