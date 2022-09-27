import sys
import cv2
import numpy as np
import time
import os
import sys
# Use the keypoints to stitch the images


def get_stitched_image(img1, img2, H):

    # Get width and height of input images
    w1, h1 = img1.shape[:2]
    w2, h2 = img2.shape[:2]

    # Get the canvas dimesions
    img1_dims = np.float32(
        [[0, 0], [0, w1], [h1, w1], [h1, 0]]).reshape(-1, 1, 2)
    img2_dims_temp = np.float32(
        [[0, 0], [0, w2], [h2, w2], [h2, 0]]).reshape(-1, 1, 2)

    # Get relative perspective of second image
    img2_dims = cv2.perspectiveTransform(img2_dims_temp, H)

    # Resulting dimensions
    result_dims = np.concatenate((img1_dims, img2_dims), axis=0)

    # Getting images together
    # Calculate dimensions of match points
    [x_min, y_min] = np.int32(result_dims.min(axis=0).ravel()-0.5)
    [x_max, y_max] = np.int32(result_dims.max(axis=0).ravel()+0.5)

    # Create output array after affine transformation
    transform_dist = [-x_min, -y_min]
    transform_array = np.array([[1, 0, transform_dist[0]],
                                [0, 1, transform_dist[1]],
                                [0, 0, 1]])

    # Warp images to get the resulting image
    result_img = cv2.warpPerspective(img2, transform_array.dot(H),
                                     (x_max-x_min, y_max-y_min))
    result_img[transform_dist[1]:w1+transform_dist[1],
               transform_dist[0]:h1+transform_dist[0]] = img1

    # Return the result
    return result_img

# Find SIFT and return Homography Matrix


def get_sift_homography(img1, img2):

    # Initialize SIFT
    sift = cv2.xfeatures2d.SIFT_create()  # opencv-python-contirb == 3.4.2.17

    # Extract keypoints and descriptors
    k1, d1 = sift.detectAndCompute(img1, None)
    k2, d2 = sift.detectAndCompute(img2, None)

    # Bruteforce matcher on the descriptors
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(d1, d2, k=2)

    # Make sure that the matches are good
    verify_ratio = 0.8  # Source: stackoverflow
    verified_matches = []
    for m1, m2 in matches:
        # Add to array only if it's a good match
        if m1.distance < 0.8*m2.distance:
            verified_matches.append(m1)

    # Mimnum number of matches
    min_matches = 8
    if len(verified_matches) > min_matches:

        # Array to store matching points
        img1_pts = []
        img2_pts = []

        # Add matching points to array
        for match in verified_matches:
            img1_pts.append(k1[match.queryIdx].pt)
            img2_pts.append(k2[match.trainIdx].pt)
        img1_pts = np.float32(img1_pts).reshape(-1, 1, 2)
        img2_pts = np.float32(img2_pts).reshape(-1, 1, 2)

        # Compute homography matrix
        H, mask = cv2.findHomography(img1_pts, img2_pts, cv2.RANSAC, 5.0)
        return H
    else:
        print('Error: Not enough matches')
        exit()

# Equalize Histogram of Color Images
def equalize_histogram_color(img):
    img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    img_yuv[:, :, 0] = cv2.equalizeHist(img_yuv[:, :, 0])
    img = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
    return img

def main():
    # Get input set of images
    img1 = cv2.imread("/home/ubuntu/workspace/data/video/sineva/00650.jpg") # 左
    img2 = cv2.imread("/home/ubuntu/workspace/data/video/sineva/00556.jpg") # 中
    img3 = cv2.imread("/home/ubuntu/workspace/data/video/sineva/00890.jpg") # 右
    img4 = cv2.imread("/home/ubuntu/workspace/data/video/sineva/00444.jpg") # 上
    img5 = cv2.imread("/home/ubuntu/workspace/data/video/sineva/00002.jpg") # 下

    # 均衡直方图
    # img1=equalize_histogram_color(img1)
    # img2=equalize_histogram_color(img2)
    # img3=equalize_histogram_color(img3)
    # img4=equalize_histogram_color(img4)
    # img5=equalize_histogram_color(img5)

    # Show input images
    # input_images=np.hstack((img1,img2))
    # cv2.imshow('Input Images',input_images)

    start_time1 = time.time() # 计时
    # Use SIFT to find keypoints and return homography matrix
    # H1 = get_sift_homography(img1, img2)
    H1 = np.array([[ 1.57462985e+00, -3.22396178e-02, -5.57393616e+02],
                                [ 1.84166714e-01,  1.30593069e+00, -1.77240359e+02],
                                [ 3.03417777e-04, -1.86729659e-05,  1.00000000e+00]],
    			)
    # print("H1:", H1)

    # Stitch the images together using homography matrix
    result_image1 = get_stitched_image(img2, img1, H1)

    stop_time1 = time.time()
    cost = (stop_time1 - start_time1) * 1000.0
    print("%s cost %s microsecond" % (os.path.basename(sys.argv[0]), cost))

    result_image_name1 = 'result1.jpg'
    # cv2.imwrite(result_image_name1, result_image1)

    start_time2 = time.time()
    # H2 = get_sift_homography(img3, img2)
    H2 = np.array([[ 5.61953291e-01, -1.74351428e-02,  4.25532617e+02],
                                [-1.41959481e-01,  7.80231724e-01,  1.62456182e+02],
                                [-2.17259372e-04, -3.59954740e-05,  1.00000000e+00]],
    			)
    # print("H2:", H2)
    result_image2 = get_stitched_image(img2, img3, H2)
    stop_time2 = time.time()
    cost = (stop_time2 - start_time2) * 1000.0
    print("%s cost %s microsecond" % (os.path.basename(sys.argv[0]), cost))
    result_image_name2 = 'result2.jpg'
    # cv2.imwrite(result_image_name2, result_image2)

    start_time3 = time.time()
    # H3 = get_sift_homography(result_image1, result_image2)
    H3 = np.array([[ 9.99991904e-01,  8.94337413e-07, -6.03994796e+02],
                                [-4.85878569e-06,  9.99996094e-01,  1.20068058e+01],
                                [-5.09084775e-09,  5.49724026e-09,  1.00000000e+00]],
    			)
    # print("H3:",H3)
    result_image3 = get_stitched_image(result_image2, result_image1, H3)
    stop_time3 = time.time()
    cost = (stop_time3 - start_time3) * 1000.0
    print("%s cost %s microsecond" % (os.path.basename(sys.argv[0]), cost))

    # Write the result to the same directory
    result_image_name3 = 'result3.jpg'
    cv2.imwrite(result_image_name3, result_image3)  # 左中右

    start_time4 = time.time()
    # H4 = get_sift_homography(img4, img2)
    H4 = np.array([[ 1.20588287e+00,  3.41382700e-01, -2.03648509e+02],
                                [-8.21628031e-03,  1.36379288e+00, -4.46022748e+02],
                                [-1.93221605e-05,  3.49510706e-04,  1.00000000e+00]],
    			)
    # print("H4:",H4)
    result_image4 = get_stitched_image(img2, img4, H4)
    stop_time4 = time.time()
    cost = (stop_time4 - start_time4) * 1000.0
    print("%s cost %s microsecond" % (os.path.basename(sys.argv[0]), cost))
    result_image_name4 = 'result4.jpg'
    # cv2.imwrite(result_image_name4, result_image4)

    # H5 = get_sift_homography(result_image3, result_image4)
    # print("H5:",H5)
    # result_image5 = get_stitched_image(result_image4, result_image3, H5)
    # result_image_name5 = 'result5.jpg'
    # cv2.imwrite(result_image_name5, result_image5)

    start_time5 = time.time()
    # H6 = get_sift_homography(img5, img2)
    H6 = np.array([[ 8.35195467e-01, -2.01723047e-01,  1.59487207e+02],
                                [-3.44895651e-02,  7.27874712e-01,  3.10933089e+02],
                                [-3.80282488e-05, -2.15717757e-04,  1.00000000e+00]],
    			)
    # print("H6:",H6)
    result_image6 = get_stitched_image(img2, img5, H6)
    stop_time5 = time.time()
    cost = (stop_time5 - start_time5) * 1000.0
    print("%s cost %s microsecond" % (os.path.basename(sys.argv[0]), cost))
    result_image_name6 = 'result6.jpg'
    # cv2.imwrite(result_image_name6, result_image6)

    start_time6 = time.time()
    # H5 = get_sift_homography(result_image4, result_image6)
    H5 = np.array([[ 1.00000142e+00, -5.22803298e-06, -1.27997968e+02],
                                [-4.77723345e-07,  9.99997667e-01, -4.79997713e+02],
                                [ 1.67038147e-09, -3.40443879e-09,  1.00000000e+00]],
    			)
    # print("H5:",H5)
    result_image5 = get_stitched_image(result_image6, result_image4, H5)
    stop_time6 = time.time()
    cost = (stop_time6 - start_time6) * 1000.0
    print("%s cost %s microsecond" % (os.path.basename(sys.argv[0]), cost))
    result_image_name5 = 'result5.jpg'
    cv2.imwrite(result_image_name5, result_image5)

    # H7 = get_sift_homography(result_image3, result_image5)
    # print("H7:",H7)
    # result_image7 = get_stitched_image(result_image5, result_image3, H7)
    # result_image_name7 = 'result7.jpg'
    # cv2.imwrite(result_image_name7, result_image7)

    stop_time7 = time.time()
    cost = (stop_time7 - start_time1) * 1000.0
    print("%s cost %s microsecond" % (os.path.basename(sys.argv[0]), cost))

    start_time8 = time.time()
    stitcher = cv2.createStitcher(True)    # 老的OpenCV版本，用这一个
    # stitcher = cv2.Stitcher.create(cv2.Stitcher_PANORAMA)  # OpenCV4
    # img7 = cv2.imread("/home/ubuntu/workspace/gitee/tools/stitch/result3.jpg")
    # img8 = cv2.imread("/home/ubuntu/workspace/gitee/tools/stitch/result5.jpg")
    # (status, pano) = stitcher.stitch((img7, img8))
    (status, pano) = stitcher.stitch((result_image3, result_image5))
    if status != cv2.Stitcher_OK:
        print("不能拼接图片, error code = %d" % status)
        sys.exit(-1)
    # print("拼接成功.")
    # cv2.imshow('pano', pano)
    result_image_name8 = 'result8.jpg'
    cv2.imwrite(result_image_name8, pano)

    stop_time8 = time.time()
    cost = (stop_time8 - start_time8) * 1000.0
    print("%s cost %s microsecond" % (os.path.basename(sys.argv[0]), cost))

    # Show the resulting image
    # cv2.imshow('Result',result_image2)
    # cv2.waitKey()


# Call main function
if __name__ == '__main__':
    main()
