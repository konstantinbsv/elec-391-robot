import cv2
import os
from cv2 import aruco
import numpy as np
import math
import threading

PIXEL_SCALE_FACTOR = int(240 / 10)
FONT = cv2.FONT_HERSHEY_DUPLEX
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_100)

win_name = 'Robot Workspace'

# coordinate variables
latest_coordinates = [0, 0]
coord = []
latest_angles = [0, 0]

# robot physical dimension constants
l1 = l4 = 6.5
l2 = l3 = 8.5
l5 = 4
yg = 3.5
plate_y = 3
plate_x = 4

VIDEO_INPUT = True
# initialize video if enables
cap = None
if VIDEO_INPUT:
    cap = cv2.VideoCapture(1)


def nothing(val):
    pass


# sliders
def create_sliders():
    # cv2.createTrackbar('x', win_name, -6, 4, nothing)
    # cv2.createTrackbar('y', win_name, 0, 15, nothing)
    cv2.createTrackbar('Angle', win_name, 0, 180, nothing)


def draw_arms():
    global latest_coordinates, coord
    cv2.namedWindow(win_name, cv2.WINDOW_AUTOSIZE)
    create_sliders()
    frame = get_frame()
    drawing = np.zeros(frame.shape, np.uint8)
    frame_shape = frame.shape
    x_right = int(frame_shape[1])  # number of columns
    y_bottom = int(frame_shape[0])  # number of rows
    x_center = int(x_right / 2 + l4 / 2 * PIXEL_SCALE_FACTOR)  # center of x-axis
    point_m1 = (x_center, y_bottom)  # location of motor 1
    point_m2 = (int(x_center - l4 * PIXEL_SCALE_FACTOR), y_bottom)  # location of motor 2
    text_x = x_right - 165
    bin_pos = (int((point_m1[0] + point_m2[0])/2), int(y_bottom - 5*PIXEL_SCALE_FACTOR))

    # init aruco detection
    ret, mtx, dist, rvecs, tvecs = camera_calibration()

    while True:
        # get newest frame and coordinates

        if len(coord) > 0:
            frame = get_frame()
            drawing = np.zeros(frame.shape, np.uint8)     # create a blank image
            latest_coord = coord.pop()
            x_e = latest_coord[0]
            y_g = latest_coord[1]
            y_e = y_g - yg
            q1 = latest_coord[2]
            q5 = latest_coord[3]
            qw = latest_coord[4]
            point_q2 = (int(point_m1[0] + l1 * math.cos(q1) * PIXEL_SCALE_FACTOR),
                        int(point_m1[1] - l1 * math.sin(q1) * PIXEL_SCALE_FACTOR))
            point_q4 = (int(point_m2[0] + l4 * math.cos(q5) * PIXEL_SCALE_FACTOR),
                        int(point_m2[1] - l4 * math.sin(q5) * PIXEL_SCALE_FACTOR))
            point_w = (int(x_center + x_e * PIXEL_SCALE_FACTOR),
                       int(y_bottom - y_e * PIXEL_SCALE_FACTOR))

            # gripper calculations
            x_q4 = -l5 + l4*math.cos(q5)
            delta_x = x_e - x_q4
            if delta_x < l3:
                q4_p = math.acos(delta_x/l3)
            else:
                q4_p = 0.15
            print(f'qw = {qw}')
            print(f'x_q4 = {x_q4}')
            point_g = (int(point_w[0] + yg*math.cos(qw + q4_p)*PIXEL_SCALE_FACTOR),
                       int(point_w[1] - yg*math.sin(qw + q4_p)*PIXEL_SCALE_FACTOR))

            # plate coord
            # plate_1a = (int(point_g[0] - plate_x/2*PIXEL_SCALE_FACTOR),
            #             int(point_g[1] - plate_y/2*PIXEL_SCALE_FACTOR))
            # plate_1b = (int(point_g[0] - plate_x/2*PIXEL_SCALE_FACTOR),
            #             int(point_g[1] + plate_y/2*PIXEL_SCALE_FACTOR))

            # draw lines
            cv2.line(drawing, point_m1, point_q2, (255, 0, 0), 3)   # draw line from m1 to q2
            cv2.line(drawing, point_q2, point_w, (0, 255, 0), 3)    # draw line from q2 to q3
            cv2.line(drawing, point_m2, point_q4, (0, 0, 255), 3)   # draw line from m2 to q4
            cv2.line(drawing, point_q4, point_w, (0, 255, 255), 3)  # draw line from q4 to q3
            cv2.line(drawing, point_w, point_g, (255, 0, 255), 3)   # draw line from q3 to gripper
            # cv2.line(drawing, plate_1a, plate_1b, (255, 0, 255), 3)   # draw line from q3 to gripper

            # update readout
            text_color = (40, 40, 40)
            cv2.putText(drawing, f'x = {x_e:.1f}cm', (text_x, y_bottom - 175), FONT, 0.75, text_color, 1, cv2.LINE_AA)
            cv2.putText(drawing, f'y = {y_g:.1f}cm', (text_x, y_bottom - 140), FONT, 0.75, text_color, 1, cv2.LINE_AA)
            cv2.putText(drawing, f'q1 = {q1:.2f}rad', (text_x, y_bottom - 105), FONT, 0.75, text_color, 1, cv2.LINE_AA)
            cv2.putText(drawing, f'q5 = {q5:.2f}rad', (text_x, y_bottom - 70), FONT, 0.75, text_color, 1, cv2.LINE_AA)

        # perform Hough Circle detection
        circles = detect_circles(frame)
        if circles is not None:
            circles = np.uint16(np.round(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv2.circle(frame, center, 1, (0, 100, 100), 3)
                cv2.line(drawing, bin_pos, center, (50, 50, 50), 1)
                # circle outline
                radius = i[2]
                cv2.circle(frame, center, radius, (0, 0, 255), 3)

        # perform marker detection
        frame, rvecs, tvecs = detect_markers_and_draw(mtx, dist, frame, drawing)

        # update display image
        cv2.imshow(win_name, frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    # close all windows
    cv2.destroyAllWindows()


def get_frame():
    if VIDEO_INPUT:
        ret, frame = cap.read()
    else:
        frame = cv2.imread('calibr_images/WIN_20210411_12_35_33_Pro.jpg')
        frame = cv2.resize(frame, (640, 480))

    return frame


# returns manually set position of robot
def get_position():
    x = cv2.getTrackbarPos('x', win_name)
    y = cv2.getTrackbarPos('y', win_name)
    gripper_angle = cv2.getTrackbarPos('Gripper Angle', win_name)
    gripper_open = cv2.getTrackbarPos('Gripper Open', win_name)

    return x, y, gripper_angle, gripper_open


def set_coord(new_coord):
    global latest_coordinates, coord
    # convert to coordinates to ints and update global list
    # latest_coordinates = [int(i) for i in new_coord]
    coord.append(new_coord)


def start_cv():
    try:
        cv_win_thread = threading.Thread(target=draw_arms)
        cv_win_thread.start()
    except:
        print('Error: thread not started')


def generate_markers(num_markers, inverted):
    # for each marker
    for i in range(num_markers):
        # get marker from dictionary
        img = aruco.drawMarker(ARUCO_DICT, i, 500)

        # invert if option selected
        if inverted:
            img = ~img

        # save image
        filename = f'marker_{i}.jpg'
        cv2.imwrite(filename, img)


# input frame from video input
# and image
def detect_markers_and_draw(mtx, dist, frame, drawing_img):
    marker_size = 2.8/100   # marker length in meters
    axis_length = 0.015

    # get dictionary and parameters
    parameters = aruco.DetectorParameters_create()  # create parameters
    parameters.detectInvertedMarker = False  # enable detection of inverted markers

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # convert to gray

    # get marker data
    corners, ids, rejected_img_points = aruco.detectMarkers(gray, ARUCO_DICT, parameters=parameters)

    # draw corners on frame
    frame_m = aruco.drawDetectedMarkers(frame, corners, ids)

    if len(corners) == 4:

        # get corners of initial image
        y_max = frame.shape[0]
        x_max = frame.shape[1]
        frame_corners = [[0, 0], [x_max, 0], [0, y_max], [x_max, y_max]]

        # get transform corners
        transform_corners = np.zeros((len(ids), 2))
        for i in range(len(ids)):
            corner_num = ids[i][0]

            # get center x and y (calculating average)
            x, y = 0, 0
            for j in range(4):
                x += corners[i][0][j][0]
                y += corners[i][0][j][1]

            # add center to transform matrix
            transform_corners[corner_num] = [x/4, y/4]

        # transform drawing
        pts1 = np.float32(frame_corners)
        pts2 = np.float32(transform_corners)
        M = cv2.getPerspectiveTransform(pts1, pts2)
        drawing_img = cv2.warpPerspective(drawing_img, M, (drawing_img.shape[1], drawing_img.shape[0]))

        # create mask
        gray_square = cv2.cvtColor(drawing_img, cv2.COLOR_BGR2GRAY)
        ret, mask = cv2.threshold(gray_square, 5, 255, cv2.THRESH_BINARY)
        mask_inv = cv2.bitwise_not(mask)

        # get background from frame
        # print(f'frame_m shape {frame_m.shape}')
        # print(f'mask shape {mask_inv.shape}')
        frame_bg = cv2.bitwise_and(frame_m, frame_m, mask=mask_inv)

        # take only drawing from drawing image
        drawing_img = cv2.bitwise_and(drawing_img, drawing_img, mask=mask)

        frame_m = cv2.add(frame_bg, drawing_img)

        # point_1 = (int(corners[0][0][0][0]), (corners[0][0][0][1]))
        # point_2 = (int(corners[1][0][0][0]), (corners[1][0][0][1]))
        # cv2.line(square_img, point_1, point_2, (255, 0, 0), 3)

    # estimate pose
    rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, mtx, dist)
    if tvecs is not None:
        for i in range(len(tvecs)):
            frame_m = aruco.drawAxis(frame_m, mtx, dist, rvecs[i], tvecs[i], axis_length)

    return frame_m, rvecs, tvecs


def detect_markers_test(live):
    # init aruco detection
    ret, mtx, dist, rvecs, tvecs = camera_calibration()

    if live:
        cap = cv2.VideoCapture(1)

    while True:
        if live:
            ret, frame = cap.read()
        else:
            frame = cv2.imread('calibr_images/WIN_20210411_12_35_33_Pro.jpg')

        # perform marker detection
        square_img = np.zeros(frame.shape, np.uint8)     # create a blank image
        # draw drawing
        s = 100
        y_max = frame.shape[0]
        x_max = frame.shape[1]
        top_left = (int(x_max / 2 - s), int(y_max / 2 - s))
        bottom_right = (int(x_max / 2 + s), int(y_max / 2 + s))
        cv2.rectangle(square_img, top_left, bottom_right, (0, 255, 0), 3)

        frame_m, tvec, rvec = detect_markers_and_draw(mtx, dist, frame, square_img)
        if tvec is not None:
            #  print(f'tvec: {tvec[0]}')
            # print(f'rvec: {rvec[0]}')
            pass

        # update display image
        cv2.imshow(win_name, frame_m)

        if (cv2.waitKey(1) & 0xFF == 27) or not live:
            break

    while True:
        if cv2.waitKey(1) & 0xFF == 27:
            break

    # close all windows
    cv2.destroyAllWindows()


# calibration source:
# https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/sandbox/ludovic/aruco_calibration_rotation.html
def read_chessboards(images):
    """
    Charuco base pose estimation.
    """
    print("POSE ESTIMATION STARTS:")
    allCorners = []
    allIds = []
    decimator = 0
    board = aruco.CharucoBoard_create(7, 5, 1, .8, ARUCO_DICT)

    # SUB PIXEL CORNER DETECTION CRITERION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

    for im in images:
        print("=> Processing image {0}".format(im))
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, ARUCO_DICT)

        if len(corners) > 0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize=(3, 3),
                                 zeroZone=(-1, -1),
                                 criteria=criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3 and decimator % 1 == 0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        decimator += 1

    imsize = gray.shape
    return allCorners, allIds, imsize


def calibrate_camera(allCorners, allIds, imsize):
    """
    Calibrates the camera using the dected corners.
    """
    board = aruco.CharucoBoard_create(7, 5, 1, .8, ARUCO_DICT)
    print("CAMERA CALIBRATION")

    cameraMatrixInit = np.array([[1000., 0., imsize[0] / 2.],
                                 [0., 1000., imsize[1] / 2.],
                                 [0., 0., 1.]])

    distCoeffsInit = np.zeros((5, 1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    # flags = (cv2.CALIB_RATIONAL_MODEL)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
        charucoCorners=allCorners,
        charucoIds=allIds,
        board=board,
        imageSize=imsize,
        cameraMatrix=cameraMatrixInit,
        distCoeffs=distCoeffsInit,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors


def camera_calibration():
    img_dir = './calibr_images/'

    # get chessboard images
    images = np.array([img_dir + f for f in os.listdir(img_dir) if f.endswith('.jpg')])
    print(images)
    # read images
    allCorners, allIds, imsize = read_chessboards(images)
    print(f'all ids detected: {allIds}')

    # perform calibration
    ret, mtx, dist, rvecs, tvecs = calibrate_camera(allCorners, allIds, imsize)

    return ret, mtx, dist, rvecs, tvecs


def detect_circles(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # convert to gray
    gray = cv2.medianBlur(gray, 5)                  # apply median blur

    rows = gray.shape[0]
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows/8, param1=100, param2=30, minRadius=1, maxRadius=30)

    return circles
