import cv2
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
    """
    Helper function to print out images, for debugging. Pass them in as a list.
    Press any key to continue.
    """
    cv2.imshow("image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
    """
    Implement the cone detection using color segmentation algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected. BGR.
        template_file_path; Not required, but can optionally be used to automate setting hue filter values.
    Return:
        bbox: ((left_bottom, left_top), (right_bottom, right_top)); the bounding boxes of the cones, unit in px
                For left box: (bottom_left, top_right)
                For right box: (bottom_right, top_left)
    """
    ########## Todo: make the code run faster in real time ##########

    bounding_box = (((0,0),(0,0)), ((0,0),(0,0)))
    ##image_print(img)
    # Cut the image to the bottom half
    #img = img[img.shape[0]//3:,:, :]
    ##print(img.shape)
    ##image_print(img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define color range for white
    darker_white = np.array([0, 0, 150]) # 30, 5, 150
    brighter_white = np.array([180, 50, 255]) # 100, 70, 255

    # Create mask and apply morphological operations
    mask = cv2.inRange(hsv, darker_white, brighter_white)
    ##image_print(mask)

    kernel1 = np.ones((5, 5), np.uint8)
    kernel2 = np.ones((3, 3), np.uint8) #
    mask = cv2.erode(mask, kernel2, iterations=2) #2
    mask = cv2.dilate(mask, kernel1, iterations=3) #3
    ##image_print(mask)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize default bounding boxes (no detection)
    bounding_box = []

    line_num = 0
    largest_contours = sorted(contours, key=cv2.contourArea, reverse=True)

    for contour in largest_contours:
        rect = cv2.minAreaRect(contour)

        #filter out horizontal lines
        angle = rect[2]
        if rect[1][0] < rect[1][1]:
            angle += 90
        threshold1 = 10 #15
        # threshold2 = 40
        # if abs(angle) < threshold1 or abs(angle - 180) < threshold1 or abs(angle) > threshold2 or abs(angle - 180) > threshold2:
        #     continue
        if abs(angle) < threshold1 or abs(angle - 180) < threshold1:
            continue

        box = cv2.boxPoints(rect)
        box = np.int32(box)
        cv2.drawContours(img, [box], 0, (0,0,255), 2)

        p1, p2, p3, p4 = box
        dis12 = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        dis13 = np.sqrt((p1[0] - p3[0])**2 + (p1[1] - p3[1])**2)
        dis14 = np.sqrt((p1[0] - p4[0])**2 + (p1[1] - p4[1])**2)

        if dis12 < dis13 and dis12 < dis14:
            p1_rect1 = (max(0, int((p1[0] + p2[0])/2.0)), max(0, int((p1[1] + p2[1])/2.0)))
            p2_rect1 = (max(0, int((p3[0] + p4[0])/2.0)), max(0, int((p3[1] + p4[1])/2.0)))
        elif dis13 < dis12 and dis13 < dis14:
            p1_rect1 = (max(0, int((p1[0] + p3[0])/2.0)), max(0, int((p1[1] + p3[1])/2.0)))
            p2_rect1 = (max(0, int((p2[0] + p4[0])/2.0)), max(0, int((p2[1] + p4[1])/2.0)))
        else:
            p1_rect1 = (max(0, int((p1[0] + p4[0])/2.0)), max(0, int((p1[1] + p4[1])/2.0)))
            p2_rect1 = (max(0, int((p2[0] + p3[0])/2.0)), max(0, int((p2[1] + p3[1])/2.0)))

        # cv2.line(img, p1_rect1, p2_rect1, (0,255,0), 2)
        bounding_box.append((p1_rect1, p2_rect1))

        line_num += 1
        if line_num == 2:
            break

    if len(bounding_box) == 1:
        bounding_box.append(bounding_box[0])
        #bounding_box.append(((10, 10), (100, 100)))
        # print("Found one box")
    elif len(bounding_box) == 0:
        # print("Found zero box")
        #bounding_box=[((10,10),(100,100)),((10,100),(100,10))]
        #raise ValueError("No bounding box found")
        bounding_box=[((0, 0), (0, 0)), ((0, 0), (0, 0))]
    ##print(bounding_box)
    ##image_print(img)

    #bounding_box = ((0,0),(0,0))
    return bounding_box



if __name__ == "__main__":
    pass
