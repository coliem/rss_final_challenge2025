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
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########

	bounding_box = ((0,0),(0,0))
	# image_print(img)
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	# image_print(hsv)

	lower_orange = np.array([0, 220, 100]) #0, 220, 102
	upper_orange = np.array([30, 255, 255]) #45, 255, 255

	mask = cv2.inRange(hsv, lower_orange, upper_orange)
	# image_print(mask)

	kernel = np.ones((5, 5), np.uint8)
	# mask = cv2.erode(mask, kernel, iterations=2)
	mask = cv2.dilate(mask, kernel, iterations=1)
	# mask = cv2.erode(mask, kernel, iterations=1)
	# image_print(mask)
    # Find external contours in the mask.
	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If contours are found, select the largest as the cone.
	if contours:
		largest_contour = max(contours, key=cv2.contourArea)
		x, y, w, h = cv2.boundingRect(largest_contour)
		bounding_box = ((x, y), (x + w, y + h))
		cv2.rectangle(img, (x,y), (x+w,y+h), (0,0,255), 2)
		# image_print(img)
	else:
		bounding_box = ((0, 0), (0, 0))


	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box



if __name__ == "__main__":
	pass

