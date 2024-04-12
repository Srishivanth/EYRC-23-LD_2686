from skimage import measure
import numpy as np
import imutils
import cv2

image = cv2.imread('/home/srishivanth/catkin_ws/src/luminosity_drone/luminosity_drone/scripts/led.jpg', 1)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (11, 11), 0)

thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]

thresh = cv2.erode(thresh, None, iterations=2)
thresh = cv2.dilate(thresh, None, iterations=4)

labels = measure.label(thresh, connectivity=2, background=0)
mask = np.zeros(thresh.shape, dtype="uint8")

for label in np.unique(labels):
    if label == 0:
        continue

    labelMask = np.zeros(thresh.shape, dtype="uint8")
    labelMask[labels == label] = 255
    numPixels = cv2.countNonZero(labelMask)

    if numPixels > 300:
        mask = cv2.add(mask, labelMask)

cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)

centroid_list = []
area_list = []

offset = 20

for i, c in enumerate(cnts):
    area = cv2.contourArea(c)

    M = cv2.moments(c)
    if M["m00"] != 0:
        cx = M["m10"] / M["m00"]  
        cy = M["m01"] / M["m00"]  
        centroid = (cx, cy)
    else:
        centroid = (0.0, 0.0)  

    centroid_list.append(centroid)
    area_list.append(area)

    cv2.drawContours(image, [c], -1, (0, 0, 255), 2)

    text = f"#{i + 1}"
    font = cv2.FONT_HERSHEY_SIMPLEX
    color = (0, 0, 255) 
    scale = 0.5
    thickness = 1
    (x, y), _ = cv2.minEnclosingCircle(c)
    x, y = int(x), int(y)
    cv2.putText(image, text, (x - 10, y - 10 - offset), font, scale, color, thickness)

with open("led_detection_results.txt", "w") as file:
    file.write(f"No. of LEDs detected: {len(cnts)}\n")
    for i, (centroid, area) in enumerate(zip(centroid_list, area_list)):
        file.write(f"Centroid #{i + 1}: ({centroid[0]:.15f}, {centroid[1]:.15f})\n")
        file.write(f"Area #{i+1}: {area:.5f}\n")

cv2.imwrite("led_detection_results.png", image)

cv2.imshow("Marked Image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
