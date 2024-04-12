# Import the necessary packages
from skimage import measure
import numpy as np
import imutils
import cv2
import argparse
import os

def identify_organism_type(num_leds):
    if num_leds == 2:
        return "alien_a"
    elif num_leds == 3:
        return "alien_b"
    elif num_leds == 4:
        return "alien_c"
    elif num_leds == 5:
        return "alien_d"
    else:
        return "unknown"

def find_closest_cluster(led, clusters, restricted_box_size):
    for i, cluster in enumerate(clusters):
        for cluster_led in cluster:
            # Consider a restricted box around each LED (4 times the size of each LED)
            if (
                led[0] - restricted_box_size <= cluster_led[0] <= led[0] + restricted_box_size
                and led[1] - restricted_box_size <= cluster_led[1] <= led[1] + restricted_box_size
            ):
                return i
    return -1

parser = argparse.ArgumentParser(description="LED Detection Script")
parser.add_argument("--image", required=True, help="Path to the image file")

args = parser.parse_args()
image_path = args.image
filename, _ = os.path.splitext(image_path)
result_file = f"{filename}.txt"

# Load the image
image = cv2.imread(image_path, 1)

# Convert it to grayscale and blur it
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (11, 11), 0)

# Threshold the image to reveal light regions in the blurred image
thresh = cv2.threshold(blurred, 230, 255, cv2.THRESH_BINARY)[1]

# Perform a series of erosions and dilations to remove small blobs of noise
thresh = cv2.erode(thresh, None, iterations=0)
thresh = cv2.dilate(thresh, None, iterations=4)

# Perform connected component analysis on the thresholded image and initialize a mask
labels = measure.label(thresh, connectivity=2, background=0)
mask = np.zeros(thresh.shape, dtype="uint8")

# Loop over the unique components
for label in np.unique(labels):
    # If this is the background label, ignore it
    if label == 0:
        continue

    # Construct the label mask and count the number of pixels
    labelMask = np.zeros(thresh.shape, dtype="uint8")
    labelMask[labels == label] = 255
    numPixels = cv2.countNonZero(labelMask)

    # If the number of pixels in the component is sufficiently large, add it to the mask
    if numPixels > 10:
        mask = cv2.add(mask, labelMask)

# Find the contours in the mask
cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)

# Initialize lists to store centroid coordinates for each LED
leds = []
sizes = []
i=0
# Loop over the contours
for c in cnts:
    # Calculate the centroid of the contour
    
    M = cv2.moments(c)
    if M["m00"] != 0:
        cx = M["m10"] / M["m00"]  # X-coordinate as a float
        cy = M["m01"] / M["m00"]  # Y-coordinate as a float
        centroid = (cx, cy)
        leds.append(centroid)

        # Calculate the size of the bounding box around the LED
        (x, y, w, h) = cv2.boundingRect(c)
        sizes.append(max(w, h))

    cv2.drawContours(image, [c], -1, (0, 0, 255), 2)
    offset=10
    # Write the position of each LED in red with an offset
    text = f"#{i + 1}"
    i+=1
    font = cv2.FONT_HERSHEY_SIMPLEX
    color = (0, 0, 255)  # Red color
    scale = 0.5
    thickness = 1
    (x, y), _ = cv2.minEnclosingCircle(c)
    x, y = int(x), int(y)
    cv2.putText(image, text, (x - 10, y - 10 - offset), font, scale, color, thickness)
    

# Compute the average size of the detected LEDs
average_size = np.mean(sizes)

# Use the average size to determine the restricted box size
restricted_box_size = int(average_size * 4)

# Initialize a list to store clusters of LEDs
clusters = []

# Loop over the LEDs
for led in leds:
    # Find the closest cluster to the current LED
    closest_cluster_idx = find_closest_cluster(led, clusters, restricted_box_size)

    # If no close cluster is found, create a new cluster with the current LED
    if closest_cluster_idx == -1:
        clusters.append([led])
    else:
        # Add the LED to the closest cluster
        clusters[closest_cluster_idx].append(led)

# Initialize lists to store centroid coordinates and area for each cluster
centroid_list = []
area_list = []

# Loop over the clusters
for i, cluster in enumerate(clusters):
    # Calculate the average centroid of the cluster
    avg_centroid = np.mean(np.array(cluster), axis=0)

    # Append centroid coordinates and area to the respective lists
    centroid_list.append(avg_centroid)
    area_list.append(len(cluster))

# Open a text file for writing
with open(result_file, "w") as file:
    # Loop over the clusters
    for i, (centroid, num_leds) in enumerate(zip(centroid_list, area_list)):
        # Identify organism type based on the number of LEDs in each cluster
        organism_type = identify_organism_type(num_leds)

        # Write the results to the file in the specified format
        file.write(f"Organism Type: {organism_type}\n")
        file.write(f"Centroid: ({centroid[0]:.4f}, {centroid[1]:.4f})\n\n")

# Save the output image as a PNG file
cv2.imwrite(f"{filename}_marked.png", image)

cv2.imshow("Marked Image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()