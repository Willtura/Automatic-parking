import cv2
import numpy as np


def resize_image(image, target_width, target_height) -> np.ndarray:
    """Resize an image to a target width and height.

    :param image: The image to resize.
    :param target_width: The target width.
    :param target_height: The target height.
    :return: The resized image.
    """

    resized_image = cv2.resize(image, (target_width, target_height))
    return resized_image

def detect_edges(image) -> np.ndarray:
    """Detect edges in an image.

    :param image: The image to detect edges in.
    :return: The image with edges detected.
    """

    blurred_image = cv2.GaussianBlur(image, (5, 5), 0)
    #perform a Gaussian blur on the image to reduce noise

    edges = cv2.Canny(blurred_image, 160, 255)  # --> Canny(image, low_threshold, high_threshold)
    cv2.imshow('Edges Detected', resize_image(edges, 1191, 591))
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return edges

def display_lines(image, lines) -> np.ndarray:
    """Display found lines on the image.

    :param image: The image to display the lines on.
    :param lines: The lines to display.
    :return: The image with the lines displayed.
    """

    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return line_image

def detect_lanes(image) -> np.ndarray:
    """Detect lanes in an image.

    :param image: The image to detect lanes in.
    :return: The image with lanes detected.
    """

    edges = detect_edges(image)
    lines = cv2.HoughLinesP(edges, 3, np.pi / 180, threshold=80, minLineLength=30, maxLineGap=4)
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return image

def filter_white(image) -> np.ndarray:
    """Filter white pixels from an image.

    :param image: The image to filter white pixels from.
    :return: The image with white pixels filtered.
    """

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    mask = cv2.inRange(hsv_image, lower_white, upper_white)
    white_filtered_image = cv2.bitwise_and(image, image, mask=mask)
    cv2.imshow('White filter', resize_image(white_filtered_image, 1191, 591))

    return white_filtered_image

def overlay_images(image1, image2, alpha) -> np.ndarray:
    """Overlay two images with a given alpha value.

    :param image1: The first image.
    :param image2: The second image.
    :param alpha: The alpha value.
    :return: The overlayed image.
    """

    image2_rescaled = cv2.resize(image2, (image1.shape[1], image1.shape[0]))
    overlay = cv2.addWeighted(image1, 1 - alpha, image2_rescaled, alpha, 0)

    return overlay

image = cv2.imread('data/3.jpg')
# Load the image
white_filtered_image = filter_white(resize_image(image, 1191, 591))
# Filter the white pixels from the image

output_image = detect_lanes(white_filtered_image)
# Detect the lanes in the image


image1 = resize_image(image, 1191, 591)
image2 = output_image
alpha = 0.5
overlay = overlay_images(image1, image2, alpha)

cv2.imshow('Overlay', resize_image(output_image, 1191, 591))
cv2.waitKey(0)
cv2.destroyAllWindows()
