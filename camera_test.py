#!/usr/bin/python
import cv2 as cv

cv_var = cv.VideoCapture(0, cv.CAP_DC1394)

def main():
    while True:
        ret, frame = cv_var.read()
        if ret:
            cv.imshow("Video", frame)
        else:
            print("Error")
            break

if __name__ == "__main__":
    main()