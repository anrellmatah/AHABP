import cv2 as cv
import time

print('Hello from ahabp_node_opencv.py')

# camera servo can only go between 0 and 130 degrees
angle = 80

capture = cv.VideoCapture(0)

# on Raspberry Pi Camera v2 the image size is:
# 480 rows
# 640 columns
cx = 320
cy = 240
minimum = 250

picture = 1
i = 0
while True:
    istrue, frame = capture.read()
    
    #gray = cv.rotate(frame, cv.ROTATE_180)
    #frame = cv.rotate(frame, cv.ROTATE_180)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    thresholding, thresh = cv.threshold(gray, minimum, 255, cv.THRESH_BINARY)
    
    # copy and convert image to grayscale then process
    (minVal, maxVal, minLoc, maxLoc) = cv.minMaxLoc(gray)
    M = cv.moments(thresh)

    # calculate centroid
    targx = int(M["m10"] / (M["m00"]+1))
    targy = int(M["m01"] / (M["m00"]+1))

    # calculate direction vector from center to centroid
    vx = targx - cx
    vy = targy - cy

    cv.putText(frame, "target", (targx - 35, targy + 35), cv.FONT_HERSHEY_SIMPLEX, 2, (100, 255, 100), 2)
    cv.circle(frame, [targx, targy], 25, (255, 255, 32), 2)
    cv.arrowedLine(frame, [cx, cy], [targx, targy], (255, 255, 0), 2) # center >> target
    
    ### Output screens ###
    #cv.imshow('Output', thresh)
    #cv.imshow('Camera', frame)
    
    if (vy/10) > 0:
        if angle >= 5:
            angle -= 1
        #servo.ChangeDutyCycle(2 + (angle/18)) # This is where the actuator stuff goes
        print(vy/10, "pitch DOWN", angle)
    elif (vy/10) < 0:
        if angle <= 95:
            angle += 1
        #servo.ChangeDutyCycle(2 + (angle/18))
        print(vy/10, "pitch UP", angle)
    
    if cv.waitKey(20) & 0xFF==ord('d'):
        break

    if i >= 900:
        cv.imwrite("thresh_" + str(picture) + ".jpg", thresh)
        cv.imwrite("frame_" + str(picture) + ".jpg", frame)
        i = 0
        picture += 1
    
    #increment i
    i += 1

capture.release()
cv.destroyAllWindows
