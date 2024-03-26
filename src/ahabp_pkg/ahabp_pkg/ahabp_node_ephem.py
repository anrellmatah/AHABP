import ephem
import datetime

print('#### Hi from ahabp_node_ephem.py ####')

pi = 3.14159265358979323846

# incorporate into video_thresholding.py for ideal tracking

def get_compass_direction(angle):
    # this just converts to cardinal directions for ease of use
    # not to be used in actual operation
    directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    index = round(angle / (360. / len(directions))) % len(directions)
    
    return directions[index]

def sun_angle_and_direction(latitude, longitude, date):
    
    observer = ephem.Observer()
    observer.lat = str(latitude)
    observer.lon = str(longitude)

    observer.date = date
    sun = ephem.Sun(observer)

    zenith_angle = 90 - sun.alt*180/pi
    azimuth_angle = (sun.az*180/pi) % 360
    compass_direction = get_compass_direction(azimuth_angle)

    return zenith_angle, azimuth_angle, compass_direction


def movement_needed(payload_heading, azimuth_angle, zenith_angle):
    # calculated yaw and pitch needed
    yaw = round(azimuth_angle - payload_heading, 2)
    pitch = round(zenith_angle - camera_angle, 2)

    return yaw, pitch

if __name__ == "__main__":

    # Tempe, AZ lat/long
    #latitude = 33.427204
    #longitude = -111.939896


    # 0 is horizontal
    # +76 is maximum
    # -28 is minimum
    
    
    while True:
        ### TO DO ###
        # get real time GPS lat/long here
        # get real time compass heading here
        # get real time servo pitch angle here
        latitude = 30.266666
        longitude = -97.733330
        payload_heading = 88   # in degrees
        camera_angle = 0  # in degrees

        date = datetime.datetime.now()
        zenith_angle, azimuth_angle, compass_direction = sun_angle_and_direction(latitude, longitude, date)
        yaw, pitch = movement_needed(payload_heading, azimuth_angle, zenith_angle)

        # print out the given information    
        #print("GPS Lat/Long:", latitude, " ", longitude)
        print(f"Zenith: {round(zenith_angle, 2)}\tAzimuth: {round(azimuth_angle, 2)}\tHeading: {payload_heading}\tYaw: {yaw}\tCamera: {camera_angle}\tPitch: {pitch}", end="\r")