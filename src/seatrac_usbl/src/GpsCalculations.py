import math as m
import numpy as np
import time

class GpsCalculations():

    def __init__(self):
        self.gps_calc = True
        self.earth_rad = 6371008

    def midpoint(self,p1_lat, p1_lon, p2_lat, p2_lon):
        """This function calculates the half-way point along a great circle path between two points
        input:
            p1_lat = the latitude in degrees of the first point [deg]
            p1_lon = the longitude in degrees of the first point [deg]
            p2_lat = the latitude in degrees of the second point [deg]
            p2_lon = the longitude in degrees of the second point [deg]
        output:
            midlat = latitude of midpoint between p1 and p2 [deg]
            midlong = longitude of midpoint between p1 and p2 [deg]"
        ---------------------------------------------------------------
        For reference:
        phi = radial distance of a point from equator = latitude in radians
        lambda (lam) = radial distance from prime meridian = longitude in radians"""
        # Update lats and longs to radians
        dlam = np.radians(p2_lon - p1_lon)  # Calc radial diff of both longitudes and convert to rads
        phi1 = np.radians(p1_lat)  # Calculate latitude of point 1 in radians
        phi2 = np.radians(p2_lat)  # Calculate latitude of point 2 in radians
        lam1 = np.radians(p1_lon)  # Calculate longitude of point 1 in radians

        # Calculate change in N-S and E-W direction
        bx = np.cos(phi2) * np.cos(dlam)  # Calculate change in x direction (East - West)
        by = np.cos(phi2) * np.sin(dlam)  # Calculate change in y direction (North - South)

        # Calculate latitude and longitude of point in the middle of two other points
        phimid = np.arctan2(np.sin(phi1) + np.sin(phi2), np.sqrt((np.cos(phi1) + bx) ** 2 + by ** 2))  # Mid lat [deg]
        lammid = ((lam1 + np.arctan2(by, np.cos(phi1) + bx)) + 3 * np.pi) % (2 * np.pi) - np.pi  # Mid long [deg]
        midlat = np.degrees(phimid)  # Middle point latitude in degrees
        midlong = np.degrees(lammid)  # middle point longitude in degrees
        return midlat, midlong  # Return lat, long pair


    def cross_track_distance(self,p1_lat, p1_lon, p2_lat, p2_lon, p3_lat, p3_lon, earth_rad):
        """This Function will calculate the distance of a point from a great-circle path
        input:
            p1_lat = the latitude in degrees of the first point [deg]
            p1_lon = the longitude in degrees of the first point [deg]
            p2_lat = the latitude in degrees of the second point [deg]
            p2_lon = the longitude in degrees of the second point [deg]
            p3_lat = the latitude in degrees of the third point [deg]
            p3_lon = the longitude in degrees of the third point [deg]
            earth_rad = volumetric mean radius of the earth [m]
        output:
            value = cross track distance [m]"""
        # Calculate radial distances between points
        delta13 = distance__haversine(p1_lat, p1_lon, p3_lat, p3_lon, earth_rad)  # Base dist start --> 3rd pnt
        theta12 = np.radians(initial_bearing(p1_lat, p1_lon, p2_lat, p2_lon))  # Calc Bearing start --> end
        theta13 = np.radians(initial_bearing(p1_lat, p1_lon, p3_lat, p3_lon))  # Calc bearing start --> 3rd pnt

        # Calculate distance that point 3 is off from Point1 --> Point2 track
        value = np.arcsin(np.sin(delta13 / earth_rad) * np.sin(theta13 - theta12)) * earth_rad
        return value  # Return the distance in meters


    def along_track_distance(self,p1_lat, p1_lon, p2_lat, p2_lon, p3_lat, p3_lon, earth_rad):
        """This function will calculate how far along a path a new point is
        input:
            p1_lat = the latitude in degrees of the first point [deg]
            p1_lon = the longitude in degrees of the first point [deg]
            p2_lat = the latitude in degrees of the second point [deg]
            p2_lon = the longitude in degrees of the second point [deg]
            p3_lat = the latitude in degrees of the third point [deg]
            p3_lon = the longitude in degrees of the third point [deg]
            earth_rad = volumetric mean radius of the earth in meters [m]
        output:
            value = distance along track  [m]"""
        # Calculate the Distance between the two points
        d13 = distance__haversine(p1_lat, p1_lon, p3_lat, p3_lon, earth_rad)
        # Calculate the distance off the track the new point is
        dxt = cross_track_distance(p1_lat, p1_lon, p2_lat, p2_lon, p3_lat, p3_lon, earth_rad)
        # Calculate distance along the path and return the value
        value = np.arccos(np.cos(d13 / earth_rad) / np.cos(dxt / earth_rad)) * earth_rad
        return value  # return the distance in meters


    def distance__haversine(self,p1_lat, p1_lon, p2_lat, p2_lon, earth_radius):
        """This function calculates the shortest, "as the crow flies", distance between two points
        where point 1 is the starting point and point 2 is the end point.
        input:
            p1_lat = the latitude in degrees of the first point [deg]
            p1_lon = the longitude in degrees of the first point [deg]
            p2_lat = the latitude in degrees of the second point [deg]
            p2_lon = the longitude in degrees of the second point [deg]
            earth_rad = volumetric mean radius of the earth [m]
        output:
            dist = distance between p1 and p2 along the earth curvature [m]"""
        # Calculate the mean square sin of the change in lat an long
        sdlat2 = np.sin(np.radians(p1_lat - p2_lat) / 2.0) ** 2  # change in lat
        sdlon2 = np.sin(np.radians(p1_lon - p2_lon) / 2.0) ** 2  # change in long
        # Calculate a coefficient of the haversine formula
        a = sdlat2 + sdlon2 * np.cos(np.radians(p1_lat)) * np.cos(np.radians(p2_lat))
        # Calculate final distance
        dist = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a)) * earth_radius
        return dist  # return the distance in meters


    def initial_bearing(self,p1_lat, p1_lon, p2_lat, p2_lon):
        """Bearing towards the starting point (point 1) to the end point (point 2)
        input:
            p1_lat = the latitude in degrees of the first point
            p1_lon = the longitude in degrees of the first point
            p2_lat = the latitude in degrees of the second point
            p2_lon = the longitude in degrees of the second point
        output:
            bearing = compass heading to get from point 2 from point 1 [deg]
        ---------------------------------------------------------------
        For reference:
        phi = radial distance of a point from equator = latitude in radians
        lambda (lam) = radial distance from prime meridian = longitude in radians"""
        # Convert lats and longs to radians
        dlam = np.radians(p2_lon - p1_lon)  # delta or change in longitude in radians
        phi1 = np.radians(p1_lat)  # The latitude of point 1 in radians
        phi2 = np.radians(p2_lat)  # the latitude of point 2 in radians
        # Calculate offsets between points
        y = np.sin(dlam) * np.cos(phi2)  # Calculate the X or E-W offset
        x = np.cos(phi1) * np.sin(phi2) - np.sin(phi1) * np.cos(phi2) * np.cos(dlam)  # Calculate the Y or N-S offset
        # Calculate resulting angle through base trig
        bearing = np.degrees(np.arctan2(y, x)) % 360  # Calculate the bearing and convert to degrees
        return bearing  # return the bearing in degrees


    def destination_point(self,p1lat, p1lon, dist, bear):
        """This function will calculate a resulting Latitude nad Longitude waypoint based upon a waypoint,
        a distance to travel, and a bearing to travel on
        input:
            p1lat = the latitude in degrees of te start point
            p1lon = the longitude in degrees of the start point
            dist = distance to travel
            bear = bearing to travel along
        output:
            lat2 = latitude of point dist awat from p1 at bear direction [deg]
            lon2 = longitude of point dist awat from p1 at bear direction [deg]
        ---------------------------------------------------------------
        For reference:
        phi = radial distance of a point from equator = latitude in radians
        lambda (lam) = radial distance from prime meridian = longitude in radians"""
        # Convert starting point Lat and long and bearing to radians
        phi1 = np.radians(p1lat)  # Start point latitude in radians
        lam1 = np.radians(p1lon)  # start point longitude in radians
        bearing = np.radians(bear)  # desired bearing in radians
        dist = dist/self.earth_rad
        # Calculate new lat in rads
        phi2 = np.arcsin(np.sin(phi1)*np.cos(dist) + np.cos(phi1)*np.sin(dist)*np.cos(bearing))
        # Calculate new long in rads
        lam2 = lam1 + np.arctan2(np.sin(bearing)*np.sin(dist)*np.cos(phi1), np.cos(dist)-np.sin(phi1)*np.sin(phi2))
        # convert new points to degrees
        lat2 = np.rad2deg(phi2)  # Point 2 latitude to degrees
        lon2 = np.rad2deg(lam2)  # Point 2 longitude to degrees
        return lat2, lon2  # Return the lat long pair

    def ang_ned(self, angle):
        # Make sure the angle is within the expected range
        while angle > 2*m.pi:
            angle = angle - 2*m.pi
        while angle < 0:
            angle = angle + 2*m.pi
        # Convert the input angle to an angle with 0 degrees pointing north
        if (angle  >= 0.0        and angle < m.pi/2):
            ned_angle = m.pi/2-angle
        elif(angle >= m.pi/2   and angle < m.pi):
            ned_angle = 2*m.pi - (angle - m.pi/2)
        elif(angle >= m.pi     and angle < 3*m.pi/2):
            ned_angle = 3*m.pi/2 - (angle - m.pi)
        elif(angle >= 3*m.pi/2 and angle <= 2*m.pi):
            ned_angle = m.pi - (angle - 3*m.pi/2)
        return ned_angle


    def convert_2_bearing(self,angle):
        """This function converts a given angle from the path follower to a compass heading to use
        Input: Angle with respect to unit circle (+x axis) in radians (0 degrees east and increasing counter-clockwise
        Output: Compass heading in degrees (0 degrees north and increasing clockwise)"""
        # print(angle)
        x = m.cos(angle)
        y = m.sin(angle)
        bearingrad = m.atan2(x, y)
        # print(bearingRad)
        bearing = bearingrad*180/m.pi # calculate bearing wrt compass heading (east -> north) and flip z axis
        if bearing < 0:  # if we got a negative bearing from atan2
            bearing = bearing + 360
        return bearing


    def convert_2_angle(self,bearing):
        """This function converts a bearing from the onboad compass into radians
        Input: Compass heading in degrees (0 degrees north and increasing clockwise)
        Output: Angle with respect to unit circle (+x axis) in radians (0 degrees east and increasing counter-clockwise"""
        bearing = bearing*m.pi/180  # convert to radians
        x = m.cos(bearing)
        y = m.sin(bearing)
        angle = m.atan2(x, y)
        if angle < 0:  # if we got a negative bearing from atan2
            angle = angle + 2*m.pi
        return angle

if __name__ == '__main__':
    print(GpsCalculations().destination_point(40.402683,-86.845276,5.099020,258.690068))
