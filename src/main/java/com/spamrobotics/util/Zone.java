package com.spamrobotics.util;

import edu.wpi.first.math.geometry.Translation2d;

// Based off https://stackoverflow.com/questions/8721406/how-to-determine-if-a-point-is-inside-a-2d-convex-polygon
public class Zone {
    private final Translation2d[] points; // Points making up the Zone

    public Zone(Translation2d... points) {
        this.points = points;
    }

    /**
     * Return true if the given point is contained inside the Zone.
     * See: http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
     * @param test The point to check
     * @return true if the point is inside the boundary, false otherwise
     *
     */
    public boolean contains(Translation2d point) {
      int i;
      int j;
      boolean result = false;
      for (i = 0, j = points.length - 1; i < points.length; j = i++) {
        if ((points[i].getY() > point.getY()) != (points[j].getY() > point.getY()) &&
            (point.getX() < (points[j].getX() - points[i].getX()) * (point.getY() - points[i].getY()) / (points[j].getY()-points[i].getY()) + points[i].getX())) {
          result = !result;
         }
      }
      return result;
    }

    public Zone allianceFlip() {
        Translation2d[] flippedPoints = new Translation2d[points.length];
        for (int i = 0; i < points.length; i++) {
            flippedPoints[i] = Helpers.allianceFlip(points[i]);
        }
        return new Zone(flippedPoints);
    }

    public Zone yFlip() {
        Translation2d[] flippedPoints = new Translation2d[points.length];
        for (int i = 0; i < points.length; i++) {
            flippedPoints[i] = Helpers.yFlip(points[i]);
        }
        return new Zone(flippedPoints);
    }
}