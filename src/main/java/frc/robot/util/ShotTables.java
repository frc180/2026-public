package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Robot;
import frc.robot.util.simulation.SimUtils;

public final class ShotTables {

    private  static final double T_FIX = 0.99 * 0.65;

    public static class ShotTable {
        final InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();
        final InterpolatingDoubleTreeMap distanceToFlywheelRPS = new InterpolatingDoubleTreeMap();
        final InterpolatingDoubleTreeMap distanceToTimeOfFlight = new InterpolatingDoubleTreeMap();
        final double minDistance, maxDistance;

        public ShotTable(double[][] shots) {
            double min = Double.MAX_VALUE;
            double max = Double.MIN_VALUE;

            for (double[] entry : shots) {
                double distance = entry[0];
                double hoodAngle = entry[1];
                double flywheelRPS = entry[2];
                double timeOfFlight = entry[3];

                min = Math.min(min, distance);
                max = Math.max(max, distance);

                distanceToHoodAngle.put(distance, hoodAngle);
                distanceToFlywheelRPS.put(distance, flywheelRPS);
                distanceToTimeOfFlight.put(distance, timeOfFlight);
            }

            this.minDistance = min;
            this.maxDistance = max;
        }

        public double getHoodAngle(double distanceMeters) {
            return distanceToHoodAngle.get(distanceMeters + Robot.getDistanceOffset());
        }

        public double getFlywheelRPS(double distanceMeters) {
            return distanceToFlywheelRPS.get(distanceMeters + Robot.getDistanceOffset());
        }

        // Temporary (at least for real robot) until we have time of flight in the shot tables
        public double getTimeOfFlight(double distanceMeters) {
            if (Robot.isReal()) {
                return distanceToTimeOfFlight.get(distanceMeters + Robot.getDistanceOffset());
            }

            double shooterVelocity = SimUtils.calculateExitVelocity(getFlywheelRPS(distanceMeters));
            double hoodAngle = getHoodAngle(distanceMeters);

            return SimUtils.calculateIdealTimeOfFlight(
                shooterVelocity,
                hoodAngle,
                SimUtils.SHOOTER_HEIGHT.in(Meters),
                SimUtils.HUB_HEIGHT.in(Meters)
            );
        }

        public double getMinDistance() {
            return minDistance;
        }

        public double getMaxDistance() {
            return maxDistance;
        }
    }

    private static ShotTable hubShotTable, feedShotTable;

    public static void init() {
        double[][] shotData;
        if (Robot.isReal()) {
            shotData = new double[][] {
                // distanceMeters, hoodAngleDegrees, flywheelRPS, timeOfFlightS
                {1.557, -2, 25.5, 0.8}, // almost closest to hub you can still see tags ;)
                {3.25, 7, 30, 0.94}, // middle of right trench
                {4, 6.6, 34, 1.2},
                {5.0, 6, 36, 1.16}, // intake facing corner shot,
                {5.67, 6, 39, 1.28}, // turret in right corner
                {6.28, 7, 44, 1.28}

            };
        } else {
            shotData = new double[][] {
                // distanceMeters, hoodAngleDegrees, flywheelRPS, timeOfFlightS
                {1.737, 80, 48, -1},
                {2.737, 60, 48, -1},
                {4.325, 60, 57, -1},
                {5.628, 60, 64, -1}
            };
        }

        hubShotTable = new ShotTable(shotData);


        double[][] feedShotData;
        if (Robot.isReal()) {
            feedShotData = new double[][] {
                // distanceMeters, hoodAngleDegrees, flywheelRPS, timeOfFlightS
                {4.08, 30.7, 45 * T_FIX, 1},
                {6.58, 30.7, 55 * T_FIX, 1},
                {9.68, 30.7, ((75 + 10) * T_FIX) + 3 + 4, 1},
                {11.27, 30.7, ((88 + 10) * T_FIX) + 3 + 4, 1}
            };
        } else {
            feedShotData = new double[][] {
                {0, 20, 50, 1},
                {10, 20, 50, 1}
            };     
        }

        feedShotTable = new ShotTable(feedShotData);
    }

    public static ShotTable getHubShotTable() {
        return hubShotTable;
    }

    public static ShotTable getFeedShotTable() {
        return feedShotTable;
    }
}
