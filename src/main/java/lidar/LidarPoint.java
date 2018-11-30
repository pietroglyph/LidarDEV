//package com.spartronics4915.frc2019.lidar;

package lidar;
//import com.spartronics4915.frc2019.RobotState;

//import com.spartronics4915.lib.math.Translation2d;
//import com.spartronics4915.lib.math.Pose2d;

import main.Main;
import lib.math.Translation2d;
import lib.math.Pose2d;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Represents a single point from the LIDAR sensor. This consists of
 * an angle, distance, and timestamp.
 */
class LidarPoint 
{
    public static final double MM_TO_IN = 1/25.4; // 1 inch = 25.4 millimeters

    // A scan is a collection of lidar points.  The scan, itself,
    // has a timestamp as does each point.  Currently, the timestamp
    // of each point is the same as scan, but this needn't be the case
    // since a full scan requires 1/(5-10hz) seconds. The robot pose
    // is tracked by the application/main and the position of the robot
    // time a point is acquired by interpolating known robot poses to
    // the requested time.  Since this operation is non-trivial and since
    // we assume that multiple LidarPoints are acquired at the "same time",
    // we employ a local cache that maps timestamp to robot pose.  This
    // cache is a "class variable" - ie it's shared across all points.
    private final static int MAX_ENTRIES = 10;
    private final static LinkedHashMap<Double, Pose2d> sRobotPoseMap = 
        new LinkedHashMap<Double, Pose2d>() 
    {
        private static final long serialVersionUID = 1L;

        @Override
        protected boolean removeEldestEntry(Map.Entry<Double,Pose2d> eldest)
        {
            return this.size() > MAX_ENTRIES;
        }
    };


    /*  instance variables ... */
    public final double timestamp;
    public final double angle;
    public final double distance;

    public LidarPoint(double timestamp, double angle, double distance) 
    {
        this.timestamp = timestamp;
        this.angle = angle;
        this.distance = distance * MM_TO_IN;
    }

    /**
     * Convert this point into a {@link Translation2d} in cartesian (x, y)
     * coordinates. The point's timestamp is used along with the {@link RobotState}
     * to take into account the robot's pose at the time the point was detected.
    */ 
    public Translation2d toCartesian() 
    {
        // convert the polar coords to cartesian coords
        double radians = Math.toRadians(angle);
        Translation2d x2d = new Translation2d(Math.cos(radians) * distance, 
                                              Math.sin(radians) * distance);

        // transform by the robot's pose
        Pose2d robotPose;
        if (sRobotPoseMap.containsKey(timestamp)) 
        {
            robotPose = sRobotPoseMap.get(timestamp);
        } 
        else
        {
            robotPose = Main.getRobotPose(timestamp);
            if(robotPose != null)
                sRobotPoseMap.put(timestamp, robotPose);
        }
        if(robotPose != null)
        {
            robotPose.transformBy(Pose2d.fromTranslation(x2d));
            return robotPose.getTranslation();
        }
        else
            return x2d;
    }
}
