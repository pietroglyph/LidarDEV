package main;

import java.util.concurrent.TimeUnit;

import lib.Looper;
import lib.util.InterpolatingDouble;
import lib.util.InterpolatingTreeMap;
import lib.util.Logger;
import lib.math.Pose2d;
import lidar.LidarProcessor;

public class Main
{
    enum OperatingMode
    {
        kRelative,
        kAbsolute
    };
    private static final int kPoseMapSize = 100;
    private static InterpolatingTreeMap<InterpolatingDouble, Pose2d> sPoses =
            new InterpolatingTreeMap<InterpolatingDouble, Pose2d>(kPoseMapSize);

    public static void main(String[] args)
    {
        final OperatingMode mode = OperatingMode.kRelative;
        Looper mLooper;
        LidarProcessor mLidarProcessor;
        Logger.setVerbosity("DEBUG");

        mLooper = new Looper();
        mLidarProcessor = new LidarProcessor();
        mLooper.register(mLidarProcessor);
        boolean started = mLidarProcessor.isConnected();
        if(!started)
            return;

        double ts = System.currentTimeMillis() / 1000d;
        Pose2d zeroPose = new Pose2d();
        sPoses.put(new InterpolatingDouble(ts), zeroPose);
        mLooper.start();
        while (true)
        {
            if (mLidarProcessor.isConnected())
            {
                try
                {
                    Pose2d p;
                    if(mode == OperatingMode.kRelative)
                    {
                        p = mLidarProcessor.doRelativeICP();
                        Logger.debug("relativeICP: " + p.toString());
                    } 
                    else
                    {
                        p = mLidarProcessor.doICP();
                        Logger.debug("absoluteICP: " + p.toString());
                    }
                    ts = System.currentTimeMillis() / 1000d;
                    // until robot is actually moving, we don't want
                    // to update robot pose. That is, we expect the
                    // "same" point cloud each iteration.
                    sPoses.put(new InterpolatingDouble(ts), zeroPose);
                    Thread.sleep(200); // 100 milliseconds -> 5hz (~LidarRate)
                }
                catch(Exception e)
                {
                    Logger.exception(e);
                }
            }
        }
    }

    public static Pose2d getRobotPose(double timestamp)
    {
        return sPoses.get(new InterpolatingDouble(timestamp));
    }
}
