package main;

import lib.Looper;
import lib.util.InterpolatingDouble;
import lib.util.InterpolatingTreeMap;
import lib.util.Logger;
import lib.math.Pose2d;
import lidar.LidarProcessor;
import lidar.LidarServer;

public class Main
{
    private static Looper mLooper;
    private static LidarProcessor mLidarProcessor;
    private static InterpolatingTreeMap<InterpolatingDouble, Pose2d> mPoses;
    public static void main(String[] args)
    {
        mLooper = new Looper();
        Logger.debug("LIDAR starting...");
        mLidarProcessor = LidarProcessor.getInstance();
        mLooper.register(mLidarProcessor);
        boolean started = LidarServer.getInstance().start();
        Logger.debug("LIDAR status" + (started ? "started" : "failed to start"));

        mLooper.start();

        while (true)
        {
            mPoses.put(new InterpolatingDouble((double) (System.currentTimeMillis() % 1000)), mLidarProcessor.doICP());
            Logger.debug("foo");
        }
    }

    public static Pose2d getPose(double timestamp)
    {
        return mPoses.get(timestamp);
    }
}