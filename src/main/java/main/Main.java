package main;

import java.util.concurrent.TimeUnit;

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
    private static final int kObservationBufferSize = 100;
    public static void main(String[] args)
    {
        Logger.setVerbosity("DEBUG");

        mPoses = new InterpolatingTreeMap<InterpolatingDouble, Pose2d>(kObservationBufferSize);
        mPoses.put(new InterpolatingDouble(0.0), new Pose2d());

        mLooper = new Looper();
        Logger.debug("LIDAR starting...");
        mLidarProcessor = LidarProcessor.getInstance();
        mLooper.register(mLidarProcessor);
        boolean started = LidarServer.getInstance().start();
        Logger.debug("LIDAR status " + (started ? "started" : "failed to start"));

        mLooper.start();

        while (true)
        {
            if (LidarServer.getInstance().isLidarConnected())
                mPoses.put(new InterpolatingDouble(System.currentTimeMillis() / 1000d), mLidarProcessor.doICP());
        }
    }

    public static Pose2d getPose(double timestamp)
    {
        return mPoses.get(new InterpolatingDouble(timestamp));
    }
}
