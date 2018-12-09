package main;

import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.geom.Dimension2D;
import java.awt.geom.Point2D;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

import javax.swing.JFrame;
import javax.swing.JPanel;

import lib.Constants;
import lib.Looper;
import lib.util.InterpolatingDouble;
import lib.util.InterpolatingTreeMap;
import lib.util.Logger;
import lib.math.Pose2d;
import lib.math.Translation2d;
import lidar.LidarProcessor;

public class Main
{
    private static final int kPoseMapSize = 100;
    private static InterpolatingTreeMap<InterpolatingDouble, Pose2d> sPoses =
            new InterpolatingTreeMap<InterpolatingDouble, Pose2d>(kPoseMapSize);
    private static LidarDebugPanel mDebugDisplayPanel = null;

    public static void main(String[] args)
    {
        Looper mLooper;
        LidarProcessor mLidarProcessor;
        Logger.setVerbosity("DEBUG");

        if (args.length > 0 && args[0].equals("--display")) {
            Logger.debug("Displaying debug window...");
            JFrame debugDisplayFrame = new JFrame("Spartronics Lidar Debug");
            mDebugDisplayPanel = new LidarDebugPanel();
            debugDisplayFrame.add(mDebugDisplayPanel);
            debugDisplayFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            debugDisplayFrame.setVisible(true);
        }

        mLooper = new Looper();
        mLidarProcessor = new LidarProcessor();
        mLooper.register(mLidarProcessor);
        boolean started = mLidarProcessor.isConnected();
        if(!started)
            return;

        Runtime.getRuntime().addShutdownHook(new Thread() {
            @Override
            public void run() {
                synchronized (mLooper) {
                    mLooper.stop();
                    Runtime.getRuntime().halt(0); // System.exit will wait on this thread to finish
                }
            }
        });

        Pose2d zeroPose = new Pose2d();
        double ts = System.currentTimeMillis() / 1000d;
        sPoses.put(new InterpolatingDouble(ts), zeroPose);
        mLooper.start();
        while (true)
        {
            try
            {
                // Lidar Processing occurs in LidarProcessor
                Thread.sleep(200);
            }
            catch(Exception e)
            {
                Logger.exception(e);
            }
        }
    }

    public static Pose2d getRobotPose(double timestamp)
    {
        return sPoses.get(new InterpolatingDouble(timestamp));
    }

    public static void putRobotPose(double timestamp, Pose2d pose)
    {
        sPoses.put(new InterpolatingDouble(timestamp), pose);
        if (mDebugDisplayPanel != null)
            mDebugDisplayPanel.setPose(pose);
        Logger.debug(pose.toString());
    }

    private static class LidarDebugPanel extends JPanel
    {
        private Translation2d lastTranslation = new Translation2d(), currentTranslation = new Translation2d();

        public void setPose(Pose2d pose)
        {
            lastTranslation = normalizeToGraphicsCoords(currentTranslation);
            currentTranslation = normalizeToGraphicsCoords(pose);
        }

        @Override
        protected void paintComponent(Graphics g)
        {
            g.drawLine((int) lastTranslation.getTranslation().x(), (int) lastTranslation.getTranslation().y(),
                (int) currentTranslation.getTranslation().x(), (int) currentTranslation.getTranslation().y());
        }

        private Translation2d normalizeToGraphicsCoords(Pose2d pose)
        {
            return normalizeToGraphicsCoords(pose.getTranslation());
        }

        private Translation2d normalizeToGraphicsCoords(Translation2d translation)
        {
            // Assumes we won't overflow
            return new Translation2d(Math.round((translation.x() / Constants.kFieldWidth) * this.getX()),
                Math.round((translation.y() / Constants.kFieldHeight) * this.getY()));
        }
    }
}
