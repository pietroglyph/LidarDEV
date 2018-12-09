package main;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GridLayout;
import java.awt.Image;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Scanner;

import javax.swing.BorderFactory;
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

        if (args.length > 0 && args[0].equals("--debug")) {
            Logger.debug("Displaying debug window... Press p to pause collection at next scan.");
            JFrame debugDisplayFrame = new JFrame("Spartronics Lidar Debug");

            mDebugDisplayPanel = new LidarDebugPanel();
            debugDisplayFrame.add(mDebugDisplayPanel);

            debugDisplayFrame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
            debugDisplayFrame.pack();
            debugDisplayFrame.setLocationRelativeTo(null);
            debugDisplayFrame.setVisible(true);
            debugDisplayFrame.setExtendedState(JFrame.MAXIMIZED_BOTH);
        }

        mLooper = new Looper();
        mLidarProcessor = new LidarProcessor();
        mLooper.register(mLidarProcessor);
        boolean started = mLidarProcessor.isConnected();
        if(!started) {
            Logger.error("Lidar is not connected");
            System.exit(1);
        }

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

        Scanner stdin = new Scanner(System.in);
        while (true)
        {
            try
            {
                // Lidar Processing occurs in LidarProcessor
                Thread.sleep(200);
                if (stdin.hasNext("p"))
                {
                    stdin.next();
                    mLidarProcessor.togglePaused();
                }
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
        {
            mDebugDisplayPanel.setPose(pose);
            mDebugDisplayPanel.repaint();
        }
    }

    public static void logLidarPoint(Translation2d point)
    {
        if (mDebugDisplayPanel != null)
        {
            mDebugDisplayPanel.addPoint(point);
            mDebugDisplayPanel.repaint();
        }
    }

    private static class LidarDebugPanel extends JPanel
    {
        private Translation2d lastTranslation = new Translation2d(), currentTranslation = new Translation2d();
        private ArrayList<Translation2d> pointsToAdd = new ArrayList<Translation2d>();
        private Image imageBuffer;

        private static final int kLidarPointDiameter = 3; // In user space pixels

        public LidarDebugPanel()
        {
            super(new GridLayout(1, 1));
        }

        public void setPose(Pose2d pose)
        {
            lastTranslation = normalizeToGraphicsCoords(currentTranslation);
            currentTranslation = normalizeToGraphicsCoords(pose);
        }

        public void addPoint(Translation2d point)
        {
            synchronized (pointsToAdd)
            {
                pointsToAdd.add(normalizeToGraphicsCoords(point));
            }
        }

        @Override
        protected void paintComponent(Graphics g)
        {
            if (imageBuffer == null)
            {
                imageBuffer = this.createImage(this.getWidth(), this.getHeight());
                return;
            }
            Graphics bufGraphics = imageBuffer.getGraphics();

            synchronized (pointsToAdd)
            {
                for (int i = 0; i < pointsToAdd.size(); i++)
                {
                    bufGraphics.drawOval((int) pointsToAdd.get(i).x(), (int) pointsToAdd.get(i).y(), kLidarPointDiameter, kLidarPointDiameter);
                    pointsToAdd.remove(i);
                }
            }

            bufGraphics.setColor(Color.RED);
            bufGraphics.drawLine((int) lastTranslation.getTranslation().x(), (int) lastTranslation.getTranslation().y(),
                (int) currentTranslation.getTranslation().x(), (int) currentTranslation.getTranslation().y());
            bufGraphics.setColor(Color.BLACK);

            g.drawImage(imageBuffer, 0, 0, null);
        }

        private Translation2d normalizeToGraphicsCoords(Pose2d pose)
        {
            return normalizeToGraphicsCoords(pose.getTranslation());
        }

        private Translation2d normalizeToGraphicsCoords(Translation2d translation)
        {
            return new Translation2d(Math.round((translation.x() / Constants.kFieldWidth) * this.getWidth() + this.getWidth() / 2),
                Math.round((translation.y() / Constants.kFieldHeight) * this.getHeight() + this.getHeight() / 2));
        }
    }
}

