package lidar;

import lib.Constants;
//import com.spartronics4915.frc2019.RobotState;

import icp.ICP;
import icp.Point;

import icp.ReferenceModel;
import icp.RelativeICPProcessor;
import icp.Transform;

import lib.Loop;

import lib.math.Translation2d;
import lib.math.Pose2d;
import lib.util.Logger;

import main.Main;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.zip.GZIPOutputStream;

/**
 * Receives LIDAR points from the {@link LidarServer}, stores a set number of
 * scans/revolutions, and provides methods for processing the data.
 * <p>
 * All interfacing with the LIDAR should be done through this class.
 *
 * @see Constants.kLidarNumScansToStore
 * @see doICP()
 * @see getTowerPosition()
 */
public class LidarProcessor implements Loop 
{
    private LidarServer mLidarServer;
    private boolean mDebugPoints=true;

    private LinkedList<LidarScan> mScans = new LinkedList<>();
    private double prev_timestamp;

    private ICP mICP = new ICP(100);
    private RelativeICPProcessor mRelativeICP = new RelativeICPProcessor(mICP);

    private DataOutputStream mDataLogFile;

    private final ReadWriteLock mRWLock = new ReentrantReadWriteLock();

    public LidarProcessor() 
    {
        Logger.debug("LIDAR starting...");
        mLidarServer = new LidarServer(this);
        mScans.add(new LidarScan());
        try 
        {
            if(mDebugPoints)
                mDataLogFile = new DataOutputStream(newLogFile());
            else
                mDataLogFile = new DataOutputStream(new GZIPOutputStream(newLogFile()));
        } 
        catch (IOException e) 
        {
            Logger.exception(e);
        }
    }

    public boolean isConnected()
    {
        return mLidarServer.isLidarConnected();
    }

    private static FileOutputStream newLogFile() throws IOException 
    {
        // delete old files if we're over the limit
        File logDir = new File(Constants.kLidarLogDir);
        File[] logFiles = logDir.listFiles();
        if (logFiles == null)
            throw new IOException("List files in " + Constants.kLidarLogDir);
        Arrays.sort(logFiles, (f1, f2) -> {
            return Long.compare(f1.lastModified(), f2.lastModified());
        });
        for (int i=0; i<logFiles.length-Constants.kNumLidarLogsToKeep+1;i++) 
        {
            logFiles[i].delete();
        }

        // create the new file and return
        String dateStr = new SimpleDateFormat("MM-dd-HH_mm_ss").format(new Date());
        File newFile = new File(logDir, "lidarLog-" + dateStr + ".dat");
        newFile.createNewFile();
        return new FileOutputStream(newFile, false);
    }

    private void logPoint(double angle, double dist, double x, double y)
    {
        try 
        {
            mDataLogFile.writeInt((int) (angle * 100));
            mDataLogFile.writeInt((int) (dist * 256));
            mDataLogFile.writeInt((int) (x * 256));
            mDataLogFile.writeInt((int) (y * 256));
        } 
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    // addPoint is invoked from LidarServer::handleLine via the ReaderThread.
    // logging is only invoked from this thread, but scan data is accessed
    // asynchronously from the main thread (which, for example, performs
    // ICP matching). This accounts for the use of our read-write lock on the
    // scan data.  Any changes to mScans or an individual scan should
    // be guarded by this lock.
    public void addPoint(double ts, double angle, double dist, 
                                      boolean newScan) 
    {
        LidarPoint lpt = new LidarPoint(ts, angle, dist);
        Translation2d cartesian = lpt.toCartesian();
        logPoint(lpt.angle, lpt.distance, cartesian.x(), cartesian.y());

        mRWLock.writeLock().lock();
        try 
        {
            if (newScan) // crosses the 360-0 threshold. start a new scan
            { 
                prev_timestamp = System.currentTimeMillis() / 1000d;

                // long start = System.nanoTime();
                // Translation2d towerPos = getTowerPosition();
                // long end = System.nanoTime();
                // SmartDashboard.putNumber("towerPos_ms", (end-start)/1000000);
                // SmartDashboard.putNumber("towerPosX", towerPos.x());
                // SmartDashboard.putNumber("towerPosY", towerPos.y());

                mScans.add(new LidarScan());
                if (mScans.size() > Constants.kLidarNumScansToStore) 
                {
                    mScans.removeFirst();
                }
            }

            if (!excludePoint(cartesian.x(), cartesian.y())) 
            {
                getCurrentScan().addPoint(new Point(cartesian), lpt.timestamp);

                // The point cloud output is relative to the robot's position, so it probably
                // won't look to good if you move the robot around.
            }
        }
        finally 
        {
            mRWLock.writeLock().unlock();
        }
    }

    private static final double FIELD_WIDTH = 27 * 12, FIELD_HEIGHT = 54 * 12;
    private static final double RECT_RX = FIELD_WIDTH / 5, RECT_RY = FIELD_HEIGHT / 2;
    private static final double FIELD_CX = FIELD_WIDTH / 2, FIELD_CY = FIELD_HEIGHT / 2;
    private static final double RECT_X_MIN = FIELD_CX - RECT_RX, RECT_X_MAX = FIELD_CX + RECT_RX,
            RECT_Y_MIN = FIELD_CY - RECT_RY, RECT_Y_MAX = FIELD_CY + RECT_RY;

    private static boolean excludePoint(double x, double y) 
    {
        return false;
        // return x < RECT_X_MIN || x > RECT_X_MAX || y < RECT_Y_MIN || y > RECT_Y_MAX;
    }

    private LidarScan getCurrentScan() 
    {
        return mScans.getLast();
    }

    private ArrayList<Point> getAllPoints() 
    {
        ArrayList<Point> list = new ArrayList<>();
        for (LidarScan scan : mScans) 
        {
            list.addAll(scan.getPoints());
        }
        return list;
    }

    private Point getAveragePoint()
    {
        double sumX = 0, sumY = 0;
        int n = 0;
        for (Point p : getAllPoints()) {
            sumX += p.x;
            sumY += p.y;
            n++;
        }
        return new Point(sumX / n, sumY / n);
    }

    private static final double BUCKET_SIZE = 3.0; // inches

    /**
     * Cantor pairing function (to bucket & hash two doubles)
     */
    private int getBucket(double x, double y)
    {
        int ix = (int) (x / BUCKET_SIZE);
        int iy = (int) (y / BUCKET_SIZE);
        int a = ix >= 0 ? 2 * ix : -2 * ix - 1;
        int b = iy >= 0 ? 2 * iy : -2 * iy - 1;
        int sum = a + b;
        return sum * (sum + 1) / 2 + a;
    }

    /**
     * Returns a list of points that have been thinned roughly uniformly.
     */
    private ArrayList<Point> getCulledPoints()
    {
        ArrayList<Point> list = new ArrayList<>();
        HashSet<Integer> buckets = new HashSet<>();
        for (Point p : getAllPoints()) {
            if (buckets.add(getBucket(p.x, p.y)))
                list.add(p);
        }
        return list;
    }

    public Pose2d doICP()
    {
        mRWLock.readLock().lock(); // holds lock on all addPoint 
        try
        {
            Pose2d guess = Main.getRobotPose(getCurrentScan().getTimestamp());
            Transform xform = mICP.doICP(getCulledPoints(), 
                                new Transform(guess).inverse(), 
                                ReferenceModel.TOWER);
            return xform.inverse().toPose2d();
        }
        finally 
        {
            mRWLock.readLock().unlock();
        }
    }

    public Pose2d doRelativeICP() 
    {
        mRWLock.readLock().lock(); // holds lock on all addPoint 
        try 
        {
            Transform xform = mRelativeICP.doRelativeICP(
                                    getCurrentScan().getPoints(), 
                                    new Pose2d()/*vehicleToLidar*/ );
            return xform.inverse().toPose2d();
        } 
        finally 
        {
            mRWLock.readLock().unlock();
        }
    }

    // public Translation2d getTowerPosition() {
    //     lock.readLock().lock();
    //     try {
    //         Point avg = getAveragePoint();
    //         Transform trans = icp.doICP(getCulledPoints(), new Transform(0, avg.x, avg.y), ReferenceModel.TOWER);
    //         return trans.apply(icp.reference).getMidpoint().toTranslation2d();
    //     } finally {
    //         lock.readLock().unlock();
    //     }
    // }

    public void setPrevTimestamp(double time) 
    {
        mRWLock.writeLock().lock();
        try 
        {
            prev_timestamp = time;
        } 
        finally 
        {
            mRWLock.writeLock().unlock();
        }
    }

    public double getPrevTimestamp() 
    {
        mRWLock.readLock().lock();
        try 
        {
            return prev_timestamp;
        } 
        finally 
        {
            mRWLock.readLock().unlock();
        }
    }

    @Override
    public void onStart(double timestamp) 
    {
        // Singletons are bad. We ended up with circular dependencies
        // here because of them (if we do LidarServer.getInstance in
        // the constructor we get a stack overflow). I'm going to just
        // leave it like this for now, but I think we should refactor
        // singletons out of (at least) the Lidar* classes for the
        // actual robot code.
        setPrevTimestamp(Double.NEGATIVE_INFINITY);
    }

    @Override
    public void onLoop(double timestamp) 
    {
        // we're called regularly from  the looper, since we have
        // a subthread for reading the lidar, we do nother here except
        // ensure that the subprocess hasn't died.
        if (timestamp - getPrevTimestamp() > Constants.kLidarRestartTime) 
        {
            if (!mLidarServer.isEnding() && !mLidarServer.isRunning()) 
            {
                if (mLidarServer.start()) 
                {
                    setPrevTimestamp(timestamp);
                } 
            }
        }
    }

    @Override
    public void onStop(double timestamp)
    {
        mLidarServer.stop();
    }
}
