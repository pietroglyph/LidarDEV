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
import java.util.concurrent.LinkedBlockingQueue;
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
    enum OperatingMode
    {
        kRelative,
        kAbsolute
    };

    private boolean mDebugPoints=true;
    private LidarServer mLidarServer;
    private double mScanTime;
    private double mLastScanTime;
    private double mScanTimeAccum;
    private int mScanCount;
    private ICP mICP; 
    private RelativeICPProcessor mRelativeICP; 
    private DataOutputStream mDataLogFile;
    private final ReadWriteLock mRWLock; 
    private LinkedBlockingQueue<LidarScan> mScanQueue;
    private LidarScan mActiveScan;
    private final OperatingMode mode = OperatingMode.kRelative;

    public LidarProcessor() 
    {
        Logger.debug("LidarProcessor starting...");
        mICP = new ICP(100);
        mScanQueue = new LinkedBlockingQueue<LidarScan>();
        mRelativeICP = new RelativeICPProcessor(mICP);
        mRWLock = new ReentrantReadWriteLock();
        mLidarServer = new LidarServer(this);
        mScanTime = Double.NEGATIVE_INFINITY;
        mLastScanTime = Double.NEGATIVE_INFINITY;
        mScanTimeAccum = 0;
        mScanCount = 0;
        mActiveScan = null;
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

    @Override
    public void onStart(double timestamp) 
    {
    }

    @Override
    public void onLoop(double timestamp) 
    {
        // we're called regularly (100hz) from the looper. 
        if (timestamp - getScanStart() > Constants.kLidarRestartTime) 
        {
            if (!mLidarServer.isEnding() && !mLidarServer.isRunning()) 
            {
                if(!mLidarServer.start())   
                {
                    // If server fails to start we update mScanTime to 
                    // ensure we don't restart each loop.
                    startNewScan(timestamp);
                }
            }
        }
        if(mLidarServer.isRunning())
        {
            try
            {
                LidarScan scan = mScanQueue.take(); // blocks
                double scanTime = scan.getTimestamp();
                if(mScanCount > 0)
                    mScanTimeAccum += scanTime - mLastScanTime;
                if(mScanCount%10 == 1)
                {
                    double scansPerSec = mScanCount/mScanTimeAccum;
                    // we might want to log this to SmartDashboard
                    Logger.notice("scan " + mScanCount + 
                                  " npts:" + scan.getPoints().size() +
                                  " scansPerSec:"+ scansPerSec);
                }
                mScanCount++;
                mLastScanTime = scanTime;
                this.processLidarScan(scan);
            }
            catch(InterruptedException ie)
            {
            }
        }
    }

    @Override
    public void onStop(double timestamp)
    {
        mLidarServer.stop();
    }

    private void processLidarScan(LidarScan scan)
    {
        try
        {
            Pose2d p;
            if(mode == OperatingMode.kRelative)
            {
                p = this.doRelativeICP(scan);
                // Logger.debug("relativeICP: " + p.toString());
            } 
            else
            {
                p = this.doICP(scan);
                Logger.debug("absoluteICP: " + p.toString());
            }
            // until robot is actually moving, we don't want
            // to update robot pose. That is, we expect the
            // "same" point cloud each iteration.
            // sPoses.put(new InterpolatingDouble(ts), zeroPose);
        }
        catch(Exception e)
        {
            Logger.exception(e);
        }
    }

    private Pose2d doICP(LidarScan scan)
    {
        Pose2d guess = Main.getRobotPose(scan.getTimestamp());
        Transform xform = mICP.doICP(getCulledPoints(scan), 
                                new Transform(guess).inverse(), 
                                ReferenceModel.TOWER);
        return xform.inverse().toPose2d();
    }

    private Pose2d doRelativeICP(LidarScan scan) 
    {
        Transform xform = mRelativeICP.doRelativeICP(
                                    scan.getPoints(), 
                                    new Pose2d()/*vehicleToLidar TBD*/ );
        return xform.inverse().toPose2d();
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

    // logPoint is invoked from 
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
        Translation2d cartesian = lpt.toCartesian(); // converts to field position
        logPoint(lpt.angle, lpt.distance, cartesian.x(), cartesian.y());
        if (newScan || mActiveScan == null) 
        { 
            if(mActiveScan != null)
                mScanQueue.add(mActiveScan); // <- send it to consumer

            mActiveScan  = new LidarScan();
            startNewScan(System.currentTimeMillis() / 1000d);
        }
        if (!excludePoint(cartesian.x(), cartesian.y())) 
        {
            mActiveScan.addPoint(new Point(cartesian), lpt.timestamp);
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

    private static final double BUCKET_SIZE = 3.0; // inches

    /**
     * Cantor pairing function (to bucket & hash two doubles)
     * converts two integers into one; used as a hash key
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
    private ArrayList<Point> getCulledPoints(LidarScan scan)
    {
        ArrayList<Point> list = new ArrayList<>();
        HashSet<Integer> buckets = new HashSet<>();
        for (Point p : scan.getPoints())
        {
            if (buckets.add(getBucket(p.x, p.y)))
                list.add(p);
        }
        return list;
    }

    public void startNewScan(double time) 
    {
        mRWLock.writeLock().lock();
        try 
        {
            mScanTime = time;
        } 
        finally 
        {
            mRWLock.writeLock().unlock();
        }
    }

    public double getScanStart() 
    {
        mRWLock.readLock().lock();
        try 
        {
            return mScanTime;
        } 
        finally 
        {
            mRWLock.readLock().unlock();
        }
    }

}
