package lib;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import lib.Constants;

import lib.util.CrashTrackingRunnable;

//import com.spartronics4915.lib.util.Logger;
import lib.util.Logger;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List
 * object. They are started when the robot
 * powers up and stopped after the match.
 */
public class Looper
{

    public final double kPeriodMS = Constants.kLooperDtMS;

    private boolean running_;

    private ScheduledExecutorService scheduler_ = Executors.newScheduledThreadPool(1);
    private final List<Loop> loops_;
    private final Object taskRunningLock_ = new Object();
    private double timestamp_ = 0;
    private double dt_ = 0;

    private final CrashTrackingRunnable runnable_ = new CrashTrackingRunnable()
    {

        @Override
        public void runCrashTracked()
        {
            synchronized (taskRunningLock_)
            {
                if (running_)
                {
                    double now = System.currentTimeMillis() / 1000d;

                    for (Loop loop : loops_)
                    {
                        loop.onLoop(now);
                    }

                    dt_ = now - timestamp_;
                    timestamp_ = now;
                }
            }
        }
    };

    public Looper()
    {
        running_ = false;
        loops_ = new ArrayList<>();
    }

    public synchronized void register(Loop loop)
    {
        synchronized (taskRunningLock_)
        {
            loops_.add(loop);
        }
    }

    public synchronized void start()
    {
        if (!running_)
        {
            Logger.notice("Looper starting subsystem loops");
            synchronized (taskRunningLock_)
            {
                timestamp_ = System.currentTimeMillis() / 1000d;
                for (Loop loop : loops_)
                {
                    loop.onStart(timestamp_);
                }
                running_ = true;
            }
            scheduler_.shutdownNow();
            while (!scheduler_.isTerminated())
            {} // Woo possibly infinite loops
            scheduler_ = Executors.newScheduledThreadPool(1); // XXX: Who knows if this works
            scheduler_.scheduleAtFixedRate(runnable_, 
                            0 /* Initial delay of 0 ms */, 
                            (long) kPeriodMS, 
                            TimeUnit.MILLISECONDS);
        }
    }

    public synchronized void stop()
    {
        if (running_)
        {
            Logger.notice("Looper stopping subsystem loops");
            scheduler_.shutdownNow();
            synchronized (taskRunningLock_)
            {
                running_ = false;
                timestamp_ = System.currentTimeMillis() / 1000d;
                for (Loop loop : loops_)
                {
                    Logger.notice("Looper stopping " + loop);
                    loop.onStop(timestamp_);
                }
            }
        }
    }

}
