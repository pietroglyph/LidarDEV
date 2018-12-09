package lib.util;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.UUID;

/**
 * Tracks start-up and caught crash events, logging them to a file which dosn't
 * roll over
 */
public class Logger
{
    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();
    public static int sVerbosity = 0; // 0: notices and above,  1: info and above, 2: all
    private static final DateFormat s_dateFormat = new SimpleDateFormat("hh:mm:ss"); 

    public static void setVerbosity(String nm)
    {
        if(nm.equals("NOTICE"))
            sVerbosity = 0;
        else
        if(nm.equals("INFO"))
            sVerbosity = 1;
        else
        if(nm.equals("DEBUG"))
            sVerbosity = 2;
        else
            error("Logger: unknown verbosity level:" + nm);
    }
    
    public static void logRobotStartup()
    {
        notice("robot startup");
    }

    public static void logRobotConstruction()
    {
        notice("robot construction");
    }

    public static void logRobotInit()
    {
        notice("robot init");
    }
    
    public static void logTeleopInit()
    {
        notice("teleop init");
    }

    public static void logAutoInit()
    {
        notice("auto init");
    }

    public static void logDisabledInit()
    {
        notice("disabled init");
    }

    public static void logThrowableCrash(Throwable throwable)
    {
        logMarker("Exception", throwable, true);
    }

    public static void logThrowableCrash(String msg, Throwable throwable)
    {
        logMarker("ERROR   " + msg, throwable, true);
    }
    
    public static void exception(Exception e)
    {
        StringWriter sw = new StringWriter();
        e.printStackTrace(new PrintWriter(sw));
        logMarker("EXCEPT  " + e.getMessage() + 
                " trace:\n" +  sw.toString(), true);
    }

    public static void error(String m)
    {
        logMarker("ERROR   " + m, true);
    }

    public static void warning(String m)
    {
        logMarker("WARNING " + m, true);
    }

    public static void notice(String m)
    {
        logMarker("NOTICE   " + m, false);
    }

    public static void info(String m)
    {
        if (sVerbosity > 0)
        {
            printMarker("INFO    " + m, false);
        }
    }

    public static void debug(String m)
    {
        if (sVerbosity > 1)
        {
            printMarker("DEBUG   " + m, false);
        }
    }
    
    private static String getTimeStamp()
    {
        Date now = new Date();
        String nowstr = s_dateFormat.format(now) + " ";
        return nowstr;
    }

    private static void logMarker(String mark, boolean asError)
    {
        logMarker(mark, null,  asError);
    }

    private static void printMarker(String mark, boolean asError)
    {
        if(asError)
            System.err.println(mark);
        else
            System.out.println(mark);
    }

    private static void logMarker(String mark, Throwable nullableException, 
                                 boolean asError)
    {
        printMarker(mark, asError);
        if(nullableException != null)
            nullableException.printStackTrace();
        try (PrintWriter writer = new PrintWriter(new FileWriter(Paths.get(System.getProperty("user.home"), "crash_tracking.txt").toString(), true)))
        {
            writer.print(RUN_INSTANCE_UUID.toString());
            writer.print(", ");
            writer.print(mark);
            if (nullableException != null)
            {
                writer.print(", ");
                nullableException.printStackTrace(writer);
            }
            writer.println();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }
}
