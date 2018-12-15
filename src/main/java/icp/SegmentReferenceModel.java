package icp;

import java.util.Collection;

public class SegmentReferenceModel implements IReferenceModel
{
    public final Segment[] segments;

    public SegmentReferenceModel(Segment... ss)
    {
        if (ss.length == 0)
            throw new IllegalArgumentException("zero Segments passed to ReferenceModel");
        segments = ss;
    }

    public SegmentReferenceModel(Collection<Segment> ss)
    {
        this(ss.toArray(new Segment[ss.size()]));
    }

    public Point getClosestPoint(Point p)
    {
        double minDist = Double.MAX_VALUE;
        Segment minSeg = null;
        for (Segment s : segments)
        {
            double dist = s.getDistanceSq(p);
            if (dist < minDist)
            {
                minDist = dist;
                minSeg = s;
            }
        }
        return minSeg.getClosestPoint(p);
    }

    public void transformBy(Transform t)
    {
        for (int i = 0; i < segments.length; i++)
        {
            segments[i] = t.apply(segments[i]);
        }
    }

    public String toString()
    {
        String str = "[";
        for (Segment s : segments)
        {
            str += "\n  " + s;
        }
        return str + "\n]";
    }

}
