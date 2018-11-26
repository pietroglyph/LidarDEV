package icp;

public interface IReferenceModel
{

    public Point getClosestPoint(Point p);

    public void transformBy(Transform t);
}
