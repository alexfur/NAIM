package aim4;

import aim4.util.ShapeUtils;

import javax.sound.sampled.Line;
import java.awt.geom.Area;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

public class CheckPoint
{
    private Point2D.Double p1;
    private Point2D.Double p2;
    private Line2D.Double line;
    private Area area;
    private float lineWidth = 1;
    private int id;

    public CheckPoint(double x1, double y1, double x2, double y2)
    {
        this.p1 = new Point2D.Double(x1,y1);
        this.p2 = new Point2D.Double(x2,y2);
        line = new Line2D.Double(p1,p2);
        area = new Area(ShapeUtils.getLineShape(line,lineWidth));
    }

    public CheckPoint(Point2D.Double p1, Point2D.Double p2)
    {
        this.p1 = p1;
        this.p2 = p2;
        line = new Line2D.Double(p1,p2);
        area = new Area(ShapeUtils.getLineShape(line,lineWidth));
    }

    public CheckPoint(Point2D.Double p1, Point2D.Double p2, int id)
    {
        this.p1 = p1;
        this.p2 = p2;
        line = new Line2D.Double(p1,p2);
        area = new Area(ShapeUtils.getLineShape(line,lineWidth));
        this.id = id;
    }

    public CheckPoint(double x1, double y1, double x2, double y2, int id)
    {
        this.p1 = new Point2D.Double(x1,y1);
        this.p2 = new Point2D.Double(x2,y2);
        line = new Line2D.Double(p1,p2);
        area = new Area(ShapeUtils.getLineShape(line,lineWidth));
        this.id = id;
    }

    public synchronized Area getArea()
    {
        return this.area;
    }

    public synchronized Line2D.Double getLine()
    {
        return this.line;
    }

    public CheckPoint(){}

}