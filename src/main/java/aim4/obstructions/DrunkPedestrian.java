package aim4.obstructions;

import aim4.util.GeomMath;

import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.*;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.awt.*;
import java.awt.geom.Point2D;

public class DrunkPedestrian
{
    private int id;                             //this drunk pedestrian's unique identification number
    private Rectangle2D.Double body;            //this drunk pedestrian's body
    private Point2D.Double startPoint;          //where this drunk pedestrian starts walking from
    private Point2D.Double endPoint;            //where this drunk pedestrian is heading
    private Point2D.Double currentPos;          //where this drunk pedestrian is currently
    private int direction;                      // = 1 or -1, depending on which direction to walk in
    private String walkingMode;                 //"VERTICAL" or "HORIZONTAL"

    public DrunkPedestrian(int id, Point2D.Double startPoint, Point2D.Double endPoint)
    {
        this.id = id;                                                                   //I'll pass in atomicIntX.getAndIncrement() as id
        this.startPoint = new Point2D.Double(startPoint.getX(),startPoint.getY());
        this.endPoint = new Point2D.Double(endPoint.getX(),endPoint.getY());
        currentPos = new Point2D.Double(startPoint.getX(),startPoint.getY());           //set initial current position to coordinates of startPoint
        body = new Rectangle2D.Double(currentPos.getX(),currentPos.getY(),1,1);         //posX, posY, width, height
        calculateDirection();
    }

    public DrunkPedestrian(int id, Point2D.Double startPoint, Point2D.Double endPoint, double width, double height)
    {
        this.id = id;                                                                       //I'll pass in atomicIntX.getAndIncrement() as id
        this.startPoint = new Point2D.Double(startPoint.getX(),startPoint.getY());
        this.endPoint = new Point2D.Double(endPoint.getX(),endPoint.getY());
        currentPos = new Point2D.Double(startPoint.getX(),startPoint.getY());                //set initial current position to coordinates of startPoint
        body = new Rectangle2D.Double(currentPos.getX(),currentPos.getY(),width, height);    //posX, posY, width, height
        calculateDirection();
    }

    public List<Line2D> getEdges()
    {
        return GeomMath.polygonalShapePerimeterSegments(body);
    }

    public void calculateDirection()
    {
        if(startPoint.x == endPoint.x)      //then this pedestrian will walk vertically
        {
            walkingMode = "VERTICAL";
            if(startPoint.y < endPoint.y)   //then start is above end, so we need to walk down
            {
                direction = 1;              // 1*timeStep will be a positive number, making us walk one step DOWN towards endpoint
            }
            if(startPoint.y > endPoint.y)   //then start is below end, so we need to walk up
            {
                direction = -1;              // -1*timeStep will be a negative number, making us walk one step UP towards endpoint
            }
        }

        if(startPoint.y == endPoint.y)  //if startY == endY then must walk horizontally
        {
            walkingMode = "HORIZONTAL";
            if(startPoint.x < endPoint.x)   //then start is to left of end, so we need to walk right
            {
                direction = 1;              // 1*timeStep will be a positive number, making us walk one step RIGHT towards endpoint
            }
            if(startPoint.x > endPoint.x)   //then start is to right of end, so we need to walk left
            {
                direction = -1;              // -1*timeStep will be a negative number, making us walk one step LEFT towards endpoint
            }
        }
    }


    public void walk(double timeStep)
    {
        if(walkingMode.equals("VERTICAL"))
        {
            if((getCurrentPos().getY() >= getEndPoint().getY()) && direction==1)  //if at or just past endpoint and walking towards DOWN
            {
                double endX = endPoint.x;
                double endY = endPoint.y;
                double startX = startPoint.x;
                double startY = startPoint.y;
                setEndPoint(startX,startY);               //swap start with end
                setStartPoint(endX,endY);                 //swap end with start
                direction = -1;                           //change direction to UP
            }

            if((getCurrentPos().getY() <= getEndPoint().getY()) && direction==-1)  //if at or just past endpoint and walking UP
            {
                double endX = endPoint.x;
                double endY = endPoint.y;
                double startX = startPoint.x;
                double startY = startPoint.y;
                setEndPoint(startX,startY);             //swap start with end
                setStartPoint(endX,endY);               //swap end with start
                direction = 1;                          //change direction to DOWN
            }

            currentPos.setLocation(currentPos.getX(),currentPos.getY()+(timeStep*direction)); //walk one timeStep on Y axis (up/down depends on direction variable)
        }

        if(walkingMode.equals("HORIZONTAL"))
        {
            if((getCurrentPos().getX() >= getEndPoint().getX()) && direction==1)  //if at or just past endpoint and walking RIGHT
            {
                double endX = endPoint.x;
                double endY = endPoint.y;
                double startX = startPoint.x;
                double startY = startPoint.y;
                setEndPoint(startX,startY);               //swap start with end
                setStartPoint(endX,endY);                 //swap end with start
                direction = -1;                           //change direction to LEFT
            }

            if((getCurrentPos().getX() <= getEndPoint().getX()) && direction==-1)  //if at or just past endpoint and walking LEFT
            {
                double endX = endPoint.x;
                double endY = endPoint.y;
                double startX = startPoint.x;
                double startY = startPoint.y;
                setEndPoint(startX,startY);               //swap start with end
                setStartPoint(endX,endY);                 //swap end with start
                direction = 1;                            //change direction to RIGHT
            }

            currentPos.setLocation(currentPos.getX()+(timeStep*direction),currentPos.getY()); //walk one timeStep on X axis (left/right depends on direction variable)
        }

        body.setRect(new Rectangle2D.Double(currentPos.getX(),currentPos.getY(),body.getWidth(),body.getHeight()));
    }


    public Rectangle2D.Double getBody()
    {
        return body;
    }

    //getters

    public Point2D.Double getStartPoint()
    {
        return this.startPoint;
    }

    public Point2D.Double getEndPoint()
    {
        return this.endPoint;
    }

    public int getId()
    {
        return id;
    }

    public Point2D.Double getCurrentPos()
    {
        return currentPos;
    }


    //setters

    public void setStartPoint(double x, double y)
    {
        this.startPoint.setLocation(x,y);
    }

    public void setStartPoint(Point2D.Double startPoint)
    {
        this.startPoint.setLocation(startPoint.getX(),startPoint.getY());
    }

    public void setEndPoint(double x, double y)
    {
        this.endPoint.setLocation(x,y);
    }

    public void setEndPoint(Point2D.Double endPoint)
    {
        this.endPoint.setLocation(endPoint.getX(),endPoint.getY());
    }

}