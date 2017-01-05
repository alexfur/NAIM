package aim4.vehicle;

import aim4.Main;
import aim4.config.Constants;
import aim4.im.RoadBasedIntersection;
import aim4.obstructions.DrunkPedestrian;
import aim4.util.LineIterator;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;

public class NeatSensor extends Sensor
{
    volatile private double degreesBetweenPtsFOV = Main.cfgDegreesBetweenPtsFOV;
    volatile private double distanceToIntersectionEdge = 0; //default
    private AtomicBoolean inIntersection = new AtomicBoolean(false);
    volatile private double distToIntersectionExitPoint;
    volatile private Point2D.Double intersectionExitPoint;


    public NeatSensor(VehicleSimView vehicle, Point2D.Double pointForDistFOV, double rad)
    {
        super(vehicle, pointForDistFOV, rad);
    }

    public NeatSensor(){}          //no arg constructor

    public NeatSensor(VehicleSimView vehicle, double width, double height, double angleStart, double angleExtent, Point2D.Double pointForDistFOV, double rad)
    {
        super(vehicle,width,height,angleStart,angleExtent, pointForDistFOV, rad);
    }

    synchronized public double getAcceleration()
    {
        return getVehicle().getAcceleration();
    }

    synchronized public double getVelocity()
    {
        return getVehicle().getVelocity();
    }

    public synchronized double getDistanceToNearestObstruction()
    {
        if(getObstructionsInFOV().isEmpty()) return 0;  //todo: return 0 or 1 ???

        double shortestDistance = getDistanceToObstruction(getObstructionsInFOV().get(0));
        for(Object obstruction : getObstructionsInFOV())
        {
            double currentDistance = getDistanceToObstruction(obstruction);
            if(currentDistance < shortestDistance)
                shortestDistance = currentDistance;
        }
        return shortestDistance;
    }

    public synchronized Line2D getRayToNearestObstruction()   //get the nearest point on the obstruction w.r.t the middle front point of housing vehicle
    {
        int index = 0;
        int i = 0;

        Line2D ray = getRayToObstruction(getObstructionsInFOV().get(0));
        double shortestDistance = ray.getP1().distance(ray.getP2());

        for(Object obstruction : getObstructionsInFOV())
        {
            ray = getRayToObstruction(obstruction);
            double currentDistance = ray.getP1().distance(ray.getP2());

            if(currentDistance < shortestDistance)
            {
                index = i;
                shortestDistance = currentDistance;
            }
            i++;
        }

        Object obstruction = getObstructionsInFOV().get(index);

        if(obstruction.getClass().equals(DrunkPedestrian.class))
        {
            return ray;                                         //p1 = front mid of vehicle, p2 = nearest point on nearest obstruction
        }

        if(obstruction.getClass().equals(BasicAutoVehicle.class))
        {
            return ray;                                         //p1 = front mid of vehicle, p2 = nearest point on nearest obstruction
        }

        return null;
    }

    public synchronized double getDistanceToObstruction(Object obstruction)
    {
        Line2D.Double ray = getRayToObstruction(obstruction);
        return ray.getP1().distance(ray.getP2());    //distance formula
    }

    public synchronized Line2D.Double getRayToObstruction(Object obstruction)  //should I get nearest point on housing vehicle too?
    {
        double x1 = getRad()==0 ? getVehicle().getPointAtMiddleFront(Constants.DOUBLE_EQUAL_PRECISION).getX() : -(Main.cfgVehicleWidth/2)*Math.sin(getVehicle().getHeading())+getVehicle().getCenterPoint().getX();
        double y1 = getRad()==0 ? getVehicle().getPointAtMiddleFront(Constants.DOUBLE_EQUAL_PRECISION).getY() : (Main.cfgVehicleWidth/2)*Math.cos(getVehicle().getHeading())+getVehicle().getCenterPoint().getY();
        double x2 = x1;  //this value will update if we find nearest distance to any obstructions
        double y2 = y1;  //this value will update if we find nearest distance to any obstructions

        Point2D rayStartPoint = new Point2D.Double(x1,y1);  //point from middle front of vehicle
        Point2D p2;                                         //nearest point on obstruction (which we will find in the next few lines of code)

        Point2D frontCornerRight = getVehicle().getCornerPoints()[0];
        Point2D frontCornerLeft = getVehicle().getCornerPoints()[3];

        if(obstruction.getClass().equals(BasicAutoVehicle.class))
        {
            BasicAutoVehicle vehicleB = (BasicAutoVehicle)obstruction;
            if (vehicleB.getVIN() != getVehicleVin())
            {
                ConcurrentHashMap<Double, Point2D> distancesPointsMap = new ConcurrentHashMap<Double, Point2D>();                             //distance between front of sensor-housing vehicle and nearest point on obstructing/detected vehicle
                for (Line2D edge : vehicleB.getEdges())
                {
                    Point2D currentPoint;
                    double distanceMiddle;
                    for (Iterator<Point2D> iter = new LineIterator(edge, Main.cfgNearestObstructionPrecision); iter.hasNext(); )                                    //iterate through points on edge
                    {
                        if (iter.hasNext())                                                                                 //if there are more points on this edge to traverse
                        {
                            //if(getRad()!=0)
                            {
                                currentPoint = iter.next();                                                                      //get the next point along the edge
                                distanceMiddle = rayStartPoint.distance(currentPoint);                                           //get the euclidean distance from middle front of vehicle to the current point on edge
                                if(getArea().contains(currentPoint))
                                    distancesPointsMap.put(distanceMiddle, currentPoint);
                            }
                            /*
                            else
                            {
                                double distanceLeft;
                                double distanceRight;

                                currentPoint = iter.next();                                                                      //get the next point along the edge
                                distanceMiddle = rayStartPoint.distance(currentPoint);                                           //get the euclidean distance from middle front of vehicle to the current point on edge
                                distanceLeft = frontCornerLeft.distance(currentPoint);
                                distanceRight = frontCornerRight.distance(currentPoint);
                                if (getArea().contains(currentPoint))
                                {
                                    if (distanceMiddle < distanceLeft && distanceMiddle < distanceRight)
                                    {
                                        distancesPointsMap.put(distanceMiddle, currentPoint);
                                    } else if (distanceLeft < distanceMiddle && distanceLeft < distanceRight)
                                    {
                                        distancesPointsMap.put(distanceLeft, currentPoint);
                                        rayStartPoint = frontCornerLeft;
                                    } else if (distanceRight < distanceMiddle && distanceRight < distanceLeft)
                                    {
                                        distancesPointsMap.put(distanceRight, currentPoint);
                                        rayStartPoint = frontCornerRight;
                                    }
                                }
                            }
                            */
                        }
                    }
                }
                double shortestDistance;
                try
                {
                    shortestDistance = Collections.min(distancesPointsMap.keySet());
                    x2 = distancesPointsMap.get(shortestDistance).getX();
                    y2 = distancesPointsMap.get(shortestDistance).getY();
                } catch (Exception e){}

                p2 = new Point2D.Double(x2,y2);
                return new Line2D.Double(rayStartPoint,p2);
            }
        }

        if(obstruction.getClass().equals(DrunkPedestrian.class))
        {
            DrunkPedestrian drunkPedestrian = (DrunkPedestrian) obstruction;
            ConcurrentHashMap<Double,Point2D> distancesPointsMap = new ConcurrentHashMap<Double, Point2D>();                //distance between front of sensor-housing vehicle and nearest point on obstruction

            for(Line2D edge : drunkPedestrian.getEdges())
            {
                Point2D currentPoint;
                double distanceMiddle;

                for(Iterator<Point2D> iter = new LineIterator(edge,Main.cfgNearestObstructionPrecision); iter.hasNext();)                                    //iterate through points on edge
                {
                    if(iter.hasNext())                                                                                   //if there are more points on this edge to traverse
                    {
                        //if(getRad()!=0)
                        {
                            currentPoint = iter.next();                                                                      //get the next point along the edge
                            distanceMiddle = rayStartPoint.distance(currentPoint);                                           //get the euclidean distance from middle front of vehicle to the current point on edge
                            if(getArea().contains(currentPoint))
                                distancesPointsMap.put(distanceMiddle, currentPoint);
                        }
                        /*
                        else
                        {
                            double distanceLeft;
                            double distanceRight;

                            currentPoint = iter.next();                                                                      //get the next point along the edge
                            distanceMiddle = rayStartPoint.distance(currentPoint);                                           //get the euclidean distance from middle front of vehicle to the current point on edge
                            distanceLeft = frontCornerLeft.distance(currentPoint);
                            distanceRight = frontCornerRight.distance(currentPoint);
                            if (getArea().contains(currentPoint))
                            {
                                if (distanceMiddle < distanceLeft && distanceMiddle < distanceRight)
                                {
                                    distancesPointsMap.put(distanceMiddle, currentPoint);
                                } else if (distanceLeft < distanceMiddle && distanceLeft < distanceRight)
                                {
                                    distancesPointsMap.put(distanceLeft, currentPoint);
                                    rayStartPoint = frontCornerLeft;
                                } else if (distanceRight < distanceMiddle && distanceRight < distanceLeft)
                                {
                                    distancesPointsMap.put(distanceRight, currentPoint);
                                    rayStartPoint = frontCornerRight;
                                }
                            }
                        }
                        */
                    }
                }
            }
            double shortestDistance;
            try
            {
                shortestDistance = Collections.min(distancesPointsMap.keySet());            //get the smallest distance (key) in the map
                x2 = distancesPointsMap.get(shortestDistance).getX();
                y2 = distancesPointsMap.get(shortestDistance).getY();
            }
            catch(Exception e){}

            p2 = new Point2D.Double(x2,y2);
            return new Line2D.Double(rayStartPoint,p2);
        }


        if(obstruction instanceof RoadBasedIntersection) //nearest distance to edges of intersection (once inside intersection) --> use to prevent from driving out of intersection onto grass
        {
            RoadBasedIntersection intersection = (RoadBasedIntersection) obstruction;
            ConcurrentHashMap<Double,Point2D> distancesPointsMap = new ConcurrentHashMap<Double, Point2D>();                //distance between front of sensor-housing vehicle and nearest point on obstruction

            for(Line2D edge : intersection.getLineEdges())
            {
                Point2D currentPoint;
                double distance;
                for(Iterator<Point2D> iter = new LineIterator(edge,Main.cfgNearestObstructionPrecision); iter.hasNext();)                                    //iterate through points on edge
                {
                    if(iter.hasNext())                                                                                   //if there are more points on this edge to traverse
                    {
                        currentPoint = iter.next();                                                                      //get the next point along the edge
                        distance = rayStartPoint.distance(currentPoint);                                          //get the euclidean distance from middle front of vehicle to the current point on edge
                        if(getArea().contains(currentPoint))                                                             //make sure the point is in the FOV area
                            distancesPointsMap.put(distance, currentPoint);
                    }
                }
            }
            double shortestDistance;
            try
            {
                shortestDistance = Collections.min(distancesPointsMap.keySet());                                        //get the smallest distance (key) in the map
                x2 = distancesPointsMap.get(shortestDistance).getX();
                y2 = distancesPointsMap.get(shortestDistance).getY();
            }
            catch(Exception e){}

            p2 = new Point2D.Double(x2,y2);
            return new Line2D.Double(rayStartPoint,p2);
        }

        p2 = new Point2D.Double(x2,y2);  //if we got this point, there weren't any obstructions in FOV - so both start & end points are the middle front of the vehicle, so distance = 0
        return new Line2D.Double(rayStartPoint,p2); //p1 == p2 if we got to this point
    }

    // traverse the arc of the FOV in steps of X degrees, add each current point to an array, and finally return the array
    public synchronized ArrayList<Point2D.Double> getFOVTurnPoints()
    {
        ArrayList<Point2D.Double> ptsOnFOV = new ArrayList<Point2D.Double>();

        double x = getVehicle().getCenterPoint().getX();
        double y = getVehicle().getCenterPoint().getY();
        double r = Main.cfgSensorWidth / 2;

        //draw lines from angle start to 0 (AKA 360)
        for (double i = Math.toRadians(Main.cfgSensorAngleStart); i < Math.toRadians(360); i+=Math.toRadians(degreesBetweenPtsFOV))
        {
            double theta = getVehicle().getHeading() + i;
            Point2D.Double p = new Point2D.Double(x + r*Math.cos(theta), y + r*Math.sin(theta));
            ptsOnFOV.add(p);
        }

        //draw lines from 0 to angle end
        for (double i = Math.toRadians(0); i < Math.toRadians(Main.cfgSensorAngleEnd); i+=Math.toRadians(degreesBetweenPtsFOV))
        {
            double theta = getVehicle().getHeading() + i;
            Point2D.Double p = new Point2D.Double(x + r*Math.cos(theta), y + r*Math.sin(theta));
            ptsOnFOV.add(p);
        }

        return ptsOnFOV;
    }

    public synchronized void setAccelWithMaxTargetVelocity(double targetAccel)       //set acceleration
    {
        getVehicle().setTargetVelocityWithMaxAccel(targetAccel);
    }

    public synchronized double getDistanceToIntersectionEdge()
    {
        return this.distanceToIntersectionEdge;
    }

    public synchronized void setDistanceToIntersectionEdge(double distanceToIntersectionEdge)
    {
        this.distanceToIntersectionEdge = distanceToIntersectionEdge;
    }

    public synchronized void turnToAngleInFOV(int pointIndex)      //turn towards one of the NN-input points on the arc of the FOV
    {
        getVehicle().turnTowardPoint(getFOVTurnPoints().get(pointIndex));
    }

    synchronized public static NeatSensor frontNeatSensorOn(VehicleSimView vehicle)
    {
        return (NeatSensor) ((BasicAutoVehicle) vehicle).getFrontSensor();
    }

    synchronized public static NeatSensor leftNeatSensorOn(VehicleSimView vehicle)
    {
        return (NeatSensor) ((BasicAutoVehicle) vehicle).getLeftSensor();
    }

    synchronized public static NeatSensor rightNeatSensorOn(VehicleSimView vehicle)
    {
        return (NeatSensor) ((BasicAutoVehicle) vehicle).getRightSensor();
    }

    public synchronized void setInIntersection(boolean inIntersection)  //set whether or not this sensor's car is in an intersection
    {
        this.inIntersection.set(inIntersection);
    }

    public synchronized boolean getInIntersection()         //get whether or not this sensor's car is in an intersection
    {
        return this.inIntersection.get();
    }

}