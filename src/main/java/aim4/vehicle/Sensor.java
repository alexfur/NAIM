package aim4.vehicle;

import aim4.Main;
import aim4.config.Constants;
import aim4.obstructions.DrunkPedestrian;
import aim4.util.ShapeUtils;

import java.awt.*;
import java.awt.geom.Arc2D;
import java.awt.geom.Area;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;


public class Sensor
{
    private VehicleSimView v;                                                    //vehicle housing this sensor
    private volatile double x;                                                   //x coordinate of arc on screen
    private volatile double y;                                                   //y coordinate of arc on screen
    private volatile double width       = Main.cfgSensorWidth;                   //width
    private volatile double height      = Main.cfgSensorHeight;                  //height
    private volatile double angleStart  = Main.cfgSensorAngleStart;              //angle start //-30
    private volatile double angleExtent = Main.cfgSensorAngleEnd;                //angle extent //60
    private volatile Area area;
    private volatile Arc2D.Double shape;
    private volatile int type = Arc2D.PIE;
    private volatile double distanceFOV;
    private AtomicBoolean obstructionDetected = new AtomicBoolean(false);
    private List<Object> obstructionsInFOV = Collections.synchronizedList(new ArrayList<Object>());
    private AtomicBoolean hasCrashed = new AtomicBoolean(false);
    private AtomicBoolean passedCheckPointOne = new AtomicBoolean(false);
    private AtomicBoolean passedCheckPointTwo = new AtomicBoolean(false);

    private volatile boolean successfulTraversal;             //boolean which tells NEAT if the car successfully traversed the intersection and reached its destination

    //constructor allowing to alter some properties of sensor's arc shape
    public Sensor(VehicleSimView vehicle, double width, double height, double angleStart, double angleExtent)
    {
            this.v = vehicle; //set the Sensor's vehicle object reference to the one being plugged into the constructor
            this.x = vehicle.getCenterPoint().getX(); //set y coordinate of sensor to y coordinate of front of vehicle
            this.y = vehicle.getCenterPoint().getY(); //set x coordinate of sensor to x coordinate of front of vehicle
            this.width = width;
            this.height = height;
            this.angleStart = angleStart;
            this.angleExtent = shiftAngleExtent(angleExtent);
            Arc2D.Double sensorConeOfSightTemp = new Arc2D.Double(v.getCenterPoint().getX(),v.getCenterPoint().getY(),width,height,angleStart,angleExtent,Arc2D.PIE); //create temp sensor cone
            double centX = v.getCenterPoint().getX() - (sensorConeOfSightTemp.getCenterX() - v.getCenterPoint().getX());   //correct the x centre value of temp sensor cone
            double centY = v.getCenterPoint().getY() - (sensorConeOfSightTemp.getCenterY() - v.getCenterPoint().getY());   //correct the y centre value of temp sensor cone
            Arc2D.Double sensorConeOfSight = new Arc2D.Double(centX,centY,width,height,angleStart,angleExtent,Arc2D.PIE);         //create a new sensor cone with the corrected centre coordinate
            this.shape = sensorConeOfSight;
            this.area = new Area(shape);                                                                                    //convert sensor cone to an area
            this.successfulTraversal = true;
            this.angleExtent = undoShiftAngleExtent(this.angleExtent);

            this.distanceFOV = getUpperMid().distance(getVehicle().getPointAtMiddleFront(Constants.DOUBLE_EQUAL_PRECISION));
    }


    //constructor with default arc shape properties - simply plug in a vehicle
    public Sensor(VehicleSimView vehicle)
    {
            this.angleExtent = shiftAngleExtent(this.angleExtent);
            this.v = vehicle; //set the Sensor's vehicle object reference to the one being plugged into the constructor
            this.x = vehicle.getCenterPoint().getX(); //set y coordinate of sensor to y coordinate of front of vehicle
            this.y = vehicle.getCenterPoint().getY(); //set x coordinate of sensor to x coordinate of front of vehicle
            Arc2D sensorConeOfSightTemp = new Arc2D.Double(x,y,width,height,angleStart,angleExtent,Arc2D.PIE); //create temp sensor cone
            double centX = v.getCenterPoint().getX() - (sensorConeOfSightTemp.getCenterX() - v.getCenterPoint().getX());   //correct the x centre value of temp sensor cone
            double centY = v.getCenterPoint().getY() - (sensorConeOfSightTemp.getCenterY() - v.getCenterPoint().getY());   //correct the y centre value of temp sensor cone
            Arc2D.Double sensorConeOfSight = new Arc2D.Double(centX,centY,width,height,angleStart,angleExtent,Arc2D.PIE);                                //create a new sensor cone with the corrected centre coordinate
            this.shape = sensorConeOfSight;
            area = new Area(shape);                                                                                        //convert sensor cone shape to an area
            this.successfulTraversal = true;
            this.angleExtent = undoShiftAngleExtent(this.angleExtent);

            this.distanceFOV = getUpperMid().distance(getVehicle().getPointAtMiddleFront(Constants.DOUBLE_EQUAL_PRECISION));
    }

    public Sensor()                                                     //no argument constructor
    {}

    public Sensor(Sensor sensor) //copy constructor
    {
        this(sensor.getVehicle(),sensor.getWidth(),sensor.getHeight(),sensor.getAngleStart(),sensor.getAngleExtent());
    }

    synchronized public boolean getPassedCheckPointOne()
    {
        return this.passedCheckPointOne.get();
    }

    synchronized public boolean getPassedCheckPointTwo()
    {
        return this.passedCheckPointTwo.get();
    }

    synchronized public void setPassedCheckPointOne(boolean passedCheckPointOne)
    {
        this.passedCheckPointOne.set(passedCheckPointOne);
    }

    synchronized public void setPassedCheckPointTwo(boolean passedCheckPointTwo)
    {
        this.passedCheckPointTwo.set(passedCheckPointTwo);
    }

    synchronized public boolean detects(Object obstruction)
    {
        if(obstructionsInFOV.contains(obstruction))    //if this obstruction has been detected already
            obstructionsInFOV.remove(obstruction);     //remove it from the 'detected list' so we can check if it's still being detected

        if(obstruction.getClass().equals(BasicAutoVehicle.class))
        {
            VehicleSimView vehicleB = (VehicleSimView)obstruction;
            if(vehicleB.getVIN() == getVehicleVin()) return false;          //vehicle detected itself, so ignore
            if(sensorOn(vehicleB).hasCrashed().get()) return false;         //detected vehicle has crashed already, so ignore
            Area vehicleBAreaCopy = new Area(vehicleB.getShape());
            vehicleBAreaCopy.intersect(new Area(this.area));
            if(!vehicleBAreaCopy.isEmpty()) obstructionsInFOV.add(obstruction);
            return !vehicleBAreaCopy.isEmpty();
        }

        if(obstruction.getClass().equals(DrunkPedestrian.class))
        {
            DrunkPedestrian drunkPedestrian = (DrunkPedestrian)obstruction;
            Area pedestrianAreaCopy = new Area(drunkPedestrian.getBody());
            pedestrianAreaCopy.intersect(new Area(this.area));
            if(!pedestrianAreaCopy.isEmpty()) obstructionsInFOV.add(obstruction);
            return !pedestrianAreaCopy.isEmpty();
        }

        /*
        if(obstruction.getClass().equals(RoadBasedIntersection.class))
        {
            RoadBasedIntersection intersection = (RoadBasedIntersection) obstruction;
            Area intersectionAreaCopy = new Area(intersection.getArea());
            intersectionAreaCopy.intersect(new Area(this.area));
            if(!intersectionAreaCopy.isEmpty()) obstructionsInFOV.add(obstruction);
            return !intersectionAreaCopy.isEmpty();
        }
        */

        return false;
    }

    synchronized public List<Object> getObstructionsInFOV()
    {
        return this.obstructionsInFOV;
    }

    synchronized public boolean getObstructionDetected()
    {
        return this.obstructionDetected.get();
    }

    synchronized public void setObstructionDetected(boolean obstructionDetected)
    {
        this.obstructionDetected.set(obstructionDetected);
    }

    synchronized public void moveWithVehicle()   //make this sensor move with its housing vehicle
    {
        this.angleExtent = shiftAngleExtent(this.angleExtent);
        this.x = v.getCenterPoint().getX(); //set y coordinate of sensor to y coordinate of front of vehicle
        this.y = v.getCenterPoint().getY(); //set x coordinate of sensor to x coordinate of front of vehicle
        Arc2D sensorConeOfSightTemp = new Arc2D.Double(x,y,width,height,angleStart,angleExtent,Arc2D.PIE);     //create temp sensor cone
        double centX = v.getCenterPoint().getX() - (sensorConeOfSightTemp.getCenterX() - v.getCenterPoint().getX());   //correct the x centre value of temp sensor cone
        double centY = v.getCenterPoint().getY() - (sensorConeOfSightTemp.getCenterY() - v.getCenterPoint().getY());   //correct the y centre value of temp sensor cone
        Arc2D.Double sensorConeOfSight = new Arc2D.Double(centX,centY,width,height,angleStart,angleExtent,Arc2D.PIE);                                //create a new sensor cone with the corrected centre coordinate
        this.shape = sensorConeOfSight;
        this.area = new Area(shape);
        ShapeUtils.rotateArea(area,v.getHeading(),v.getCenterPoint());     //rotate the sensor cone in its housing vehicle's direction
        this.angleExtent = undoShiftAngleExtent(this.angleExtent);
    }

    public synchronized double shiftAngleExtent(double unshiftedAngleExtent)  //must multiply by 2 to keep shape symmetrical with car (something to do with sim's 2D coordinate system I think)
    {
        double shiftedAngleExtent = 2*unshiftedAngleExtent;
        return shiftedAngleExtent;                               //this means our arc goes from (angleStart) degrees to 2*(angleExtent) degrees
    }

    public synchronized double undoShiftAngleExtent(double shiftedAngleExtent)  //undo shifting of angle extent
    {
        double unShiftedAngleExtent = shiftedAngleExtent/2;
        return unShiftedAngleExtent;
    }

    public synchronized void setSuccessfulTraversal(boolean successfulTraversal)
    {
        this.successfulTraversal = successfulTraversal;
    }

    public synchronized boolean getSuccessfulTraversal()
    {
        return this.successfulTraversal;
    }

    public synchronized void setHasCrashed(boolean hasCrashed)
    {
        this.hasCrashed.set(hasCrashed);
    }

    public synchronized AtomicBoolean hasCrashed()
    {
        return hasCrashed;
    }

    public synchronized Shape getShape()
    {
        return shape;
    }

    public synchronized Arc2D.Double getArc()
    {
        return shape;
    }

    public synchronized Area getArea()
    {
        return area;
    }


    public synchronized int getVehicleVin()
    {
        return v.getVIN();
    }

    public synchronized VehicleSimView getVehicle()
    {
        return v;
    }

    public synchronized double getX()
    {
        return x;
    }

    public synchronized double getY()
    {
        return y;
    }

    public synchronized double getWidth()
    {
        return width;
    }

    public synchronized double getHeight()
    {
        return height;
    }

    public synchronized double getAngleStart()
    {
        return angleStart;
    }

    public synchronized double getAngleExtent()
    {
        return angleExtent;
    }

    public synchronized int getType()
    {
        return type;
    }

    public synchronized Point2D.Double getLowerMid()
    {
        return (Point2D.Double)v.getCenterPoint();
    }

    public synchronized Point2D.Double getUpperMid()
    {
        double x = getVehicle().getCenterPoint().getX();
        double y = getVehicle().getCenterPoint().getY();
        double r = Main.cfgSensorWidth / 2;
        double theta = getVehicle().getHeading();

        return new Point2D.Double(x + r*Math.cos(theta), y + r*Math.sin(theta));
    }

    public synchronized double getDistanceFOV()
    {
        return this.distanceFOV;
    }

    synchronized public static Sensor sensorOn(VehicleSimView vehicle)  //return a car's basic sensor (without aim or neat functionality)
    {
        return ((BasicAutoVehicle) vehicle).getSensor();
    }


}