package aim4.vehicle;

import java.awt.geom.Point2D;
import java.util.concurrent.atomic.AtomicBoolean;

public class AimSensor extends Sensor
{
    private AtomicBoolean waitingForObstruction;

    public AimSensor(VehicleSimView vehicle, Point2D.Double pointForDistFOV, double rad)
    {
        super(vehicle, pointForDistFOV, rad);
        this.waitingForObstruction = new AtomicBoolean(false);
    }

    public AimSensor(VehicleSimView vehicle, double width, double height, double angleStart, double angleExtent, Point2D.Double pointForDistFOV, double rad)
    {
        super(vehicle,width,height,angleStart,angleExtent,pointForDistFOV, rad);
        this.waitingForObstruction = new AtomicBoolean(false);
    }

    public AimSensor()          //no arg constructor
    {}

    synchronized public void setWaitingForObstruction(AtomicBoolean waiting)
    {
        this.waitingForObstruction.set(waiting.get());
    }

    public void avoidCollisionHeuristic()
    {
        getVehicle().slowToStop();
    }

    synchronized public static AimSensor frontAimSensorOn(VehicleSimView vehicle)
    {
        return (AimSensor) ((BasicAutoVehicle) vehicle).getFrontSensor();
    }

    synchronized public static AimSensor leftAimSensorOn(VehicleSimView vehicle)
    {
        return (AimSensor) ((BasicAutoVehicle) vehicle).getLeftSensor();
    }

    synchronized public static AimSensor rightAimSensorOn(VehicleSimView vehicle)
    {
        return (AimSensor) ((BasicAutoVehicle) vehicle).getRightSensor();
    }


    synchronized public AtomicBoolean isWaitingForObstruction()
    {
        return this.waitingForObstruction;
    }
}