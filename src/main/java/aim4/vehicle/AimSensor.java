package aim4.vehicle;

import java.awt.geom.*;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.*;
import java.awt.Shape;

import aim4.Main;
import aim4.util.ShapeUtils;
import aim4.config.Debug;
import aim4.driver.AutoDriver;
import aim4.driver.DriverSimView;
import aim4.map.lane.Lane;
import aim4.msg.i2v.I2VMessage;
import aim4.msg.v2i.V2IMessage;
import aim4.noise.DoubleGauge;
import aim4.obstructions.DrunkPedestrian;
import aim4.util.GeomMath;
import aim4.util.GeomUtil;
import aim4.vehicle.AutoVehicleDriverView.LRFMode;
import java.awt.*;
import java.util.concurrent.atomic.AtomicBoolean;

public class AimSensor extends Sensor
{
    private AtomicBoolean waitingForObstruction;

    public AimSensor(VehicleSimView vehicle)
    {
        super(vehicle);
        this.waitingForObstruction = new AtomicBoolean(false);
    }

    public AimSensor(VehicleSimView vehicle, double width, double height, double angleStart, double angleExtent)
    {
        super(vehicle,width,height,angleStart,angleExtent);
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

    synchronized public static AimSensor sensorOn(VehicleSimView vehicle)  //return a car's basic sensor (without aim or neat functionality)
    {
        return (AimSensor) ((BasicAutoVehicle) vehicle).getSensor();
    }

    synchronized public AtomicBoolean isWaitingForObstruction()
    {
        return this.waitingForObstruction;
    }
}