package aim4.vehicle;


public abstract class Controller
{
    private VehicleSimView vehicle;
    private Sensor frontSensor;
    private Sensor leftSensor;
    private Sensor rightSensor;

    public Controller(VehicleSimView vehicle)
    {
        this.vehicle = vehicle;
        this.frontSensor = Sensor.frontSensorOn(vehicle);
        this.leftSensor = Sensor.leftSensorOn(vehicle);
        this.rightSensor = Sensor.rightSensorOn(vehicle);
    }

    abstract void control();

}