package aim4.vehicle;


public abstract class Controller
{
    private VehicleSimView vehicle;
    private Sensor sensor;

    public Controller(VehicleSimView vehicle)
    {
        this.vehicle = vehicle;
        this.sensor = Sensor.sensorOn(vehicle);
    }

    abstract void control();

}