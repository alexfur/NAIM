package aim4.vehicle;

public class AIMController extends Controller
{

    private VehicleSimView vehicle;
    private AimSensor sensor;

    public AIMController(VehicleSimView vehicle)
    {
        super(vehicle);
        this.sensor = AimSensor.frontAimSensorOn(vehicle);
    }

    public void control()
    {
        if(sensor.isWaitingForObstruction().get())
            vehicle.slowToStop();
    }

}