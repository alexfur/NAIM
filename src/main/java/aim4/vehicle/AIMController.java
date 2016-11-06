package aim4.vehicle;

import org.encog.neural.data.NeuralData;
import org.encog.neural.data.basic.BasicNeuralData;
import org.encog.neural.networks.BasicNetwork;

import javax.sound.midi.ControllerEventListener;
import java.awt.geom.Point2D;
import java.util.concurrent.atomic.AtomicBoolean;

public class AIMController extends Controller
{

    private VehicleSimView vehicle;
    private AimSensor sensor;

    public AIMController(VehicleSimView vehicle)
    {
        super(vehicle);
        this.sensor = AimSensor.sensorOn(vehicle);
    }

    public void control()
    {
        if(sensor.isWaitingForObstruction().get())
            vehicle.slowToStop();
    }

}