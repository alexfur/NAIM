package aim4.vehicle;

import aim4.Main;
import aim4.config.Constants;
import aim4.util.GeomMath;
import org.encog.ml.data.MLData;
import org.encog.ml.data.basic.BasicMLData;
import org.encog.neural.neat.NEATNetwork;
import org.encog.util.arrayutil.NormalizationAction;
import org.encog.util.arrayutil.NormalizedField;

public class NEATController extends Controller
{
    private VehicleSimView vehicle;
    private NeatSensor sensor;
    private NEATNetwork network;

    private NormalizedField inputDistToObstr;                       //distance from car to obstruction
    private NormalizedField inputAngleToObstr;                      //angle from front of car to obstruction
    private NormalizedField inputDistToIntersectionEdge;            //distance from car to intersection (from POV of car inside intersection)
    //private NormalizedField inputDistToIntersectionExitPoint;       //distance from centre of a car to the intersection exit point that it must reach

    private NormalizedField outputSteeringAngle;
    private NormalizedField outputAccel;

    public NEATController(VehicleSimView vehicle, NEATNetwork network)
    {
        super(vehicle);
        this.vehicle = vehicle;
        this.sensor = NeatSensor.sensorOn(vehicle);
        this.network = network;
        setupNormalizers();
    }

    public synchronized void control()
    {
        MLData networkOutput = network.compute(getInputs());
        if(sensor.getInIntersection())
        {
            turnToAngleInFOV(networkOutput.getData(0));             //NEAT can only steer inside the intersection
        }
        setAcceleration(networkOutput.getData(1));                  //NEAT can control acceleration inside or outside (but not too far from) intersection
    }

    public synchronized BasicMLData getInputs()
    {
        double[] networkInputs = new double[4];

        networkInputs[0] = inputDistToObstr.normalize(sensor.getDistanceToNearestObstruction());
        networkInputs[1] = inputAngleToObstr.normalize(GeomMath.getAngleBetweenObjects(vehicle.getPointAtMiddleFront(Constants.DOUBLE_EQUAL_PRECISION),vehicle.getHeading(),sensor.getPositionOfNearestObstruction()));
        networkInputs[2] = inputDistToIntersectionEdge.normalize(sensor.getDistanceToIntersectionEdge());

        //networkInputs[3] = inputDistToIntersectionExitPoint.normalize(sensor.getDistToIntersectionExitPoint());

        return new BasicMLData(networkInputs);
    }

    public synchronized void setupNormalizers()
    {
        /*      INPUTS
         --------------------*/

        inputDistToObstr = new NormalizedField(NormalizationAction.SingleField,
                "distance to obstruction in FOV",
                sensor.getDistanceFOV(),
                0,
                1,
                0);

        inputAngleToObstr = new NormalizedField(NormalizationAction.SingleField,
                "angle to obstruction in FOV",
                Math.PI,
                -1*Math.PI,
                1,
                -1);

        inputDistToIntersectionEdge = new NormalizedField(NormalizationAction.SingleField,
                "distance to edge of intersection (from POV of vehicle inside intersection",
                sensor.getDistanceFOV(),
                0,
                1,
                0);

        /*
        inputDistToIntersectionExitPoint = new NormalizedField(NormalizationAction.SingleField,
                "distance from centre of a car to the intersection exit point that it must reach",
                //vehicle.getDriver().getSpawnPoint().getPosition().distance(sensor.getIntersectionExitPoint()), //max real distance from vehicle to its exit point in intersection is the distance from that vehicle's spawnpoint to the intersection exit point
                179,
                0,          //min real distance from vehicle to its intersection exit point is 0
                1,          //max normalised distance from vehicle to its intersection exit point is 1
                0);         //min normalised distance from vehicle to its intersection exit point is 0
        */


        /*      OUTPUTS
         --------------------*/
        outputSteeringAngle = new NormalizedField(NormalizationAction.SingleField,
                "steering angle to turn to",
                sensor.getFOVTurnPoints().size()-1,
                0,
                1,
                0);

        outputAccel = new NormalizedField(NormalizationAction.SingleField,
                "acceleration to change to",
                Main.cfgMaxAccel,
                Main.cfgMaxDecel,
                1,
                0);

    }

    public synchronized void turnToAngleInFOV(double normNetOutput)   //turn towards one of the NN-input points on the arc of the FOV
    {
        double denormalisedOutput = outputSteeringAngle.deNormalize(normNetOutput);
        //System.out.println((int)denormalisedOutput);
        sensor.turnToAngleInFOV((int)denormalisedOutput);
    }

    public synchronized void setAcceleration(double normNetOutput)
    {
        double denormalisedOutput = outputAccel.deNormalize(normNetOutput);
        //System.out.println(denormalisedOutput);
        sensor.setAccelWithMaxTargetVelocity((int)denormalisedOutput);
    }

}