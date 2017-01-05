package aim4.vehicle;

import aim4.Main;
import aim4.util.GeomMath;
import org.encog.ml.data.MLData;
import org.encog.ml.data.basic.BasicMLData;
import org.encog.neural.neat.NEATNetwork;
import org.encog.util.arrayutil.NormalizationAction;
import org.encog.util.arrayutil.NormalizedField;

public class NEATController extends Controller
{
    private VehicleSimView vehicle;
    private NeatSensor frontSensor;
    private NeatSensor leftSensor;
    private NeatSensor rightSensor;

    private NEATNetwork network;

    private NormalizedField inputDistToObstrFrontSensor;            //distance from car to obstruction front sensor
    private NormalizedField inputDistToObstrLeftSensor;             //distance from car to obstruction left sensor
    private NormalizedField inputDistToObstrRightSensor;            //distance from car to obstruction right sensor
    private NormalizedField inputAngleToObstrFrontSensor;           //angle from front of car to obstruction
    private NormalizedField inputDistToIntersectionEdge;            //distance from car to intersection (from POV of car inside intersection)

    private NormalizedField outputSteeringAngle;
    private NormalizedField outputAccel;

    public NEATController(VehicleSimView vehicle, NEATNetwork network)
    {
        super(vehicle);
        this.vehicle = vehicle;
        this.frontSensor = NeatSensor.frontNeatSensorOn(vehicle);
        this.leftSensor = NeatSensor.leftNeatSensorOn(vehicle);
        this.rightSensor = NeatSensor.rightNeatSensorOn(vehicle);
        this.network = network;
        setupNormalizers();
    }

    public synchronized void control()
    {
        MLData networkOutput = network.compute(getInputs());
        if(frontSensor.getInIntersection())
        {
            turnToAngleInFOV(networkOutput.getData(0));             //NEAT can only steer inside the intersection
        }
        setAcceleration(networkOutput.getData(1));                  //NEAT can control acceleration inside or outside (but not too far from) intersection
    }

    public synchronized BasicMLData getInputs()
    {
        double[] networkInputs = new double[5];

        if(frontSensor.getObstructionDetected())
        {
            networkInputs[0] = inputDistToObstrFrontSensor.normalize(frontSensor.getDistanceToNearestObstruction());
            networkInputs[1] = inputAngleToObstrFrontSensor.normalize(GeomMath.getAngleBetweenObjects(frontSensor.getRayToNearestObstruction().getP1(),vehicle.getHeading(),frontSensor.getRayToNearestObstruction().getP2()));
        }

        networkInputs[2] = inputDistToIntersectionEdge.normalize(frontSensor.getDistanceToIntersectionEdge());

        if(leftSensor.getObstructionDetected())
        {
            networkInputs[3] = inputDistToObstrLeftSensor.normalize(leftSensor.getDistanceToNearestObstruction());
        }

        if(rightSensor.getObstructionDetected())
        {
            networkInputs[4] = inputDistToObstrRightSensor.normalize(rightSensor.getDistanceToNearestObstruction());
        }

        return new BasicMLData(networkInputs);
    }

    public synchronized void setupNormalizers()
    {
        /*      INPUTS
         --------------------*/

        inputDistToObstrFrontSensor = new NormalizedField(NormalizationAction.SingleField,
                "distance to obstruction in front sensor FOV",
                frontSensor.getDistanceFOV(),
                0,
                1,
                0);

        inputDistToObstrLeftSensor = new NormalizedField(NormalizationAction.SingleField,
                "distance to obstruction in left sensor FOV",
                frontSensor.getDistanceFOV(),   //TODO: make it leftSensor.getDistanceFOV() - they're the same size so it doesn't really matter tho
                0,
                1,
                0);

        inputDistToObstrRightSensor = new NormalizedField(NormalizationAction.SingleField,
                "distance to obstruction in right sensor FOV",
                frontSensor.getDistanceFOV(),   //TODO: make it rightSensor.getDistanceFOV() - they're the same size so it doesn't really matter tho
                0,
                1,
                0);

        inputAngleToObstrFrontSensor = new NormalizedField(NormalizationAction.SingleField,
                "angle to obstruction in FOV",
                Math.PI,
                -1*Math.PI,
                //Math.toRadians(Main.cfgSensorAngleEnd),
                //Math.toRadians(Main.cfgSensorAngleStart),
                1,
                -1);

        inputDistToIntersectionEdge = new NormalizedField(NormalizationAction.SingleField,
                "distance to edge of intersection (from POV of vehicle inside intersection",
                frontSensor.getDistanceFOV(),
                0,
                1,
                0);


        /*      OUTPUTS
         --------------------*/
        outputSteeringAngle = new NormalizedField(NormalizationAction.SingleField,
                "steering angle to turn to",
                frontSensor.getFOVTurnPoints().size()-1,
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
        frontSensor.turnToAngleInFOV((int)denormalisedOutput);
    }

    public synchronized void setAcceleration(double normNetOutput)
    {
        double denormalisedOutput = outputAccel.deNormalize(normNetOutput);
        //System.out.println(denormalisedOutput);
        frontSensor.setAccelWithMaxTargetVelocity((int)denormalisedOutput);
    }

}