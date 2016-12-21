/*
Copyright (c) 2011 Tsz-Chiu Au, Peter Stone
University of Texas at Austin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the University of Texas at Austin nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package aim4.sim;

import aim4.Main;
import aim4.config.Debug;
import aim4.config.DebugPoint;
import aim4.driver.AutoDriver;
import aim4.driver.DriverSimView;
import aim4.driver.ProxyDriver;
import aim4.im.IntersectionManager;
import aim4.im.RoadBasedIntersection;
import aim4.im.v2i.V2IManager;
import aim4.map.BasicMap;
import aim4.map.DataCollectionLine;
import aim4.map.Road;
import aim4.map.SpawnPoint;
import aim4.map.SpawnPoint.SpawnSpec;
import aim4.map.lane.Lane;
import aim4.msg.i2v.I2VMessage;
import aim4.msg.v2i.V2IMessage;
import aim4.obstructions.DrunkPedestrian;
import aim4.vehicle.*;
import org.encog.neural.neat.NEATNetwork;

import java.awt.*;
import java.awt.geom.Area;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.text.DecimalFormat;
import java.util.*;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * The autonomous drivers only simulator.
 */
public class AutoDriverOnlySimulator implements Simulator
{

  /////////////////////////////////
  // NESTED CLASSES
  /////////////////////////////////

  /**
   * The result of a simulation step.
   */
  public static class AutoDriverOnlySimStepResult implements SimStepResult
  {

    /** The VIN of the completed vehicles in this time step */
    List<Integer> completedVINs;

    /**
     * Create a result of a simulation step
     *
     * @param completedVINs  the VINs of completed vehicles.
     */
    public AutoDriverOnlySimStepResult(List<Integer> completedVINs)
    {
      this.completedVINs = completedVINs;
    }

    /**
     * Get the list of VINs of completed vehicles.
     *
     * @return the list of VINs of completed vehicles.
     */
    public List<Integer> getCompletedVINs() {
      return completedVINs;
    }
  }

  /////////////////////////////////
  // PRIVATE FIELDS
  /////////////////////////////////

  /** The map */
  private BasicMap basicMap;
  /** All active vehicles, in form of a map from VINs to vehicle objects. */
  private volatile Map<Integer,VehicleSimView> vinToVehicles;
  /** The current time */
  private volatile double currentTime;
  /** The number of completed vehicles */
  private int numOfCompletedVehicles;
  /** The total number of bits transmitted by the completed vehicles */
  private int totalBitsTransmittedByCompletedVehicles;
  /** The total number of bits received by the completed vehicles */
  private int totalBitsReceivedByCompletedVehicles;
  /** Atomic Integer used to set Pedestrian IDs */
  private AtomicInteger atomicRef = new AtomicInteger(0);

  volatile private double timestep;

  /** format/round off to 2 decimal places                                        -rudolf         */
  DecimalFormat df = new DecimalFormat("#.##");

  /** Arraylist containing drunk pedestrians                                      -rudolf         */
   private List<DrunkPedestrian> drunkPedestrians;

  /** Number of vehicles to spawn in this round                                         -rudolf         */
  private AtomicInteger numVehiclesToSpawn = new AtomicInteger(Main.cfgNumVehicles);

  /** Number of vehicles spawned in this round (updated as more vehicles get spawned) (should not exceed numVehiclesToSpawn)    -rudolf         */
  public AtomicInteger numVehiclesSpawned = new AtomicInteger();

  private AtomicBoolean noMoreVehiclesToSpawn = new AtomicBoolean(false);

  public List<VehicleSimView> crashedVehicles = Collections.synchronizedList(new ArrayList<VehicleSimView>());

  public NEATNetwork network;

  private volatile double score;

  private volatile int numIntersectionTraversals;

  private List<Double> traversalTimes = Collections.synchronizedList(new ArrayList<Double>());

  private volatile boolean printSimResults = true;

    /////////////////////////////////
    // CLASS CONSTRUCTORS
    /////////////////////////////////

  /**
   * Create an instance of the simulator.
   *
   * @param basicMap             the map of the simulation
   */
  public AutoDriverOnlySimulator(BasicMap basicMap)
  {
    drunkPedestrians = Collections.synchronizedList(new ArrayList<DrunkPedestrian>());

    this.basicMap = basicMap;
    this.vinToVehicles = new HashMap<Integer,VehicleSimView>();

    currentTime = 0.0;
    numOfCompletedVehicles = 0;
    totalBitsTransmittedByCompletedVehicles = 0;
    totalBitsReceivedByCompletedVehicles = 0;

    if(Main.cfgNumPedestrians>0)
      spawnDrunkPedestrians();    //only spawn pedestrians once (when the simulation starts) - we don't want a constant flow of pedestrians spawning each timestep

    numIntersectionTraversals = 0;
    score = 0;
  }

  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  // the main loop

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized AutoDriverOnlySimStepResult step(double timeStep)
  {

    if(printSimResults)                                               //if sim results still need to be printed (ie: the simulation hasn't finished yet)
    {
      if((numIntersectionTraversals + crashedVehicles.size()) == numVehiclesToSpawn.get())
      {
        double averageTraversalTime = 0;
        for (double traversalTime : traversalTimes)
        {
          averageTraversalTime += traversalTime;
        }
        averageTraversalTime /= numIntersectionTraversals;

        System.out.println("Average timesteps taken for a vehicle to traverse intersection: " + averageTraversalTime);

        double x = numIntersectionTraversals;
        double y = numVehiclesToSpawn.get();
        double percentageThroughput =  ((x / y) * 100);
        System.out.println("Percentage throughput of intersection: " + percentageThroughput);
        printSimResults = false;                                    //don't print sim results again for this simulation
      }
    }


    this.timestep = timeStep;

    if (Debug.PRINT_SIMULATOR_STAGE)
    {
      System.err.printf("--------------------------------------\n");
      System.err.printf("------SIM:spawnVehicles---------------\n");
    }

    if(numVehiclesSpawned.get() < numVehiclesToSpawn.get())   //only spawn as many vehicles as the user asked for
    {
      spawnVehicles(timeStep);
    }
    else
    {
      noMoreVehiclesToSpawn.set(true);
    }

    if (Debug.PRINT_SIMULATOR_STAGE)
    {
      System.err.printf("------SIM:provideSensorInput---------------\n");
    }
    provideSensorInput(); //unrelated to my sensors - rudolf

    if (Debug.PRINT_SIMULATOR_STAGE)
    {
      System.err.printf("------SIM:letDriversAct---------------\n");
    }

    checkCollisions();

    letDriversAct();
    if (Debug.PRINT_SIMULATOR_STAGE)
    {
      System.err.printf("------SIM:letIntersectionManagersAct--------------\n");
    }
    letIntersectionManagersAct(timeStep);
    if (Debug.PRINT_SIMULATOR_STAGE)
    {
      System.err.printf("------SIM:communication---------------\n");
    }
    communication();
    if (Debug.PRINT_SIMULATOR_STAGE)
    {
      System.err.printf("------SIM:moveVehicles---------------\n");
    }

      if(Main.cfgController.equals("AIM"))
      {
          checkSensorsAIM();
      }
      if(Main.cfgController.equals("NEAT"))
      {
        checkSensorsNEAT();
        checkDistancesToIntersectionEdges();
        deductPointsForLeavingTrack();
      }

    moveVehicles(timeStep);
    moveDrunkPedestrians(timeStep);

    List<Integer> completedVINs = Collections.synchronizedList(cleanUpCompletedVehicles());
    currentTime += timeStep;
    checkClocks();

    return new AutoDriverOnlySimStepResult(completedVINs);
  }

  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  // information retrieval

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized BasicMap getMap() {
    return basicMap;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized double getSimulationTime() {
    return currentTime;
  }


  public synchronized void setNetwork(NEATNetwork network)
  {
      this.network = network;
  }

  public synchronized NEATNetwork getNetwork()
  {
      return this.network;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized int getNumCompletedVehicles() {
    return numOfCompletedVehicles;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized double getAvgBitsTransmittedByCompletedVehicles() {
    if (numOfCompletedVehicles > 0) {
      return ((double)totalBitsTransmittedByCompletedVehicles)
             / numOfCompletedVehicles;
    } else {
      return 0.0;
    }
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized double getAvgBitsReceivedByCompletedVehicles() {
    if (numOfCompletedVehicles > 0) {
      return ((double)totalBitsReceivedByCompletedVehicles)
             / numOfCompletedVehicles;
    } else {
      return 0.0;

    }



  }


  public synchronized AtomicBoolean getNoMoreVehiclesToSpawn()
  {
    return noMoreVehiclesToSpawn;
  }

  /**
   * Allow external classes to access the list of vehicles that are currently on screen.
   * Canvas.java needs this method to draw them onto the screen for users to see.
   */
  @Override
  public synchronized Set<VehicleSimView> getActiveVehicles()                                         //- rudolf
  {
    Set<VehicleSimView> temp;
    temp = Collections.synchronizedSet((new HashSet<VehicleSimView>(vinToVehicles.values())));
    return temp;
  }


  /**
   * Allow external classes to access the list of drunk padestrians currently on screen.
   * Canvas.java needs this method to draw them onto the screen for users to see.
   */
  public synchronized ArrayList<DrunkPedestrian> getActiveDrunkPedestrians()      // - rudolf
  {
      ArrayList<DrunkPedestrian> temp = new ArrayList<DrunkPedestrian>();
      temp.addAll(drunkPedestrians);
      return temp;                    //return the list of drunk pedestrians
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized VehicleSimView getActiveVehicle(int vin)
  {
    return vinToVehicles.get(vin);
  }


  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////


  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized void addProxyVehicle(ProxyVehicleSimView vehicle) {
    Point2D pos = vehicle.getPosition();
    Lane minLane = null;
    double minDistance = -1.0;

    for(Road road : basicMap.getRoads()) {
      for(Lane lane : road.getLanes()) {
        double d = lane.nearestDistance(pos);
        if (minLane == null || d < minDistance) {
          minLane = lane;
          minDistance = d;
        }
      }
    }
    assert minLane != null;

    ProxyDriver driver = vehicle.getDriver();
    if (driver != null) {
      driver.setCurrentLane(minLane);
      driver.setSpawnPoint(null);
      driver.setDestination(null);
    }

    vinToVehicles.put(vehicle.getVIN(), vehicle);
  }


  /////////////////////////////////
  // PRIVATE METHODS
  /////////////////////////////////

  /////////////////////////////////
  // STEP 1
  /////////////////////////////////

  /**
   * Spawn vehicles.
   *
   * @param timeStep  the time step
   */
  synchronized private void spawnVehicles(double timeStep) //rudolf - #spawnvehicles
  {
    for(SpawnPoint spawnPoint : basicMap.getSpawnPoints())
    {
      List<SpawnSpec> spawnSpecs = spawnPoint.act(timeStep);
      if (!spawnSpecs.isEmpty())
      {
        //if (canSpawnVehicle(spawnPoint) && getActiveVehicles().size() < Main.cfgNumVehicles)
        if (canSpawnVehicle(spawnPoint))
        {
          for(SpawnSpec spawnSpec : spawnSpecs)
          {
              //if(!(spawnPoint.getLane().getId()==2)) continue;                  //force vehicles to spawn from West road

            VehicleSimView vehicle = makeVehicle(spawnPoint, spawnSpec);
              ((BasicAutoVehicle)vehicle).setSpawnTime(spawnSpec.getSpawnTime());                         //store vehicle's spawntime in the vehicle itself

            Sensor sensor;
            Controller controller;

            if(Main.cfgController.equals("AIM"))
            {
              sensor = new AimSensor(vehicle);
              ((BasicAutoVehicle)vehicle).installSensor(sensor);
              controller = new AIMController(vehicle);
              ((BasicAutoVehicle)vehicle).installController(controller);
            }

            if(Main.cfgController.equals("NEAT"))
            {
              sensor = new NeatSensor(vehicle);
              ((BasicAutoVehicle)vehicle).installSensor(sensor);
              controller = new NEATController(vehicle, network);
              ((BasicAutoVehicle)vehicle).installController(controller);
            }


            VinRegistry.registerVehicle(vehicle); // Get vehicle a VIN number
            vinToVehicles.put(vehicle.getVIN(), vehicle);
            numVehiclesSpawned.incrementAndGet();         //rudolf - increment number of spawned vehicles
            break; // only handle the first spawn vehicle
                   // TODO: need to fix this
          }
        } // else ignore the spawnSpecs and do nothing
      }
    }
  }



  /**
   * Spawn drunk pedestrians at the spawnpoints entered in config.txt
   */
  synchronized private void spawnDrunkPedestrians()                           // - rudolf
  {
    if(Main.cfgNumPedestrians > 0)                                      //if there are more than 0 pedestrians on the screen
      for(Point2D.Double[] waypoint : Main.cfgPedestrianWaypoints)
        drunkPedestrians.add(new DrunkPedestrian(atomicRef.getAndIncrement(), waypoint[0], waypoint[1]));
  }

  /**
   * Whether a spawn point can spawn any vehicle
   *
   * @param spawnPoint  the spawn point
   * @return Whether the spawn point can spawn any vehicle
   */
  synchronized private boolean canSpawnVehicle(SpawnPoint spawnPoint)
  {
    // TODO: can be made much faster.
    Rectangle2D noVehicleZone = spawnPoint.getNoVehicleZone();
    for(VehicleSimView vehicle : vinToVehicles.values())
    {
      if (vehicle.getShape().intersects(noVehicleZone))
      {
        return false;
      }
    }
    return true;
  }

  /**
   * Create a vehicle at a spawn point.
   *
   * @param spawnPoint  the spawn point
   * @param spawnSpec   the spawn specification
   * @return the vehicle
   */
  synchronized private VehicleSimView makeVehicle(SpawnPoint spawnPoint,
                                     SpawnSpec spawnSpec)
  {
    VehicleSpec spec = spawnSpec.getVehicleSpec();                 //use this if tester wants random vehicles (random spec)


    VehicleSpec customSpec = new VehicleSpec("neuroMobile",        //use this if tester wants to choose universal spec for all cars
            Main.cfgMaxAccel,       // maxAcceleration (m/s/s)     //default 4.5
            Main.cfgMaxDecel,       // maxDeceleration (m/s/s)     //default -45.0
            Main.cfgMaxVelocity,       // maxVelocity (m/s)        //default 60
            Main.cfgMinVelocity,       // minVelocity (m/s)
            Main.cfgVehicleLength,       // length (meters)         //default 4.0
            Main.cfgVehicleWidth,      // width (meters)            //default 1.75
            1.0,       // frontAxleDisplacement (meters)
            3.5,       // rearAxleDisplacement (meters)
            (1.75-0.25)/2, // wheelSpan (meters)
            0.3,       // wheelRadius (meters)
            0.25,      // wheelWidth (meters)
            Math.PI/3,   // maxSteeringAngle (radian)
            Math.PI/2);

    if(Main.cfgCarSpecMode == 'C')        //if user chose to use custom specs,
    {
      spec = customSpec;                  //use the custom spec just above
      //else just use the random spec chosen at the top
    }

    Lane lane = spawnPoint.getLane();
    // Now just take the minimum of the max velocity of the vehicle, and
    // the speed limit in the lane
    double initVelocity = Math.min(spec.getMaxVelocity(), lane.getSpeedLimit());
    // Obtain a Vehicle
    AutoVehicleSimView vehicle =
      new BasicAutoVehicle(spec,
                           spawnPoint.getPosition(),
                           spawnPoint.getHeading(),
                           spawnPoint.getSteeringAngle(),
                           initVelocity, // velocity
                           initVelocity,  // target velocity
                           spawnPoint.getAcceleration(),
                           spawnSpec.getSpawnTime());
    // Set the driver
    AutoDriver driver = new AutoDriver(vehicle, basicMap);
    driver.setCurrentLane(lane);
    driver.setSpawnPoint(spawnPoint);
    driver.setDestination(spawnSpec.getDestinationRoad());
    vehicle.setDriver(driver);

    return vehicle;
  }

  /////////////////////////////////
  // STEP 2
  /////////////////////////////////

  /**
   * Compute the lists of vehicles of all lanes.
   *
   * @return a mapping from lanes to lists of vehicles sorted by their
   *         distance on their lanes
   */
  synchronized private Map<Lane,SortedMap<Double,VehicleSimView>> computeVehicleLists()
  {
    // Set up the structure that will hold all the Vehicles as they are
    // currently ordered in the Lanes
    Map<Lane,SortedMap<Double,VehicleSimView>> vehicleLists =
      new HashMap<Lane,SortedMap<Double,VehicleSimView>>();
    for(Road road : basicMap.getRoads()) {
      for(Lane lane : road.getLanes()) {
        vehicleLists.put(lane, new TreeMap<Double,VehicleSimView>());
      }
    }
    // Now add each of the Vehicles, but make sure to exclude those that are
    // already inside (partially or entirely) the intersection
    for(VehicleSimView vehicle : vinToVehicles.values()) {
      // Find out what lanes it is in.
      Set<Lane> lanes = vehicle.getDriver().getCurrentlyOccupiedLanes();
      for(Lane lane : lanes) {
        // Find out what IntersectionManager is coming up for this vehicle
        IntersectionManager im =
          lane.getLaneIM().nextIntersectionManager(vehicle.getPosition());
        // Only include this Vehicle if it is not in the intersection.
        if(lane.getLaneIM().distanceToNextIntersection(vehicle.getPosition())>0
            || im == null || !im.intersects(vehicle.getShape().getBounds2D())) {
          // Now find how far along the lane it is.
          double dst = lane.distanceAlongLane(vehicle.getPosition());
          // Now add it to the map.
          vehicleLists.get(lane).put(dst, vehicle);
        }
      }
    }
    // Now consolidate the lists based on lanes
    for(Road road : basicMap.getRoads()) {
      for(Lane lane : road.getLanes()) {
        // We may have already removed this Lane from the map
        if(vehicleLists.containsKey(lane)) {
          Lane currLane = lane;
          // Now run through the lanes
          while(currLane.hasNextLane()) {
            currLane = currLane.getNextLane();
            // Put everything from the next lane into the original lane
            // and remove the mapping for the next lane
            vehicleLists.get(lane).putAll(vehicleLists.remove(currLane));
          }
        }
      }
    }

    return vehicleLists;
  }

  /**
   * Compute the next vehicles of all vehicles.
   *
   * @param vehicleLists  a mapping from lanes to lists of vehicles sorted by
   *                      their distance on their lanes
   * @return a mapping from vehicles to next vehicles
   */
  synchronized private Map<VehicleSimView, VehicleSimView> computeNextVehicle(
    Map<Lane,SortedMap<Double,VehicleSimView>> vehicleLists)
  {
    // At this point we should only have mappings for start Lanes, and they
    // should include all the Lanes they run into.  Now we need to turn this
    // into a hash map that maps Vehicles to the next vehicle in the Lane
    // or any Lane the Lane runs into
    Map<VehicleSimView, VehicleSimView> nextVehicle =
      new HashMap<VehicleSimView,VehicleSimView>();
    // For each of the ordered lists of vehicles
    for(SortedMap<Double,VehicleSimView> vehicleList : vehicleLists.values())
    {
      VehicleSimView lastVehicle = null;
      // Go through the Vehicles in order of their position in the Lane
      for(VehicleSimView currVehicle : vehicleList.values()) {
        if(lastVehicle != null) {
          // Create the mapping from the previous Vehicle to the current one
          nextVehicle.put(lastVehicle,currVehicle);
        }
        lastVehicle = currVehicle;
      }
    }

    return nextVehicle;
  }

  /**
   * Provide each vehicle with sensor information to allow it to make
   * decisions.  This works first by making an ordered list for each Lane of
   * all the vehicles in that Lane, in order from the start of the Lane to
   * the end of the Lane.  We must make sure to leave out all vehicles that
   * are in the intersection.  We must also concatenate the lists for lanes
   * that feed into one another.  Then, for each vehicle, depending on the
   * state of its sensors, we provide it with the appropriate sensor input.
   */
  synchronized private void provideSensorInput()
  {
    Map<Lane,SortedMap<Double,VehicleSimView>> vehicleLists =
      computeVehicleLists();
    Map<VehicleSimView, VehicleSimView> nextVehicle =
      computeNextVehicle(vehicleLists);

    provideIntervalInfo(nextVehicle);
    provideVehicleTrackingInfo(vehicleLists);
    provideTrafficSignal();
  }

  /**
   * Provide sensing information to the intervalometers of all vehicles.
   *
   * @param nextVehicle  a mapping from vehicles to next vehicles
   */
  synchronized private void provideIntervalInfo(
    Map<VehicleSimView, VehicleSimView> nextVehicle)
  {

    // Now that we have this list set up, let's provide input to all the
    // Vehicles.
    for(VehicleSimView vehicle: vinToVehicles.values()) {
      // If the vehicle is autonomous
      if (vehicle instanceof AutoVehicleSimView) {
        AutoVehicleSimView autoVehicle = (AutoVehicleSimView)vehicle;

        switch(autoVehicle.getLRFMode()) {
        case DISABLED:
          // Find the interval to the next vehicle
          double interval;
          // If there is a next vehicle, then calculate it
          if(nextVehicle.containsKey(autoVehicle)) {
            // It's the distance from the front of this Vehicle to the point
            // at the rear of the Vehicle in front of it
            interval = calcInterval(autoVehicle, nextVehicle.get(autoVehicle));
          } else { // Otherwise, just set it to the maximum possible value
            interval = Double.MAX_VALUE;
          }
          // Now actually record it in the vehicle
          autoVehicle.getIntervalometer().record(interval);
          autoVehicle.setLRFSensing(false); // Vehicle is not using
                                            // the LRF sensor
          break;
        case LIMITED:
          // FIXME
          autoVehicle.setLRFSensing(true); // Vehicle is using the LRF sensor
          break;
        case ENABLED:
          // FIXME
          autoVehicle.setLRFSensing(true); // Vehicle is using the LRF sensor
          break;
        default:
          throw new RuntimeException("Unknown LRF Mode: " +
                                     autoVehicle.getLRFMode().toString());
        }
      }
    }
  }

  /**
   * Provide tracking information to vehicles.
   *
   * @param vehicleLists  a mapping from lanes to lists of vehicles sorted by
   *                      their distance on their lanes
   */
  synchronized private void provideVehicleTrackingInfo(
    Map<Lane, SortedMap<Double, VehicleSimView>> vehicleLists)
  {
    // Vehicle Tracking
    for(VehicleSimView vehicle: vinToVehicles.values()) {
      // If the vehicle is autonomous
      if (vehicle instanceof AutoVehicleSimView) {
        AutoVehicleSimView autoVehicle = (AutoVehicleSimView)vehicle;

        if (autoVehicle.isVehicleTracking()) {
          DriverSimView driver = autoVehicle.getDriver();
          Lane targetLane = autoVehicle.getTargetLaneForVehicleTracking();
          Point2D pos = autoVehicle.getPosition();
          double dst = targetLane.distanceAlongLane(pos);

          // initialize the distances to infinity
          double frontDst = Double.MAX_VALUE;
          double rearDst = Double.MAX_VALUE;
          VehicleSimView frontVehicle = null ;
          VehicleSimView rearVehicle = null ;

          // only consider the vehicles on the target lane
          SortedMap<Double,VehicleSimView> vehiclesOnTargetLane =
            vehicleLists.get(targetLane);

          // compute the distances and the corresponding vehicles
          try {
            double d = vehiclesOnTargetLane.tailMap(dst).firstKey();
            frontVehicle = vehiclesOnTargetLane.get(d);
            frontDst = (d-dst)-frontVehicle.getSpec().getLength();
          } catch(NoSuchElementException e) {
            frontDst = Double.MAX_VALUE;
            frontVehicle = null;
          }
          try {
            double d = vehiclesOnTargetLane.headMap(dst).lastKey();
            rearVehicle = vehiclesOnTargetLane.get(d);
            rearDst = dst-d;
          } catch(NoSuchElementException e) {
            rearDst = Double.MAX_VALUE;
            rearVehicle = null;
          }

          // assign the sensor readings

          autoVehicle.getFrontVehicleDistanceSensor().record(frontDst);
          autoVehicle.getRearVehicleDistanceSensor().record(rearDst);

          // assign the vehicles' velocities

          if(frontVehicle!=null) {
            autoVehicle.getFrontVehicleSpeedSensor().record(
                frontVehicle.getVelocity());
          } else {
            autoVehicle.getFrontVehicleSpeedSensor().record(Double.MAX_VALUE);
          }
          if(rearVehicle!=null) {
            autoVehicle.getRearVehicleSpeedSensor().record(
                rearVehicle.getVelocity());
          } else {
            autoVehicle.getRearVehicleSpeedSensor().record(Double.MAX_VALUE);
          }

          // show the section on the viewer
          if (Debug.isTargetVIN(driver.getVehicle().getVIN())) {
            Point2D p1 = targetLane.getPointAtNormalizedDistance(
                Math.max((dst-rearDst)/targetLane.getLength(),0.0));
            Point2D p2 = targetLane.getPointAtNormalizedDistance(
                Math.min((frontDst+dst)/targetLane.getLength(),1.0));
            Debug.addLongTermDebugPoint(
              new DebugPoint(p2, p1, "cl", Color.RED.brighter()));
          }
        }
      }
    }

  }

  /**
   * Provide traffic signals.
   */
  synchronized private void provideTrafficSignal() {
    for(VehicleSimView vehicle: vinToVehicles.values()) {
      if (vehicle instanceof HumanDrivenVehicleSimView) {
        HumanDrivenVehicleSimView manualVehicle =
          (HumanDrivenVehicleSimView)vehicle;
        provideTrafficLightSignal(manualVehicle);
      }
    }
  }

  /**
   * Calculate the distance between vehicle and the next vehicle on a lane.
   *
   * @param vehicle      the vehicle
   * @param nextVehicle  the next vehicle
   * @return the distance between vehicle and the next vehicle on a lane
   */
  synchronized private double calcInterval(VehicleSimView vehicle,
                              VehicleSimView nextVehicle) {
    // From Chiu: Kurt, if you think this function is not okay, probably
    // we should talk to see what to do.
    Point2D pos = vehicle.getPosition();
    if(nextVehicle.getShape().contains(pos))
    {
      return 0.0;
    } else {
      // TODO: make it more efficient
      double interval = Double.MAX_VALUE ;
      for(Line2D edge : nextVehicle.getEdges())
      {
        double dst = edge.ptSegDist(pos);
        if(dst < interval){
          interval = dst;
        }
      }
      return interval;
    }
  }
  // Kurt's code:
  // interval = vehicle.getPosition().
  //   distance(nextVehicle.get(vehicle).getPointAtRear());


  /**
   * Provide traffic light signals to a vehicle.
   *
   * @param vehicle  the vehicle
   */
  private void provideTrafficLightSignal(HumanDrivenVehicleSimView vehicle)
  {
    // TODO: implement it later
//    DriverSimView driver = vehicle.getDriver();
//    Lane currentLane = driver.getCurrentLane();
//    Point2D pos = vehicle.getPosition();
//    IntersectionManager im = currentLane.getLaneIM().
//                             nextIntersectionManager(pos);
//    if (im != null) {
//      if (im instanceof LightOnlyManager) {
//        LightOnlyManager lightOnlyIM = (LightOnlyManager)im;
//        if (!im.getIntersection().getArea().contains(pos)) {
//          LightState s = lightOnlyIM.getLightState(currentLane);
//          vehicle.setLightState(s);
//          if (driver instanceof HumanDriver) {
//            ((HumanDriver)driver).setLightState(s);
//          }
//        } else {
//          vehicle.setLightState(null);
//          if (driver instanceof HumanDriver) {
//            ((HumanDriver)driver).setLightState(null);
//          }
//        }
//      }
//    } else {
//      vehicle.setLightState(null);
//      if (driver instanceof HumanDriver) {
//        ((HumanDriver)driver).setLightState(null);
//      }
//    }
  }

  /////////////////////////////////
  // STEP 3
  /////////////////////////////////



  synchronized private void checkCollisions()
  {
    /*           VEHICLE - VEHICLE COLLISION
    --------------------------------------------------------*/

      for (VehicleSimView vehicle1 : getActiveVehicles())
        {
          for (VehicleSimView vehicle2 : getActiveVehicles())                             //loop through all vehicle objects on screen except the one calling this method
          {
            if (!sensorOn(vehicle1).hasCrashed().get() && !sensorOn(vehicle2).hasCrashed().get())   //if we aren't dealing with an already crashed vehicle
            {
              if (VehicleUtil.intersects(vehicle2, new Area(vehicle1.getShape())) && vehicle1.getVIN() != vehicle2.getVIN())              //if vehicle1 and vehicle2 crash
              {

                if (!crashedVehicles.contains(vehicle1))                                   //make extra sure the vehicle hasn't already crashed prior to this
                {
                  sensorOn(vehicle1).setObstructionDetected(false);                        //we're not detecting the obstruction because we hit the obstruction !
                  sensorOn(vehicle1).setHasCrashed(true);
                  crashedVehicles.add(vehicle1);
                  if(Main.cfgNEATSetting.equals("Train"))
                    score -= 0.025;                                        //deduct 2 points for crashing into car
                }

                if (!crashedVehicles.contains(vehicle2))                                   //make extra sure the vehicle hasn't already crashed prior to this
                {
                  sensorOn(vehicle2).setObstructionDetected(false);                        //we're not detecting the obstruction because we hit the obstruction !
                  sensorOn(vehicle2).setHasCrashed(true);
                  crashedVehicles.add(vehicle2);
                  if(Main.cfgController.equals("Train"))
                    score -= 0.025;                                        //deduct 2 points for crashing into car
                }

              }
            }
          }

      }

    /*         VEHICLE - PEDESTRIAN COLLISION
    --------------------------------------------------------*/
      for(VehicleSimView vehicle : getActiveVehicles())
      {
        for(DrunkPedestrian drunkPedestrian : getActiveDrunkPedestrians())
        {
          if (! sensorOn(vehicle).hasCrashed().get())   //if we aren't dealing with an already crashed vehicle
          {
            if (VehicleUtil.intersects(vehicle, new Area(drunkPedestrian.getBody())) && vehicle.getVelocity() > 0.3)  //if car and pedestrian are touching, AND if the car is moving fast enough to kill someone
            {
              sensorOn(vehicle).setObstructionDetected(false);                        //we're not detecting the obstruction because we hit the obstruction !
              sensorOn(vehicle).setHasCrashed(true);
              if (!crashedVehicles.contains(vehicle))
              {
                crashedVehicles.add(vehicle);
                score -= 0.05;
              }
            }
          }
        }
      }
  }

  synchronized private void checkSensorsNEAT()
  {
    /*          SENSORS SCANNING FOR VEHICLES
    --------------------------------------------------------*/
      for (VehicleSimView vehicle1 : getActiveVehicles())
      {
        NeatSensor sensor1 = neatSensorOn(vehicle1);
        if(sensor1.getPassedCheckPointOne())                                          //sensors only turn on after hitting first checkpoint
          {
            for (VehicleSimView vehicle2 : getActiveVehicles())
            {
              if (sensor1.detects(vehicle2) && vehicle1.getVIN() != vehicle2.getVIN())  //if vehicle 1 detects vehicle 2 (but doesn't detect itself!)
                sensor1.setObstructionDetected(true);
            }
          }
      }

    /*          SENSORS SCANNING FOR PEDESTRIANS
    --------------------------------------------------------*/
    if(Main.cfgNumPedestrians > 0)                                            //if there are pedestrians in this simulation
    {
      for(VehicleSimView vehicle : getActiveVehicles())
      {
        NeatSensor sensor = neatSensorOn(vehicle);
        if(sensor.getPassedCheckPointOne())                                          //sensors only turn on after hitting first checkpoint
          {
            for(DrunkPedestrian drunkPedestrian : getActiveDrunkPedestrians())
            {
              if(sensor.detects(drunkPedestrian))
                sensor.setObstructionDetected(true);
            }
          }
      }
    }

      /* SENSORS CHECKING DISTANCES FROM THEIR CARS TO EDGES OF INTERSECTIONS
         (IF THEIR CARS ARE INSIDE INTERSECTIONS)
    -------------------------------------------------------------------------*/
      checkDistancesToIntersectionEdges();

  }

  public void checkDistancesToIntersectionEdges() // (this method also updates each vehicle's sensor's 'inIntersection' boolean)
  {
    for (VehicleSimView vehicle : getActiveVehicles())
    {
      NeatSensor sensor = NeatSensor.sensorOn(vehicle);
      for(IntersectionManager im : basicMap.getIntersectionManagers())
      {
        if(im.semiContains(vehicle))
        {
          sensor.setInIntersection(true);  //tell the sensor its car is in the intersection, so the NEAT controller will be allowed to turnToAngle in addition to controlling acceleration
          RoadBasedIntersection intersection = ((RoadBasedIntersection)im.getIntersection());
          try
          {
            double distToIntersection = sensor.getDistanceToObstruction(intersection);      //distance from front of vehicle to the edge of the intersection
            sensor.setDistanceToIntersectionEdge(distToIntersection);                       //provide sensor with knowledge of how far vehicle is to edge of intersection

          } catch(Exception e){e.printStackTrace();}
          break;  //no need to check if this vehicle is in other intersections if we found it was in this one
        }
        else
        {
          sensor.setInIntersection(false);  //tell the sensor its car isn't in the intersection, so the NEAT controller knows not to turnToAngle, but only to control acceleration
        }
      }
    }
  }

  synchronized private void checkSensorsAIM()
  {
    /*
    //          SENSORS SCANNING FOR VEHICLES
    //--------------------------------------------------------
      for (VehicleSimView vehicle1 : getActiveVehicles())
      {
        AimSensor sensor1 = aimSensorOn(vehicle1);
        if(sensor1.getPassedCheckPointOne())                                              //sensors only turn on after hitting first checkpoint
        {
            for (VehicleSimView vehicle2 : getActiveVehicles())                           //loop through all vehicle objects active on screen
            {
              if (sensor1.detects(vehicle2) && vehicle2.getVIN() != vehicle1.getVIN())    //if vehicle1 and vehicle2 crash
              {
                sensor1.setWaitingForObstruction(new AtomicBoolean(true));
              }
              else
              {
                if(!sensor1.isWaitingForObstruction().get())
                  sensor1.setWaitingForObstruction(new AtomicBoolean(false));
              }
            }
          }
      }
    */

    /*          SENSORS SCANNING FOR PEDESTRIANS
    --------------------------------------------------------*/
    if(Main.cfgNumPedestrians > 0)
    {
      for(VehicleSimView vehicle : getActiveVehicles())
      {
        AimSensor sensor = aimSensorOn(vehicle);
        if(sensor.getPassedCheckPointOne())                                          //sensors only turn on after hitting first checkpoint
        {
            for(DrunkPedestrian drunkPedestrian : getActiveDrunkPedestrians())
            {
              if(sensor.detects(drunkPedestrian))
              {
                sensor.setWaitingForObstruction(new AtomicBoolean(true));
              }
              else
              {
                if(!sensor.isWaitingForObstruction().get())
                  sensor.setWaitingForObstruction(new AtomicBoolean(false));
              }
            }
          }
      }
    }
  }

  /*
      Check if the intersection is in the FOV of a vehicle
      -------------------------------------------------------
      (this method is inefficient and encapsulated terribly)
   */
  public synchronized boolean intersectionInFOV(VehicleSimView vehicle)
  {
    for (int i = 0; i < basicMap.getIntersectionManagers().size(); i++)
    {
      Area FOV = new Area(Sensor.sensorOn(vehicle).getArea());
      FOV.intersect(new Area(basicMap.getIntersectionManagers().get(i).getShape()));
      return !FOV.isEmpty();
    }
    return false;                   //will jump to here if sensor not detecting IM or there are no IMs to loop through
  }

  synchronized private boolean inIntersection(VehicleSimView vehicle)
  {
    for (int i = 0; i < basicMap.getIntersectionManagers().size(); i++)
      if (basicMap.getIntersectionManagers().get(i).semiContains(vehicle))    //if this intersection (fully) contains the vehicle
      {
          return true;
      }
    return false;
  }

  synchronized public Sensor sensorOn(VehicleSimView vehicle)  //return a basic sensor (without aim or neat functionality)
  {
      return ((BasicAutoVehicle) vehicle).getSensor();
  }

  synchronized public NeatSensor neatSensorOn(VehicleSimView vehicle) //return a NEAT sensor (without aim or neat functionality)
    {
      return (((NeatSensor)((BasicAutoVehicle) vehicle).getSensor()));
    }

    synchronized public AimSensor aimSensorOn(VehicleSimView vehicle) //return an AIM sensor
    {
        return (((AimSensor)((BasicAutoVehicle) vehicle).getSensor()));
    }

  synchronized public Controller generalControllerIn(VehicleSimView vehicle)
  {
    return ((BasicAutoVehicle) vehicle).getController();
  }

  synchronized public AIMController aimControllerIn(VehicleSimView vehicle)
  {
    return (((AIMController) ((BasicAutoVehicle) vehicle).getController()));
  }

  synchronized public NEATController NEATControllerIn(VehicleSimView vehicle)
  {
    return (((NEATController) ((BasicAutoVehicle) vehicle).getController()));
  }


  /**
   * Allow each driver to act.
   */
  synchronized private void letDriversAct()
  {
    for(VehicleSimView vehicle : getActiveVehicles())
    {
      vehicle.getDriver().act();
    }
  }

  /////////////////////////////////
  // STEP 4
  /////////////////////////////////

  /**
   * Allow each intersection manager to act.
   *
   * @param timeStep  the time step
   */
  synchronized private void letIntersectionManagersAct(double timeStep)
  {
    for(IntersectionManager im : basicMap.getIntersectionManagers())
    {
      im.act(timeStep);
    }
  }

  /////////////////////////////////
  // STEP 5
  /////////////////////////////////

  /**
   * Deliver the V2I and I2V messages.
   */
  synchronized private void communication()
  {
    deliverV2IMessages();
    deliverI2VMessages();
  }

  /**
   * Deliver the V2I messages.
   */
  synchronized private void deliverV2IMessages()
  {
    // Go through each vehicle and deliver each of its messages
    for(VehicleSimView vehicle : vinToVehicles.values()) {
      // Start with V2I messages
      if (vehicle instanceof AutoVehicleSimView) {
        AutoVehicleSimView sender = (AutoVehicleSimView)vehicle;
        Queue<V2IMessage> v2iOutbox = sender.getV2IOutbox();
        while(!v2iOutbox.isEmpty()) {
          V2IMessage msg = v2iOutbox.poll();
          V2IManager receiver =
            (V2IManager)basicMap.getImRegistry().get(msg.getImId());
          // Calculate the distance the message must travel
          double txDistance =
            sender.getPosition().distance(
                receiver.getIntersection().getCentroid());

          if(Main.cfgController.equals("NEAT"))
          {
            txDistance = 1; //rudolf - added this
          }

          // Find out if the message will make it that far
          if(transmit(txDistance, sender.getTransmissionPower()))
          {
            // Actually deliver the message
            receiver.receive(msg);
            // Add the delivery to the debugging information
          }
          // Either way, we increment the number of transmitted messages
        }
      }
    }
  }

  /**
   * Deliver the I2V messages.
   */
  synchronized private void deliverI2VMessages()
  {

    // Now deliver all the I2V messages
    for(IntersectionManager im : basicMap.getIntersectionManagers()) {
      V2IManager senderIM = (V2IManager)im;
      for(Iterator<I2VMessage> i2vIter = senderIM.outboxIterator();
          i2vIter.hasNext();)
      {
        I2VMessage msg = i2vIter.next();
        AutoVehicleSimView vehicle =
          (AutoVehicleSimView)VinRegistry.getVehicleFromVIN(
            msg.getVin());


        // Calculate the distance the message must travel
        double txDistance;


        if(Main.cfgController.equals("NEAT"))       //rudolf - added this in
          {
          txDistance = 1;
        }
        else
        {
           txDistance =
          senderIM.getIntersection().getCentroid().distance(
                  vehicle.getPosition());
        }

        // Find out if the message will make it that far
        if(transmit(txDistance, senderIM.getTransmissionPower()))
        {
          if(vehicle != null)
            vehicle.receive(msg);
          // Actually deliver the message
        }

      }
      // Done delivering the IntersectionManager's messages, so clear the
      // outbox.
      senderIM.clearOutbox();
    }
  }



  /**
   * Whether the transmission of a message is successful
   *
   * @param distance  the distance of the transmission
   * @param power     the power of the transmission
   * @return whether the transmission of a messsage is successful
   */
  synchronized private boolean transmit(double distance, double power)
  {
    // Simple for now
    return distance <= power;
  }


  /////////////////////////////////
  // STEP 6
  /////////////////////////////////


  /**
   * Move all the vehicles.
   *
   * @param timeStep  the time step
   */
      synchronized private void moveVehicles(double timeStep) //#moveVehicles
      {
        for (VehicleSimView vehicle : vinToVehicles.values())
        {
          if (Main.cfgController.equals("NEAT"))
          {
            NeatSensor sensor = neatSensorOn(vehicle);
            if (!sensor.hasCrashed().get())            //only use NEAT controller if vehicle hasn't crashed
            {
              if (sensor.getObstructionDetected())
              {
                NEATControllerIn(vehicle).control();
                sensor.setObstructionDetected(false);  //we'll check obstructions from a "clean slate" in next timestep
              }
            }
          }

          if (Main.cfgController.equals("AIM"))
          {
            for (VehicleSimView v : getActiveVehicles())
            {
              if (aimSensorOn(v).isWaitingForObstruction().get() && intersectionInFOV(v))
              {
                if (Main.cfgAIMAntiCrashHeuristicEnabled)
                {
                  aimSensorOn(v).avoidCollisionHeuristic();                             //slow down to stop --> heuristic to avoid crashing
                  aimSensorOn(v).setWaitingForObstruction(new AtomicBoolean(false));
                }
              }
            }
          }

          Point2D posBeforeMove = vehicle.getPosition();                          //Vehicle's position before moving
          vehicle.move(timeStep);
          sensorOn(vehicle).moveWithVehicle();                                    //make sensor move with vehicle
          Point2D posAfterMove = vehicle.getPosition();                           //vehicle's position after moving

          Sensor sensor = sensorOn(vehicle);                                      //has general sensor functionality (that both AIM and NEAT have)

          for (DataCollectionLine line : basicMap.getDataCollectionLines())       //checkpoint lines on the map (1 at each end of each lane)
          {
            if (!sensor.hasCrashed().get())                                           //if vehicle hasn't crashed
            {
              if (line.intersect(vehicle, currentTime, posBeforeMove, posAfterMove))  //if vehicle hits this checkpoint
              {
                if (!sensor.getPassedCheckPointOne())                           //if checkpoint one (before entering intersection) has not been passed
                {
                  sensor.setPassedCheckPointOne(true, currentTime);                          //then this must be the first checkpoint, so tell the sensor we've passed checkpoint one
                } else                                                          //otherwise, if checkpoint one has been passed...
                {
                  sensor.setPassedCheckPointTwo(true, currentTime);                         //then we successfully traversed the intersection! So tell the sensor.
                  score += 0.05;                                               //award the NEAT controller 0.5 points for passing this checkpoint.
                  numIntersectionTraversals++;                                 //record this additional intersection traversal
                  traversalTimes.add(sensor.getTraversalTimeTaken());          //record how long it took this vehicle to traverse the intersection
                }
              }

            }

          }
        }
      }

  /**
   * Move all the drunk pedestrians - ie: make them walk.
   */
  public synchronized void moveDrunkPedestrians(double timeStep)
  {
    if(Main.cfgNumPedestrians > 0)                              //if there are more than 0 pedestrians on screen
    {
      for(DrunkPedestrian drunkPedestrian: drunkPedestrians)  //loop through pedestrian list
      {
        if(Main.cfgTimeStepsPedestriansWalk == -1)
        {
          drunkPedestrian.walk(timeStep);
        }
        else if(getSimulationTime() < Main.cfgTimeStepsPedestriansWalk)   //after this many timesteps have passed, pedestrians must stop walking and stand still
        {
          drunkPedestrian.walk(timeStep);                       //and make each one walk an additional time step
        }

      }
    }
  }

  /**
   * Deduct points for every timestep that a vehicle is touching the grass or is out of its correct lane
   */
  public synchronized void deductPointsForLeavingTrack()      // - rudolf
  {
    for(VehicleSimView vehicle : getActiveVehicles())
    {
      AutoDriver driver = (AutoDriver) vehicle.getDriver();
      Sensor sensor = sensorOn(vehicle);

      if (sensor.getPassedCheckPointOne())                //if the car has passed checkpoint one
      {
        if (!sensor.hasCrashed().get())   //if car hasn't crashed
        {
          if (!(driver.inCurrentIntersection()))  //if car is not in intersection
          {
            if (!VehicleUtil.intersects(vehicle, new Area(driver.getCurrentLane().getShape())))  //if it's not in its correct lane but is also within NEAT's allowed activation distance from the intersection
            {
              if (!sensor.getPassedCheckPointTwo())     //if it hasn't reached checkpoint two
              {
                score -= 0.025;                               //deduct points for leaving intersection illegally
                sensorOn(vehicle).setHasCrashed(true);
                crashedVehicles.add(vehicle);
              }
            }
          }
        }
      }
    }

  }


  public synchronized double getScore()   //return fitness score
  {
    return Math.pow(2,score);   //exponential function ensures we don't feed negative scores to NEAT
  }

  /////////////////////////////////
  // STEP 7
  /////////////////////////////////

  /**
   * Remove all completed vehicles.
   *
   * @return the VINs of the completed vehicles
   */
  synchronized private List<Integer> cleanUpCompletedVehicles()
  {
    List<Integer> completedVINs = new LinkedList<Integer>();

    Rectangle2D mapBoundary = basicMap.getDimensions();

    List<Integer> removedVINs = new ArrayList<Integer>(vinToVehicles.size());
    for(int vin : vinToVehicles.keySet()) {
      VehicleSimView v = vinToVehicles.get(vin);
      // If the vehicle is no longer in the layout
      // TODO: this should be replaced with destination zone.
      if(!v.getShape().intersects(mapBoundary)) {
        // Process all the things we need to from this vehicle
        if (v instanceof AutoVehicleSimView) {
          AutoVehicleSimView v2 = (AutoVehicleSimView)v;
          totalBitsTransmittedByCompletedVehicles += v2.getBitsTransmitted();
          totalBitsReceivedByCompletedVehicles += v2.getBitsReceived();
        }
        removedVINs.add(vin);
      }
    }
    // Remove the marked vehicles
    for(int vin : removedVINs) {
      vinToVehicles.remove(vin);
      completedVINs.add(vin);
      numOfCompletedVehicles++;
    }

    return completedVINs;
  }

  /////////////////////////////////
  // DEBUG
  /////////////////////////////////

  /**
   * Check whether the clocks are in sync.
   */
  synchronized private void checkClocks()
  {
    // Check the clocks for all autonomous vehicles.
    for(VehicleSimView vehicle: vinToVehicles.values())
    {
      vehicle.checkCurrentTime(currentTime);
    }
    // Check the clocks for all the intersection managers.
    for(IntersectionManager im : basicMap.getIntersectionManagers())
    {
      im.checkCurrentTime(currentTime);
    }
  }
}
