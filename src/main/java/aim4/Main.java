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
package aim4;

import aim4.gui.Viewer;
import aim4.sim.setup.AutoDriverOnlySimSetup;
import aim4.sim.setup.BasicSimSetup;
import org.encog.neural.neat.NEATCODEC;
import org.encog.neural.neat.NEATNetwork;
import org.encog.neural.neat.NEATPopulation;
import org.encog.persist.EncogDirectoryPersistence;

import java.awt.geom.Point2D;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
/**
 * The default main class to show the GUI.
 */
public class Main
{
  /////////////////////////////////
  // THE MAIN FUNCTION
  /////////////////////////////////

  /**
   * The main function of the simulator.
   * It starts the GUI.
   *
   * @param args  the command-line arguments; it should be empty since the GUI
   *              does not take any command-line arguments
   *
   */

  // create the basic setup

  static BasicSimSetup simSetup;

  // config variables
  public static String cfgController;
  public static double cfgSimSpeed;
  public static int cfgLanesPerRoad;
  public static double cfgTrafficLevel;
  public static int cfgNumVehicles;
  public static double cfgMaxAccel;
  public static double cfgMaxDecel;
  public static double cfgMaxVelocity;
  public static double cfgMinVelocity;
  public static double cfgSensorHeight;
  public static double cfgSensorWidth;
  public static double cfgSensorAngleStart;
  public static double cfgSensorAngleEnd;
  public static int cfgColumns;
  public static int cfgRows;
  public static char cfgCarSpecMode;
  public static double cfgVehicleLength;
  public static double cfgVehicleWidth;
  public static double cfgPedestrianHeight;
  public static double cfgPedestrianWidth;
  public static int cfgNumPedestrians;
  public static boolean cfgShowVin;
  public static double cfgDegreesBetweenPtsFOV;
  public static ArrayList<Point2D.Double[]> cfgPedestrianWaypoints = new ArrayList<Point2D.Double[]>();
  public static int cfgNumSimulations;
  public static double cfgTimestepsPerSim;
  public static double cfgNearestObstructionPrecision;
  public static String cfgNEATSetting;
  public static String cfgNetwork;  //network to use from controller (must be in 'Saved Networks' folder)
  public static double cfgLinePosition; //draw a checkpoint line on a lane at Nth (where N = this variable) metre along the lane
  public static boolean cfgDrawSensorFOVs;
  public static double cfgTimeStepsPedestriansWalk;

  public static double cfgCompatibilityThreshold;
  public static int cfgNumGensAllowedNoImprovement;
  public static int cfgMaxNumberOfSpecies;
  public static int cfgPOPULATION_SIZE;
  public static int cfgNUMBER_OF_GENERATIONS;
  public static double cfgINITIAL_POP_DENSITY;



  public Main()
  {
    readConfig();
    start();
  }

  public synchronized void start()
  {

    if(cfgController.equals("NEAT"))
    {
      if(cfgNEATSetting.equals("Train"))
      {
        trainNEAT();
      }
      else if(cfgNEATSetting.equals("Demo"))
      {
        demoNEAT();
      }
    }

    else if(cfgController.equals("AIM"))
    {
      demoAIM();
    }
  }

  public synchronized void trainNEAT()
  {
    Neuroevolution NE = new Neuroevolution();
  }

  public synchronized void demoNEAT()
  {
    AutoDriverOnlySimSetup simSetup = new AutoDriverOnlySimSetup(Main.cfgColumns, // columns
            Main.cfgRows, // rows
            6, // lane width //make lanes wider
            10, // speed limit //make cars drive faster
            Main.cfgLanesPerRoad, // lanes per road
            1, // median size
            150, // distance between // -- to zoom in,  ++ to zoom out
            Main.cfgTrafficLevel, // traffic level
            1.0 // stop distance before intersection
    );

    NEATPopulation pop = (NEATPopulation) EncogDirectoryPersistence.loadObject(new File(cfgNetwork));  //get saved population

    NEATNetwork bestNetwork = (NEATNetwork) new NEATCODEC().decode(pop.getBestGenome());        //extract best genome from saved population

    new Viewer(simSetup,bestNetwork,true).runSim();                                         //convert best genome to NEATNetwork that can control a vehicle
  }

  public synchronized void demoAIM()
  {
    AutoDriverOnlySimSetup simSetup = new AutoDriverOnlySimSetup(Main.cfgColumns, // columns
            Main.cfgRows, // rows
            6, // lane width //make lanes wider
            10, // speed limit //make cars drive faster
            Main.cfgLanesPerRoad, // lanes per road
            1, // median size
            150, // distance between // -- to zoom in,  ++ to zoom out
            Main.cfgTrafficLevel, // traffic level
            1.0 // stop distance before intersection
    );

    new Viewer(simSetup,true).runSim();
  }

  public synchronized void readConfig()
  {
    try //try to read from config.txt
    {
      BufferedReader f = new BufferedReader(new FileReader("config.txt"));
      String s = f.readLine();

      while(s!=null)
      {
        if(s.contains("Lanes per road:"))
          cfgLanesPerRoad = Integer.parseInt(s.split(":")[1].replace(" ",""));
        if(s.contains("Traffic level (max 0.7):"))
          cfgTrafficLevel = Double.parseDouble(s.split(":")[1].replace(" ", ""));
        if(s.contains("Number of cars per simulation:"))
          cfgNumVehicles = Integer.parseInt(s.split(":")[1].replace(" ", ""));
        if(s.contains("Max acceleration:"))
          cfgMaxAccel = Double.parseDouble(s.split(":")[1].replace(" ", ""));
        if(s.contains("Max deceleration:"))
          cfgMaxDecel = Double.parseDouble(s.split(":")[1].replace(" ",""));
        if(s.contains("Max velocity:"))
          cfgMaxVelocity = Double.parseDouble(s.split(":")[1].replace(" ",""));
        if(s.contains("Min velocity:"))
          cfgMinVelocity = Double.parseDouble(s.split(":")[1].replace(" ",""));
        if(s.contains("Simulation speed:"))
          cfgSimSpeed = Double.parseDouble(s.split(":")[1].replace(" ",""));
        if(s.contains("Height of FOV:"))
          cfgSensorHeight = Double.parseDouble(s.split(":")[1].replace(" ",""));
        if(s.contains("Width of FOV:"))
          cfgSensorWidth = Double.parseDouble(s.split(":")[1].replace(" ",""));
        if(s.contains("Angle start:"))
          cfgSensorAngleStart = Double.parseDouble(s.split(":")[1].replace(" ",""));
        if(s.contains("Angle end:"))
          cfgSensorAngleEnd = Double.parseDouble(s.split(":")[1].replace(" ",""));
        if(s.contains("Controller (AIM/NEAT):"))
          cfgController = s.split(":")[1].replace(" ", "");
        if(s.contains("Size of GridMap (COLSxROWS):"))
          cfgColumns = Integer.parseInt((s.split(":")[1].replace(" ", "")).split("x")[0]);
        if(s.contains("Size of GridMap (COLSxROWS):"))
          cfgRows = Integer.parseInt((s.split(":")[1].replace(" ","")).split("x")[1]);
        if(s.contains("Custom or Random? (C / R):"))
          cfgCarSpecMode = (s.split(":")[1].replace(" ","")).charAt(0);
        if(s.contains("Length of car:"))
          cfgVehicleLength = Double.parseDouble(s.split(":")[1].replace(" ", ""));
        if(s.contains("Width of car:"))
          cfgVehicleWidth = Double.parseDouble(s.split(":")[1].replace(" ",""));
        if(s.contains("Height of drunk pedestrian:"))
          cfgPedestrianHeight = Double.parseDouble(s.split(":")[1].replace(" ",""));
        if(s.contains("Width of drunk pedestrian:"))
          cfgPedestrianWidth = Double.parseDouble(s.split(":")[1].replace(" ",""));
        if(s.contains("Show vehicle VINs on screen:"))
          cfgShowVin = (s.split(":")[1].replace(" ","")).equalsIgnoreCase("Yes");
        if(s.contains("Pedestrian waypoint:"))
        {
          s = s.split(":")[1].replace(" ","").replace("(","").replace(")","");
          double x1 = Double.parseDouble(s.split(",")[0]);
          double y1 = Double.parseDouble(s.split(",")[1]);
          double x2 = Double.parseDouble(s.split(",")[2]);
          double y2 = Double.parseDouble(s.split(",")[3]);
          Point2D.Double[] waypoint = {new Point2D.Double(x1,y1),new Point2D.Double(x2,y2)};
          cfgPedestrianWaypoints.add(waypoint);
          cfgNumPedestrians = cfgPedestrianWaypoints.size();
        }
        if(s.contains("Degrees between each FOV point:"))
          cfgDegreesBetweenPtsFOV = Double.parseDouble(s.split(":")[1].replace(" ",""));
        if(s.contains("Number of simulations:"))
          cfgNumSimulations = Integer.parseInt(s.split(":")[1].replace(" ", ""));
        if(s.contains("Timesteps per sim (-1 = unlimited):"))
          cfgTimestepsPerSim = Integer.parseInt(s.split(":")[1].replace(" ",""));
        if(s.contains("Nearest obstruction precision (best = 0):"))
          cfgNearestObstructionPrecision = Double.parseDouble(s.split(":")[1].replace(" ", ""));
        if(s.contains("NEAT setting (Demo/Train):"))
          cfgNEATSetting = s.split(":")[1].replace(" ", "");
        if(s.contains("Network to use:"))
        {
          cfgNetwork = s.split(":")[1].replace(" ","");
        }
        if(s.contains("Metres along lane to draw checkpoint line:"))
          cfgLinePosition = Integer.parseInt(s.split(":")[1].replace(" ",""));
        if(s.contains("compatibilityThreshold:"))
          cfgCompatibilityThreshold = Double.parseDouble(s.split(":")[1].replace(" ",""));
        if(s.contains("numGensAllowedNoImprovement:"))
          cfgNumGensAllowedNoImprovement = Integer.parseInt(s.split(":")[1].replace(" ",""));
        if(s.contains("maxNumberOfSpecies:"))
          cfgMaxNumberOfSpecies = Integer.parseInt(s.split(":")[1].replace(" ",""));
        if(s.contains("POPULATION_SIZE:"))
          cfgPOPULATION_SIZE = Integer.parseInt(s.split(":")[1].replace(" ",""));
        if(s.contains("NUMBER_OF_GENERATIONS:"))
          cfgNUMBER_OF_GENERATIONS = Integer.parseInt(s.split(":")[1].replace(" ",""));
        if(s.contains("INITIAL_POP_DENSITY:"))
          cfgINITIAL_POP_DENSITY = Double.parseDouble(s.split(":")[1].replace(" ",""));
        if(s.contains("Draw Sensor FOVs (yes/no):"))
        {
          if ((s.split(":")[1].replace(" ","")).contains("yes"))
            cfgDrawSensorFOVs = true;
          else if ((s.split(":")[1].replace(" ","")).contains("no"))
            cfgDrawSensorFOVs = false;

        }
        if(s.contains("Stop walking after this many timesteps:"))
          cfgTimeStepsPedestriansWalk = Double.parseDouble(s.split(":")[1].replace(" ",""));







        s = f.readLine();
      }
      f.close();
    }
    catch(Exception e)
    { e.printStackTrace();}
  }

  public static void main(String[] args)
  {
    new Main();
  }
}