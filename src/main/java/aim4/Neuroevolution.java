package aim4;

import aim4.gui.Viewer;
import aim4.sim.setup.AutoDriverOnlySimSetup;
import aim4.util.Util;
import org.encog.Encog;
import org.encog.ml.ea.train.basic.TrainEA;
import org.encog.neural.neat.NEATNetwork;
import org.encog.neural.neat.NEATPopulation;
import org.encog.neural.neat.NEATUtil;
import org.encog.persist.EncogDirectoryPersistence;
import org.encog.util.Stopwatch;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.PrintStream;

public class Neuroevolution
{
    final static private boolean SAVE_BEST_NETWORK = true;
    final static private boolean VIEW_SIMULATION_OF_BEST_NETWORK = false;
    final static private int NUM_DEMO_SIMULATIONS_OF_BEST_NETWORK = 1000;

    final static private int POPULATION_SIZE = Main.cfgPOPULATION_SIZE;
    final static private int NUMBER_OF_GENERATIONS = Main.cfgNUMBER_OF_GENERATIONS;
    final static private double INITIAL_POP_DENSITY = Main.cfgINITIAL_POP_DENSITY;

    public Neuroevolution()
    {
        Main.cfgNumSimulations = NUM_DEMO_SIMULATIONS_OF_BEST_NETWORK;
        runEvolution();
    }

    //REMEMBER: Species are run in parallel!

    public synchronized void runEvolution()                                              //training NEATController
    {
        ScoreCalculator scoreCalculator = new ScoreCalculator();                         //fitness function

        NEATPopulation pop = new NEATPopulation(5,2,POPULATION_SIZE);                    //inputs, outputs, population size
        pop.setInitialConnectionDensity(INITIAL_POP_DENSITY);
        pop.reset();

        TrainEA evolution = NEATUtil.constructNEATTrainer(pop,scoreCalculator);

        Stopwatch stopwatch = new Stopwatch();
        stopwatch.start();

        //number of scores printed =  (populationSize * numGenerations) + 2


        //this code just makes it easier for me to see how far training is (I ssh into lab PCs and check the currentGeneration txt file)
        //---------------------------
        int resultSet = 1;
        String path = "TrainingResults/Results"+resultSet;
        File resultsDir = new File(path);
        try
        {
            while (resultsDir.exists())         //if we already have a results folder with this name
            {
                path = "TrainingResults/Results"+(++resultSet);
                resultsDir = new File(path);
            }
            resultsDir.mkdir();                 //create the directory which will store training results of this session
        PrintStream consoleOutputTxt = new PrintStream(new File(resultsDir+"/ConsoleOutput.txt"));
        System.setOut(consoleOutputTxt);
        }
        catch(Exception e){e.printStackTrace();}


        //copy the config that was used for this training session to the folder containing results of this training session
        try
        {
            File sourceFile = new File("config.txt");
            File destinationFile = new File(resultsDir+"/"+sourceFile.getName());

            FileInputStream fileInputStream = new FileInputStream(sourceFile);
            FileOutputStream fileOutputStream = new FileOutputStream(destinationFile);

            int bufferSize;
            byte[] bufffer = new byte[512];
            while ((bufferSize = fileInputStream.read(bufffer)) > 0)
            {
                fileOutputStream.write(bufffer, 0, bufferSize);
            }
            fileInputStream.close();
            fileOutputStream.close();
        }
        catch(Exception e)
        {
            e.printStackTrace();
        }

        //---------------------------

        System.out.println("Starting training with "+NUMBER_OF_GENERATIONS+" generations and population size of "+POPULATION_SIZE +" NNs per generation.\n-------------");

        for(int i = evolution.getIteration(); i < NUMBER_OF_GENERATIONS; i++)
        {
            System.out.println("Running generation "+i+" ("+evolution.getPopulation().getSpecies().size()+" species currently)"+"\n-------------");

            evolution.iteration();
        }
        evolution.finishTraining();                      //set the best performing genome = to the final population's best performing network (genome/genotype)

        stopwatch.stop();

        NEATNetwork bestPerformingNetwork = (NEATNetwork) evolution.getCODEC().decode(evolution.getBestGenome());   //extract best performing NN from the population

        String trainingLog =
                        "Evolution finished. Took " + Util.getDurationBreakdown(stopwatch.getElapsedMilliseconds())+"\n"+
                        "Number of training generations: " + NUMBER_OF_GENERATIONS+"\n"+
                        "Population size per generation: " + POPULATION_SIZE+"\n"+
                        "Timesteps per simulation: " + Main.cfgTimestepsPerSim+"\n"+
                        "Number of cars per simulation: " + Main.cfgNumVehicles+"\n"+
                        "Best genome was "+evolution.getBestGenome();

        System.out.println(trainingLog);                                                       //print results of training session (append to ConsoleOutput.txt)

        if(SAVE_BEST_NETWORK)   //Save the population of NNs, as well as the training log, to a subdirectory within TrainingResults
        {
            try
            {
                EncogDirectoryPersistence.saveObject(new File(resultsDir+"/Population.eg"), pop);      //save population of NNs to file (can later load it and extract best NN)
                PrintStream logStream = new PrintStream(new File(resultsDir+"/NetworkLog.txt"));       //save the training log (details) of this training session to file
                System.setOut(logStream);                                                             //redirect 'System.out' to the log file
                System.out.println(trainingLog);                                                      //write to log file
                System.setOut(System.out);                                                            //redirect 'System.out' back to the console
                System.out.println("Population.eg saved in "+resultsDir);
            }
            catch(Exception e)
            {
                e.printStackTrace();
            }
        }

        if(VIEW_SIMULATION_OF_BEST_NETWORK)
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

            new Viewer(simSetup,bestPerformingNetwork,true).runSim();
        }



        Encog.getInstance().shutdown();
    }

    public static void main(String[] args)
    {
        new Neuroevolution();
    }
}