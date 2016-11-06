package aim4;

import aim4.config.SimConfig;
import aim4.map.DataCollectionLine;
import aim4.sim.AutoDriverOnlySimulator;
import aim4.sim.Simulator;
import aim4.sim.setup.AutoDriverOnlySimSetup;
import aim4.sim.setup.SimFactory;
import org.encog.neural.neat.NEATNetwork;

import java.util.concurrent.TimeUnit;


public class SimRunner
{
    AutoDriverOnlySimSetup simSetup;
    Simulator basicSim;
    AutoDriverOnlySimulator sim;


    public SimRunner(NEATNetwork network)
    {
        simSetup = new AutoDriverOnlySimSetup(Main.cfgColumns, // columns
                Main.cfgRows, // rows
                6, // lane width //make lanes wider
                10, // speed limit //make cars drive faster
                Main.cfgLanesPerRoad, // lanes per road
                1, // median size
                150, // distance between // -- to zoom in,  ++ to zoom out
                Main.cfgTrafficLevel, // traffic level
                1.0 // stop distance before intersection
        );

        basicSim = SimFactory.makeSimulator(simSetup);   //create basic simulator
        sim = ((AutoDriverOnlySimulator)basicSim);

        sim.setNetwork(network);
    }

    public synchronized double run(NEATNetwork network) //returns a score (fitness)
    {
        while(true)
        {
            try
            {
                sim.step(SimConfig.TIME_STEP);

                if(sim.getSimulationTime() > Main.cfgTimestepsPerSim)  //remember not to set cfgTimeStepsPerSim to -1 in config while training
                {
                    return sim.getScore();
                }

                TimeUnit.NANOSECONDS.sleep(10);                //wait for 10 nanoseconds

            } catch (Exception e)
            {
                e.printStackTrace();
                return -1;
            }
        }

    }

}
