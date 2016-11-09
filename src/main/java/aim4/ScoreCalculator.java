/*
 * Encog(tm) Java Examples v3.3
 * http://www.heatonresearch.com/encog/
 * https://github.com/encog/encog-java-examples
 *
 * Copyright 2008-2014 Heaton Research, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * For more information on Heaton Research copyrights, licenses
 * and trademarks visit:
 * http://www.heatonresearch.com/copyright
 */
package aim4;


import org.encog.ml.CalculateScore;
import org.encog.ml.MLMethod;
import org.encog.neural.neat.NEATNetwork;


public class ScoreCalculator implements CalculateScore
{
    public NEATNetwork network;

    public ScoreCalculator()
    {
    }

    @Override
    public synchronized double calculateScore(MLMethod mlMethod)                    //During evolution, this method will be called for each NN in the population each generation
    {
        network = (NEATNetwork)mlMethod;                                            //a NEAT neural network
        SimRunner trainer = new SimRunner(network);
        double score = -1;

        while(score == -1)                                                          //keep looping till we're not getting an error in sim (AIM loves to crash)
        {
            score = trainer.run(network);
        }

        System.out.println("NN score: "+score);                                     //fitness score of the neural network that just had its turn

        return score;
    }

    @Override
    public boolean shouldMinimize()
    {
        return false;
    }

    @Override
    public boolean requireSingleThreaded()
    {
        return false;
    }       //NB - true = multithreading turned off

}

