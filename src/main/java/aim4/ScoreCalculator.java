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

        double score1 = trainer.run(network);
        double score2 = trainer.run(network);
        double score3 = trainer.run(network);
        double score4 = trainer.run(network);
        double score5 = trainer.run(network);

        double finalScore = (score1+score2+score3+score4+score5)/5;   //final score is average score over three trials

        System.out.println("NN score: "+finalScore);  //fitness score of the neural network that just had its turn (averaged over three trials)

        return finalScore;
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

