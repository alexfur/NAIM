# Neuroevolution for Autonomous Intersection Management

## What is this?

This is the codebase I extended and used for an independent research programme I took part in during the second year of my degree. 

* It is based on the [AIM Project](http://www.cs.utexas.edu/~aim/), a centralised intersection management policy for autonomous vehicles.
* AIM uses a reservation system to coordinate intersections without traffic lights or other traditional coordination mechanisms.
* The goal of my project was to investigate a decentralised alternative to AIM. Namely, I used *Neuro-Evolution of Augmenting Topologies (NEAT)* for evolving individual vehicle control policies, so all that is needed is the control policy and vehicle sensors rather than relying on a centralised system.
* You can check out the [final paper](https://github.com/rudolfbono/NAIM/blob/master/FINAL-REPORT.pdf) I wrote for this.

## Tech

NAIM uses the following open source projects (all Java-based):

* [Encog] - Machine Learning Suite with support for Neuro-Evolution of Augmenting Topologies (NEAT)
* [AIM] - Simulator for research on autonomous management and coordination of vehicles at intersections.

## Setting up and Running

_Note that Gradle is used for management of dependencies and running._

 Clone and cd into this repository, then run it with Gradle:

```sh
$ ./gradlew run
```

By default, this will run evolution (NEAT) with the paramaters in the *config* file in the root of this project. To change these parameters, edit the config file.

## Best Solution I evolved

This gif shows the best control policy I evolved in action (150 individuals in population, 150 generations).

![Alt text](https://github.com/rudolfbono/NAIM/blob/master/NEAT%20demo.gif)
