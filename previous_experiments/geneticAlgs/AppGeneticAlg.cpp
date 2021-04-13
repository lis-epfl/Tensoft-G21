/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file AppNestedTetrahedrons.cpp
 * @brief Contains the definition function main() for the Nested Tetrahedrons
 * application.
 * @author Jean Marc Bejjani
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */


// The C++ Standard Library
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <sstream>

//The data logging library

#include <ga/ga.h>
#include <ga/std_stream.h>
#include <vector>

#include <time.h>
/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */

float objective(GAGenome &);


int count=0;
int nbActuators = 0;

struct Module
{
	int order;
	std::vector<int> connected_modules;
	std::vector<int> connected_faces;

	double frequency;
	double amplitude;
	double phase;
	int rotation;
} ;

std::ofstream logSpecies;

int population=30;

int main(int argc, char** argv)
{




	time_t time0;   // create timers.
	time_t time1;

	time(&time0);   // get current time.
	// GA parameters

	GAParameterList params;
	GASimpleGA::registerDefaultParameters(params);
	GASteadyStateGA::registerDefaultParameters(params);
	GAIncrementalGA::registerDefaultParameters(params);
	params.set(gaNpopulationSize, population);	// population size
	params.set(gaNpCrossover, 0.8);	// probability of crossover
	params.set(gaNpMutation, 0.02);	// probability of mutation
	params.set(gaNnGenerations, 120);	// number of generations
	params.set(gaNpReplacement, 0.90);	// how much of pop to replace each gen
	params.set(gaNscoreFrequency, 1);	// how often to record scores
	params.set(gaNflushFrequency, 5);	// how often to dump scores to file
	params.set(gaNscoreFilename, "bog.dat");
	params.parse(argc, argv, gaFalse);    // parse command line for GAlib args


	 unsigned int seed = 0;


	// Declare variables for the GA parameters and set them to some default values.

	/*int popsize  = 100;
	int ngen     = 70;
	float pmut   = 0.01;
	float pcross = 0.6;*/



	// Create a phenotype for all variables.  The number of bits you can use to
	// represent any number is limited by the type of computer you are using.  In
	// this case, we use 16 bits to represent a floating point number whose value
	// can range from -5 to 5, inclusive.  The bounds on x1 and x2 can be applied
	// here and/or in the objective function.

    std::ifstream inFile;



    inFile.close();

	GABin2DecPhenotype map;

    nbActuators = 9;
    for (int i; i < nbActuators ; i++)
	{
		map.add(16, 0, 1);
		map.add(16, 0, 1.0);
		map.add(16, 0, 6.28);
		map.add(16,0,4);
	}


	map.add(16,1,nbActuators);

	std::cout << "number of actuators: " << nbActuators << std::endl;

	GABin2DecGenome genome(map, objective);

	GASteadyStateGA ga(genome);
	ga.parameters(params);


	//run the simulation i times

	/*GASimpleGA ga(genome);
	GASigmaTruncationScaling scaling;
	ga.populationSize(popsize);
	ga.nGenerations(ngen);
	ga.pMutation(pmut);
	ga.pCrossover(pcross);
	ga.scaling(scaling);
	ga.scoreFilename("bog.dat");
	ga.scoreFrequency(1);
	ga.flushFrequency(50);
	ga.selectScores(GAStatistics::AllScores);
	ga.evolve(seed);*/



	// Dump the results of the GA to the screen.

  logSpecies.open("linear_species.csv");
	ga.selectScores(GAStatistics::AllScores);
	ga.evolve(seed);

	genome = ga.statistics().bestIndividual();
	std::cout << "the ga found an optimum at the point: " << std::endl;
	for(int i=0; i<genome.length()/16;i++)
	{
		if(i%4==0)
		{
			std::cout << "f" << i/4 << "="<<genome.phenotype(i) << std::endl;
		}
		if(i%4==1)
		{
			std::cout << "amp" << i/4 << "="<<genome.phenotype(i) << std::endl;
		}
		if(i%4==2)
		{
			std::cout << "phase" << i/4 << "="<<genome.phenotype(i) << std::endl;
		}
		if(i%4==2)
		{
			std::cout << "rot" << i/4 << "="<<int(genome.phenotype(i)) << std::endl;
		}
	}

	std::cout << "best of generation data are in '" << ga.scoreFilename() << "'\n";
	std::ofstream outFile;


	std::cout << "hey" << std::endl;

    outFile.open("input_sim.txt");


	for(int i=0; i<genome.phenotype(4*nbActuators)-1 ; i++)
	{
        outFile << "order: "<<i+1<<" connectedModules: "<<i+2<<" connectedFaces: "<<1<<" freq: "<<genome.phenotype(4*i)<<" amplitude: "
				<<genome.phenotype(4*i+1)<<" phase: "<<genome.phenotype(4*i+2) << " rot: " << int(genome.phenotype(4*i+3)) <<std::endl;
	}
	outFile << "order: "<<int(genome.phenotype(4*nbActuators))+1<<" connectedModules: "<<0<<" connectedFaces: "<<0<<" freq: "
			<<genome.phenotype(4*(nbActuators-1))<< " amplitude: "<<genome.phenotype(4*(nbActuators-1)+1)<<" phase: "
			<<genome.phenotype(4*(nbActuators-1)+2)<<" rot: " << int(genome.phenotype(4*(nbActuators-1)+3))<<std::endl;
	outFile.close();


	logSpecies.close();


    return 0;
}
float
objective(GAGenome & c)
{

	if(count%population==0)
	{
			logSpecies << std::endl;
	}

	GABin2DecGenome & genome = (GABin2DecGenome &)c;
	std::ofstream outFile;

	float fitness=0;

    outFile.open("input_sim.txt");

	std::cout<< "nb modules: " << int(genome.phenotype(4*nbActuators))+1 << std::endl;

	logSpecies << int(genome.phenotype(4*nbActuators)-1) << ", ";


	for(int i=0; i<genome.phenotype(4*nbActuators)-1 ; i++)
	{
        outFile << "order: "<<i+1<<" connectedModules: "<<i+2<<" connectedFaces: "<<1<<" freq: "<<genome.phenotype(4*i)<<" amplitude: "
				<<genome.phenotype(4*i+1)<<" phase: "<<genome.phenotype(4*i+2)<< " rot: " << int(genome.phenotype(4*i+3))<<std::endl;
	}
	outFile << "order: "<<int(genome.phenotype(4*nbActuators))+1<<" connectedModules: "<<0<<" connectedFaces: "<<0<<" freq: "
			<<genome.phenotype(4*(nbActuators-1))<<" amplitude: "<<genome.phenotype(4*(nbActuators-1)+1)<<" phase: "
			<<genome.phenotype(4*(nbActuators-1)+2)<<" rot: " << int(genome.phenotype(4*(nbActuators-1)+3))<<std::endl;
	outFile.close();

    system("../build/movingBall_sim/AppMovingBall_sim");

	std::ifstream inFile;

    inFile.open("output_sim.txt");

	if (!inFile) {
        std::cerr << "Unable to open file input_2.txt";
		exit(1);   // call system to stop
	}
	std::string line="";

	while (std::getline(inFile, line))
	{

		//loading line
		std::cout << line << std::endl;
		std::istringstream iss(line);

		//skipping "order:"
		iss >> fitness;
	}
	inFile.close();

	std::cout << "simulation number:" << count << std::endl;
	std::cout << "fitness:" << fitness << std::endl;

	count++;

	return fitness;
}
