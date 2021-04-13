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

#include <string>

#include <time.h>

//The data logging library

#include <ga/ga.h>
#include <ga/std_stream.h>
#include <vector>


/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */

float objective(GAGenome &);

void printInput(GAGenome &);

std::string removeSpaces(std::string str);

int count=0;

int population=40;

int actuatorsPerBranch[4] = {0,0,0,0};

std::ofstream logSpecies;

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


int main(int argc, char** argv)
{

	time_t time0;   // create timers.
	time_t time1;

	time(&time0);   // get current time.
	// GA parameters

	GAParameterList params;
	//GASimpleGA::registerDefaultParameters(params);
	GASteadyStateGA::registerDefaultParameters(params);
	//GAIncrementalGA::registerDefaultParameters(params);
	params.set(gaNpopulationSize, population);	// population size
	params.set(gaNpCrossover, 0.9);	// probability of crossover
	params.set(gaNpMutation, 0.02);	// probability of mutation
  params.set(gaNnGenerations, 199);	// number of generations
	params.set(gaNpReplacement, 0.90);	// how much of pop to replace each gen
	params.set(gaNscoreFrequency, 1);	// how often to record scores
	params.set(gaNflushFrequency, 5);	// how often to dump scores to file
  params.set(gaNscoreFilename, "bog.dat");
	params.parse(argc, argv, gaFalse);    // parse command line for GAlib args


	 unsigned int seed = 0;


	// Create a phenotype for all variables.  The number of bits you can use to
	// represent any number is limited by the type of computer you are using.  In
	// this case, we use 16 bits to represent a floating point number whose value
	// can range from -5 to 5, inclusive.  The bounds on x1 and x2 can be applied
	// here and/or in the objective function.

	std::ifstream inFile;

    inFile.open("input_sim.txt");

	if (!inFile) {
		std::cerr << "Unable to open file input.txt";
		exit(1);   // call system to stop
	}
	std::string line="";



	GABin2DecPhenotype map;


	for(int i=0;i<13;i++)
	{
        map.add(16, 0, 1.0);
        map.add(16, 0, 1.0);
		map.add(16, 0, 6.28);
		map.add(16,0,4);
	}
	inFile.close();

	map.add(16,1,4);
	map.add(16,1,4);
	map.add(16,1,4);
	map.add(16,0,1);


	GABin2DecGenome genome(map, objective);

	enum _GABoolean div= gaTrue;

	GASteadyStateGA ga(genome);
	ga.parameters(params);
	ga.recordDiversity(div);

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

  logSpecies.open("species.csv");
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
		if(i%4==3)
		{
			std::cout << "rotation" << i/4 << "="<<genome.phenotype(i) << std::endl;
		}
	}

	std::cout << "best of generation data are in '" << ga.scoreFilename() << "'\n";
	std::ofstream outFile;

	time(&time1);   // get current time.

	double seconds = time1 - time0;

	std::cout << "time: " << int(seconds/3600) <<"h" << int(seconds/60)-int(seconds/3600)*60 << "min" << int(seconds)%3600 << "s" <<'\n';

    outFile.open("input_sim.txt");

	printInput(genome);

	logSpecies.close();

/*

	for(int i=0; i<genome.phenotype(3*nbActuators)-1 ; i++)
	{
		outFile << "order: "<<i+1<<" connectedModules: "<<i+2<<" connectedFaces: "<<1<<" freq: "<<genome.phenotype(3*i)<<" amplitude: "
				<<genome.phenotype(3*i+1)<<" phase: "<<genome.phenotype(3*i+2)<<std::endl;
	}
	outFile << "order: "<<int(genome.phenotype(3*nbActuators))+1<<" connectedModules: "<<0<<" connectedFaces: "<<0<<" freq: "<<genome.phenotype(3*(nbActuators-1))<<
			" amplitude: "<<genome.phenotype(3*(nbActuators-1)+1)<<" phase: "<<genome.phenotype(3*(nbActuators-1)+2)<<std::endl;
	outFile.close();

    outFile.open("./NTRTsim-master/src/dev/GeneticAlg/results.txt");

	outFile.close();
*/

    return 0;
}

void printInput(GAGenome & c)
{
	GABin2DecGenome & genome = (GABin2DecGenome &)c;




	if(count%population==0)
	{
		logSpecies << std::endl;
	}

	std::ofstream outFile;
	std::cout<<"length: "<< genome.length() << std::endl;
    outFile.open("input_sim.txt");

		actuatorsPerBranch[0]=int(genome.phenotype(4*13));
		actuatorsPerBranch[1]=int(genome.phenotype(4*13+1));
		actuatorsPerBranch[2]=int(genome.phenotype(4*13+2));
		actuatorsPerBranch[3]=int(genome.phenotype(4*13+3));

		int nbActuators = actuatorsPerBranch[0] + actuatorsPerBranch[1] + actuatorsPerBranch[2] + actuatorsPerBranch[3];

		std::cout<< "nb modules: " << nbActuators << std::endl;

        //outFile.open("./NTRTsim-master/src/dev/GeneticAlg/test2.txt");

		std::stringstream cModules;
		std::string cFaces = " ";

		#define A_BIT (1 << 0)
		#define B_BIT (1 << 1)
		#define C_BIT (1 << 2)
		#define D_BIT (1 << 3)


		switch( (actuatorsPerBranch[0]? A_BIT : 0) | (actuatorsPerBranch[1]? B_BIT : 0) | (actuatorsPerBranch[2]? C_BIT : 0)
				| (actuatorsPerBranch[3]? D_BIT : 0))
		{
			 case 0:                     //none of the conditions holds true.
			 {
				 cModules << std::string("0");
				 cFaces = std::string("0");
			 } break;
			 case A_BIT:				 //condition A is true, everything else is false.
			 {
				 cModules << std::string("2");
				 cFaces = std::string("1");
			 } break;
			 case B_BIT:                 //condition B is true, everything else is false.
			 {
				 cModules << std::string("2");
				 cFaces = std::string("2");
			 } break;
			 case C_BIT:                 //condition C is true, everything else is false.
			 {
				 cModules << std::string("2");
				 cFaces = std::string("3");
			 } break;
			 case D_BIT:                 //condition C is true, everything else is false.
			 {
				 cModules << std::string("2");
				 cFaces = std::string("4");
			 } break;
			 case A_BIT + B_BIT:         //conditions A and B are true, C is false.
			 {
				 cModules << std::string("2 ") << 2+actuatorsPerBranch[0];
				 cFaces = "1 2";
			 } break;
			 case A_BIT + C_BIT:         //conditions A and C are true, B is false.
			 {
				 cModules << std::string("2 ") << 2+actuatorsPerBranch[0];
				 cFaces = "1 3";
			 } break;
			 case A_BIT + D_BIT:         //conditions B and C are true, A is false.
			 {
				 cModules << std::string("2 ") << 2+actuatorsPerBranch[0];
				 cFaces = "1 4";
			 } break;
			 case B_BIT + C_BIT:         //conditions B and C are true, A is false.
			 {
				 cModules << std::string("2 ") << 2+actuatorsPerBranch[1];
				 cFaces = "2 3";
			 } break;
			 case B_BIT + D_BIT:         //conditions B and C are true, A is false.
			 {
				 cModules << std::string("2 ") << 2+actuatorsPerBranch[1];
				 cFaces = "2 4";
			 } break;
			 case C_BIT + D_BIT:         //conditions B and C are true, A is false.
			 {
				 cModules << "2 " << 2+actuatorsPerBranch[2];
				 cFaces = "3 4";
			 } break;
			 case A_BIT + B_BIT + C_BIT: //all conditions are true.
			 {
				 cModules << std::string("2 ") << 2+actuatorsPerBranch[0] << " " << 2+actuatorsPerBranch[0]+actuatorsPerBranch[1];
				 cFaces = "1 2 3";
			 } break;
			 case A_BIT + B_BIT + D_BIT: //all conditions are true.
			 {
				 cModules << std::string("2 ") << 2+actuatorsPerBranch[0] << " " << 2+actuatorsPerBranch[0]+actuatorsPerBranch[1];
				 cFaces = "1 2 4";
			 } break;
			 case A_BIT + C_BIT + D_BIT: //all conditions are true.
			 {
				 cModules << std::string("2 ") << 2+actuatorsPerBranch[0] << " " << 2+actuatorsPerBranch[0]+actuatorsPerBranch[2];
				 cFaces = "1 3 4";
			 } break;
			 case B_BIT + C_BIT + D_BIT: //all conditions are true.
			 {
				 cModules << std::string("2 ") << 2+actuatorsPerBranch[1] << " " << 2+actuatorsPerBranch[1]+actuatorsPerBranch[2];
				 cFaces = "2 3 4";
			 } break;
			 case A_BIT + B_BIT + C_BIT + D_BIT: //all conditions are true.
			 {
				 cModules << std::string("2 ") << 2+actuatorsPerBranch[0] << " " << 2+actuatorsPerBranch[0]+actuatorsPerBranch[1]
						<< " " << 2+actuatorsPerBranch[0]+actuatorsPerBranch[1]+actuatorsPerBranch[2];
				 cFaces = "1 2 3 4";
			 } break;
			 default: assert(0);   //something went wrong with the bits.
		}

		logSpecies << removeSpaces(cModules.str()) << nbActuators << " ";

		outFile << "order: "<<1<<" connectedModules: "<<cModules.str()<<" connectedFaces: "<<cFaces<<" freq: "<<genome.phenotype(0)<<" amplitude: "
						<<genome.phenotype(1)<<" phase: "<<genome.phenotype(2)<< " rot: "<<int(genome.phenotype(3))<<std::endl;

		for(int i=0; i<actuatorsPerBranch[0] - 1 ; i++)
		{
			outFile << "order: "<<i+2<<" connectedModules: "<<i+3<<" connectedFaces: "<<1<<" freq: "<<genome.phenotype(4*i+4)<<" amplitude: "
					<<genome.phenotype(4*i+5)<<" phase: "<<genome.phenotype(4*i+6) << " rot: "<< int(genome.phenotype(4*i+7)) <<std::endl;
		}

		if(actuatorsPerBranch[0])
		{
			outFile << "order: "<<actuatorsPerBranch[0]+1<<" connectedModules: "<<0<<" connectedFaces: "<<0<<" freq: "
					<<genome.phenotype(4*(actuatorsPerBranch[0]-1)+4)<<" amplitude: "<<genome.phenotype(4*(actuatorsPerBranch[0]-1)+5)<<" phase: "<<
					genome.phenotype(4*(actuatorsPerBranch[0]-1)+6)<< " rot: "<< int(genome.phenotype(4*(actuatorsPerBranch[0]-1)+7))
					<<std::endl;
		}

		for(int i=0; i<actuatorsPerBranch[1] - 1 ; i++)
		{
			outFile << "order: "<<i+actuatorsPerBranch[0]+2<<" connectedModules: "<<i+actuatorsPerBranch[0]+3<<" connectedFaces: "<<2<<" freq: "
					<<genome.phenotype(4*i+4*actuatorsPerBranch[0]+4)<<" amplitude: " <<genome.phenotype(4*i+4*actuatorsPerBranch[0]+5)<<
					" phase: "<<genome.phenotype(4*i+4*actuatorsPerBranch[0]+6)<<
					" rot: "<< int(genome.phenotype(4*i+4*actuatorsPerBranch[0]+7))<<std::endl;
		}

		if(actuatorsPerBranch[1])
		{
			outFile << "order: "<<actuatorsPerBranch[0]+actuatorsPerBranch[1]+1<<" connectedModules: "<<0<<" connectedFaces: "<<0<<" freq: "
					<<genome.phenotype(4*(actuatorsPerBranch[1]-1)+4*actuatorsPerBranch[0]+4)<<" amplitude: "
					<<genome.phenotype(4*(actuatorsPerBranch[1]-1)+4*actuatorsPerBranch[0]+5)<<" phase: "<<
					  genome.phenotype(4*(actuatorsPerBranch[1]-1)+4*actuatorsPerBranch[0]+6)<< " rot: "<<
					  int(genome.phenotype(4*(actuatorsPerBranch[1]-1)+4*actuatorsPerBranch[0]+7))<<std::endl;
		}
		for(int i=0; i<actuatorsPerBranch[2] - 1 ; i++)
		{
			outFile << "order: "<<i+actuatorsPerBranch[0]+actuatorsPerBranch[1]+2<<" connectedModules: "
					<<i+actuatorsPerBranch[0]+actuatorsPerBranch[1]+3<<" connectedFaces: "<<3<<" freq: "
					<<genome.phenotype(4*i+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+4)<<" amplitude: "
					<<genome.phenotype(4*i+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+5)<<" phase: "
					<<genome.phenotype(4*i+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+6)<<" rot: "<<
					  int(genome.phenotype(4*i+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+7))<<std::endl;
		}

		if(actuatorsPerBranch[2])
		{
			outFile << "order: "<<actuatorsPerBranch[0]+actuatorsPerBranch[1]+actuatorsPerBranch[2]+1<<" connectedModules: "<<0
					<<" connectedFaces: "<<0
					<<" freq: "<<genome.phenotype(4*(actuatorsPerBranch[2]-1)+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+4)
					<<" amplitude: "<<genome.phenotype(4*(actuatorsPerBranch[2]-1)+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+5)
					<<" phase: "<<genome.phenotype(4*(actuatorsPerBranch[2]-1)+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+6)
					<<" rot: "<<int(genome.phenotype(4*(actuatorsPerBranch[2]-1)+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+7))
					<<std::endl;
		}
		for(int i=0; i<actuatorsPerBranch[3] - 1 ; i++)
		{
			outFile << "order: "<<i+actuatorsPerBranch[0]+actuatorsPerBranch[1]+actuatorsPerBranch[2]+2<<" connectedModules: "
					<<i+actuatorsPerBranch[0]+actuatorsPerBranch[1]+actuatorsPerBranch[2]+3<<" connectedFaces: "<<4<<" freq: "
					<<genome.phenotype(4*i+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+4*actuatorsPerBranch[2]+4)<<" amplitude: "
					<<genome.phenotype(4*i+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+4*actuatorsPerBranch[2]+5)<<" phase: "
					<<genome.phenotype(4*i+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+4*actuatorsPerBranch[2]+6)<<" rot: "
					<<int(genome.phenotype(4*i+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+4*actuatorsPerBranch[2]+7))<<std::endl;
		}

		if(actuatorsPerBranch[3])
		{
			outFile << "order: "<<actuatorsPerBranch[0]+actuatorsPerBranch[1]+actuatorsPerBranch[2]+actuatorsPerBranch[3]+1
					<<" connectedModules: "<<0
					<<" connectedFaces: "<<0
					<<" freq: "
					<<genome.phenotype(4*(actuatorsPerBranch[3]-1)+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+4*actuatorsPerBranch[2]+4)
					<<" amplitude: "
					<<genome.phenotype(4*(actuatorsPerBranch[3]-1)+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+4*actuatorsPerBranch[2]+5)
					<<" phase: "
					<<genome.phenotype(4*(actuatorsPerBranch[3]-1)+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+4*actuatorsPerBranch[2]+6)
					<<" rot: "
					<<int(genome.phenotype(4*(actuatorsPerBranch[3]-1)+4*actuatorsPerBranch[0]+4*actuatorsPerBranch[1]+4*actuatorsPerBranch[2]+7))
					<<std::endl;
		}
		outFile.close();


}
std::string removeSpaces(std::string str)
{
    std::stringstream ss;
    std::string temp;

    /* Storing the whole string into string stream */
    ss << str;

    /* Making the string empty */
    str = "";

    /* Running loop till end of stream */
    while (!ss.eof())
    {
        /* extracting word by word from stream */
        ss >> temp;

        /* concatenating in the string to be
          returned*/
        str = str + temp;
    }
    return str;
}
float
objective(GAGenome & c)
{


	float fitness=0;

	printInput(c);

    system("./NTRTsim-master/build/dev/MovingBall_sim/AppMovingBall_sim");

	std::ifstream inFile;

    inFile.open("output_sim.txt");

	if (!inFile) {
		std::cerr << "Unable to open file input.txt";
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
