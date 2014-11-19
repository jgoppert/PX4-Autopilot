#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <inttypes.h>
#include <unistd.h>
#include <time.h>

#include <stdio.h>

#include "GeneticAlgorithm.hpp"

int main(int argc, char * argv[]) {

	// random seed
	srand(1232);

	// static allocation of data
	static const uint16_t populationSize = 6;
	float reproducingRatio = 0.34;
	float mutateProb = 0.05;
	Genome genomeArray[populationSize];
	Genome genomeNextArray[populationSize];
	float fitnessArray[populationSize];
	GeneticAlgorithm ga = GeneticAlgorithm(
		populationSize,
		reproducingRatio,
		mutateProb,
		genomeArray,
		genomeNextArray,
		fitnessArray);

	for (int i=0; i<42; i++) {
		float g0 = ga.getTestNorm(0);
		float g1 = ga.getTestNorm(1);
		float t0 = 0.9;
		float t1 = 0.6;
		float fitness = 1 -((g0 - t0)*(g0 - t0)  + (g1 - t1)*(g1 - t1));
		ga.reportFitness(fitness);
		ga.nextTest();
	};
	printf("\n");
}
