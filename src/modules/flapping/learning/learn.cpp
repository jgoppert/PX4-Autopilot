#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <inttypes.h>
#include <unistd.h>
#include <time.h>

/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *   Author: James Goppert, Ben Perseghetti, Scott Yantek
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

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
