#ifndef GENETIC_ALGORITHM_HPP__
#define GENETIC_ALGORITHM_HPP__

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

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <inttypes.h>
#include <unistd.h>
#include <time.h>

#include <stdio.h>

#ifndef RAND_MAX
#define RAND_MAX MAX_RAND 
#endif

/**
 * Class to simplify binary genome manipulation.
 */
class Genome {
private:
	uint16_t m_genome;
	static const uint8_t m_numberGenes;
	static const uint8_t m_geneLengths[];
	static const uint8_t m_genomeLength;
	static const uint8_t m_genePositions[];
public:
	Genome()  :
		m_genome(0) {
	}
	void setGenome(uint16_t genome) {
		m_genome = genome;
	}
	float getGeneNorm(uint8_t geneIndex) {
		return float(getGene(geneIndex)) /
			((1 << m_geneLengths[geneIndex]) - 1);
	}
	uint16_t getGenome() {
		return m_genome;
	}
	void randomize() {
		uint32_t searchSpace = 1 << m_genomeLength;
		m_genome = searchSpace*float(rand())/RAND_MAX;
	}
	void print() {
		for (int i=m_genomeLength-1; i>=0; i--) {
			printf("%d", ((1 << i) & m_genome) >> i);
		}
		printf(", %3d", m_genome);
		for (int i=0; i<m_numberGenes; i++) {
			printf(", %4.2f", double(getGeneNorm(i)));
		}
	}
	static Genome crossOver(Genome & parent1, Genome & parent2) {
		uint8_t crossoverPoint = (1 + m_genomeLength)*
			float(rand())/RAND_MAX;
		uint16_t crossoverMask = (1 << crossoverPoint) - 1;
		//printf("crossover point: %d\n", crossoverPoint);
		//printf("mask: ");
		//printBinary(crossoverMask, getGenomeLength());
		//printf("\n");
		uint16_t childGenome = 
			(parent1.m_genome & crossoverMask) |
			(parent2.m_genome & (~crossoverMask) );
		Genome child;
		child.setGenome(childGenome);
		return child;
	}
	void mutate(float mutateProb) {
		for (int bit=0; bit<m_genomeLength; bit++) {
			if (float(rand())/RAND_MAX < mutateProb) {
				m_genome ^= (1 << bit);
			}
		}
	}

private:
	uint16_t getGeneMask(uint8_t geneIndex) {
		return (1 << (m_genePositions[geneIndex] + m_geneLengths[geneIndex])) -
		(1 << (m_genePositions[geneIndex]));
	}
	uint16_t getGene(uint16_t geneIndex) {
		return (getGeneMask(geneIndex) & m_genome) >>
			m_genePositions[geneIndex];
	}
};

/**
 * GA class
 */
class GeneticAlgorithm {
private:
	uint16_t m_populationSize;
	float m_reproducingRatio;
	float m_mutateProb;
	uint8_t m_nGenes;
	uint8_t * m_geneLengths;
	Genome * m_genomeArray;
	Genome * m_genomeNextArray;
	float * m_fitnessArray;
	uint32_t m_generation;
	uint16_t m_genomeTestId;
	static const float m_nullFitness;
	Genome m_bestGenome;
	float m_bestFitness;
public:
	typedef uint16_t id_t;

	/**
	 * Constructor
	 */
	GeneticAlgorithm(uint8_t populationSize,
		float reproducingRatio,
		float mutateProb,
		Genome * genomeArray,
		Genome * genomeNextArray,
		float * fitnessArray) :
			m_populationSize(populationSize),
			m_reproducingRatio(reproducingRatio),
			m_mutateProb(mutateProb),
			m_genomeArray(genomeArray),
			m_genomeNextArray(genomeNextArray),
       			m_fitnessArray(fitnessArray),
       			m_generation(0),
       			m_genomeTestId(0),
			m_bestGenome(),
			m_bestFitness(m_nullFitness)
       	{

		// randomize initial population
		for (id_t genomeId=0;
				genomeId < m_populationSize;
				genomeId++) {
			m_genomeArray[genomeId].randomize();
			m_fitnessArray[genomeId] = m_nullFitness;
		}
		printPopulation();
       		m_generation++;
	}

	void setMutateProb(float mutateProb) {
		m_mutateProb = mutateProb;
	}

	void setReproducingRatio(float reproducingRatio) {
		m_reproducingRatio = reproducingRatio;
	}

	/**
	 * Returns value to test (0-1)
	 */
	float getTestNorm(uint8_t geneIndex) {
		return m_genomeArray[m_genomeTestId].getGeneNorm(geneIndex);
	}

	/**
	 * Report fitness for current test.
	 */
	void reportFitness(float fitness) {
		// if a new value for best genome is given, overwrite it
		if (m_genomeArray[m_genomeTestId].getGenome() == m_bestGenome.getGenome()) {
			m_bestFitness = fitness;
		}
		m_fitnessArray[m_genomeTestId] = fitness;
		printFitness(m_genomeTestId);
		printf("\n");
	}

	/**
	 * Increment test
	 */
	void nextTest() {
		if (++m_genomeTestId >= m_populationSize) {
			m_genomeTestId = 0;

			sortPopulation();
			if (m_fitnessArray[0] > m_bestFitness) {
				m_bestFitness = m_fitnessArray[0];
				m_bestGenome = m_genomeArray[0];
			}
			printPopulation();

			reproduce();
			m_generation++;

			//printf("after reproducing\n");
			//printPopulation();
		}
		printf("T ");
		printId(m_genomeTestId);
		printf(", ");
	}

	/**
	 * Print the entire population
	 */
	void printPopulation() {
		printf("*********************************************\n");
		printf("\nG: %d\n", m_generation);
		printf("BEST TRANS-GENERATION\n");
		m_bestGenome.print();
		printf(" (%4.2f)\n", double(m_bestFitness));
		printf("*********************************************\n");
		for (id_t genomeId=0;
		     genomeId < m_populationSize;
		     genomeId++) {
			printf("P ");
			printId(genomeId);
			printf(", ");
			printFitness(genomeId);
			printf("\n");
		}
		printf("---------------------------------------------\n");
	}

private:

	/**
	 * Randomly draw a parent from the top of the
	 * reproducing ratio
	 */
	id_t drawParent() {
		return m_reproducingRatio*m_populationSize*
			float(rand())/RAND_MAX;
	}

	/**
	 * Create a new generation
	 */
	void reproduce() {
		m_genomeNextArray[0] = m_bestGenome;
		for (id_t genomeId=1;
			genomeId < m_populationSize;
			genomeId++) {
			id_t parent1 = drawParent();
			id_t parent2 = drawParent();
			Genome child = Genome::crossOver(
					m_genomeArray[parent1],
					m_genomeArray[parent2]);
			child.mutate(m_mutateProb);
			m_fitnessArray[genomeId] = m_nullFitness;
			m_genomeNextArray[genomeId] = child;

		}
		m_genomeArray = m_genomeNextArray;
	}

	/**
	 * Print an individual from id
	 */
	void printId(id_t genomeId) {
		printf("%d: ", genomeId);
		m_genomeArray[genomeId].print();
	}

	void printFitness(id_t genomeId) {
		printf("(%4.2f)", double(m_fitnessArray[genomeId]));
	}

	/**
	 * Swap two id's for sort etc.
	 */
	void swapId(id_t id1, id_t id2) {
		float fitness1 = m_fitnessArray[id1];
		m_fitnessArray[id1] = m_fitnessArray[id2];
		m_fitnessArray[id2] = fitness1;
		Genome genome1 = m_genomeArray[id1];
		m_genomeArray[id1] = m_genomeArray[id2];
		m_genomeArray[id2] = genome1;
	}

	/**
	 * Do sorting (bubble sort)
	 */
	void sortPopulation() {
		uint8_t n = m_populationSize;
		for (int i = 0; i < (n-1); i++) {
			for (int j = 0; j < n - i - 1; j++) {
				if (m_fitnessArray[j] < m_fitnessArray[j+1]) {
					swapId(j, j+1);
				}
			}
		}
	}

};

#endif // GENETIC_ALGORITHM_HPP__
