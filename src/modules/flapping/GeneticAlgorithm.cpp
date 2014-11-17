/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *   Author: James Goppert, Scott Yantek
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
#include <stdio.h>

#include "GeneticAlgorithm.hpp"

// methods

GeneticAlgorithm::GeneticAlgorithm() :
		m_parentPercentage(0.5)
{
	// initialize gene positions and masks
	m_geneLengths[0] = 4;
	m_geneLengths[1] = 4;

	m_genePositions[0] = 0;
	for (int i=1; i< m_numberGenes; i++) {
		m_genePositions[i] = m_genePositions[i-1]
			+ m_geneLengths[i-1];
	}
	for (int i=0; i < m_numberGenes; i++) {
		m_geneMasks[i] = 
			(1 << (m_genePositions[i] + m_geneLengths[i])) -
			(1 << (m_genePositions[i]));
	}
	for (int i=0; i < (1 << m_genomeLength); i++) {
		setVisitedFitness(i, 5.0); //positive value since all valid fitness values are negative
	}

	// random initialization of population
	uint32_t maxGenome = 1 << getGenomeLength();
	//printf("max genome: %d\n", maxGenome);
	for (int id=0; id < getPopulationSize() ; id++) {
		setGenome(id, maxGenome*float(rand())/RAND_MAX);
		setFitness(id, -1e9); 
	}
	setBestGenome(getGenome(0));
	setBestFitness(getFitness(0));
	setGeneration(0);
	setCurrentId(0);
}

void GeneticAlgorithm::swapId(uint8_t id1, uint8_t id2) {
	float fitness1 = getFitness(id1);
	setFitness(id1, getFitness(id2));
	setFitness(id2, fitness1);
	uint16_t genome1 = getGenome(id1);
	setGenome(id1, getGenome(id2));
	setGenome(id2, genome1);
}

void GeneticAlgorithm::bubbleSort() {
	uint8_t n = getPopulationSize();
	for (int i = 0; i < (n-1); i++) {
		for (int j = 0; j < n - i - 1; j++) {
			if (getFitness(j) < getFitness(j+1)) {
				swapId(j, j+1);
			}
		}
	}
}

uint8_t GeneticAlgorithm::selectParent() {
	// return random id in top half of array
	return m_parentPercentage*getPopulationSize()*
		float(rand())/RAND_MAX;
}

/**
 * Selected population reproduces
 **/
void GeneticAlgorithm::reproduce() {
	// sort parents by fitness
	bubbleSort();

	// if first generation or fitness better than best, set to
	// best
	if ((getGeneration() == 0) ||
	    (getFitness(0) > getBestFitness())) {
		//printf("setting best\n");
		setBestGenome(getGenome(0));
		setBestFitness(getFitness(0));
	}
	//printf("sorted\n");
	//printf("reproducing\n");
	//printPopulation();
	// draw parents for each child
	for (int childId = 0; childId < getPopulationSize(); childId++) {
		uint8_t parent1 = selectParent();
		uint8_t parent2 = selectParent();
		//printf("\nchild: %d, parents: %d %d\n",
		//   childId, parent1, parent2);
		uint16_t genome1 = getGenome(parent1);
		uint16_t genome2 = getGenome(parent2);
		uint8_t crossoverPoint = (1 + getGenomeLength())*float(rand())/RAND_MAX;
		uint16_t crossoverMask = (1 << crossoverPoint) - 1;
		//printf("crossover point: %d\n", crossoverPoint);
		//printf("mask: ");
		//printBinary(crossoverMask, getGenomeLength());
		//printf("\n");

		uint16_t childGenome =  (genome1 & crossoverMask) |
				(genome2 & (~crossoverMask) );

		//printf("parent1: ");
		//printBinary(genome1, getGenomeLength());
		//printf("\n");
		//printf("parent2: ");
		//printBinary(genome2, getGenomeLength());
		//printf("\n");
		//printf("child: ");
		//printBinary(childGenome, getGenomeLength());
		//printf("\n");
		childGenome =  mutate(childGenome);
		setNextGenome(childId, childGenome);
	}
	// set current genome to next genome
	for (int id=0; id < getPopulationSize(); id++) {
		setGenome(id, getNextGenome(id));
	}
	incGeneration();
}

/*
 * Mutate a genome.
 **/
uint16_t GeneticAlgorithm::mutate(uint16_t genome) {
	for (int bit=0; bit<getGenomeLength(); bit++) {
		float mutate_prob = 0.2;
		if (float(rand())/RAND_MAX < mutate_prob) {
			//printf("mutating\n");
			//printBinary(genome, getGenomeLength());
			//printf(" ");
			genome ^= (1 << bit);
			//printBinary(genome, getGenomeLength());
			//printf("\n");
			
			// random mutants	
			//uint32_t maxGenome = 1 << getGenomeLength();
			//genome = maxGenome*float(rand())/RAND_MAX;
		}
	}
	return genome;
}

/**
 * Get gene value as integer.
 **/
uint16_t GeneticAlgorithm::getGene(uint16_t genome, uint8_t gene) {
	// get the raw integer of the gene
	return (getGeneMask(gene) & genome) >> getGenePosition(gene);
}

/**
 * Convert gene to float 0-1
 **/
float GeneticAlgorithm::getValue(uint16_t genome, uint8_t gene) {
	//printf("\n, binary: ");
	//printBinary(genome, getGenomeLength());
	//printf(", gene: %d, n: %d\n", gene, getGeneLength(gene));
	return float(getGene(genome, gene)) / ((1 << getGeneLength(gene)) - 1);
}

/**
 * This moves to the next id, and sets the last fitness value.
 **/
bool GeneticAlgorithm::testNext(float lastFitness) {
	// user tells us how last test performed
	setFitness(getCurrentId(), lastFitness);
	//printf("Current ID: %d,", getCurrentId());
	//printf("Genome: %d\n", getGenome(getCurrentId()));
	setVisitedFitness(getGenome(getCurrentId()), lastFitness);
	bool isNewGeneration = false;
	uint8_t loopCounts = 0;
	uint8_t loopReset = 10;
	while(true) {
		// now move to next test
		uint8_t nextId = getCurrentId() + 1;
		//printf("nextId %d\n", nextId);
		// if already went through all of pop,
		// do a new generation
		if (nextId >= getPopulationSize()) {
			if (getGeneration() == 0) {
				//printf("initial population\n");
				//printPopulation();
			}
			reproduce();
			nextId = 0;
			// tell user there was a new generation
			isNewGeneration = true;
		}
		setCurrentId(nextId);
		/*printf("XXX id: %d, genome: %d fitness: %f\n",
		 	getCurrentId(), getGenome(getCurrentId()),
		 	lastFitness);*/
		float newFitness = getVisitedFitness(getGenome(getCurrentId()));
		if (newFitness > 0) {
			//printf("returning for test\n");
			break;
		} else {
			//printf("using cache\n");
		}
		if (loopCounts++ > loopReset) {
			//printf("retesting\n");
			break;
		}
	}
	return isNewGeneration;
}

// print methods

void GeneticAlgorithm::printBinary(uint16_t genome, uint8_t n) {
	for (int i=n-1; i>=0; i--) {
		printf("%d", ((1 << i) & genome) >> i);
	}
}

void GeneticAlgorithm::printId(uint8_t id) {
	printf("id: %d, fitness: %f, ", id, getFitness(id));
	printGenome(getGenome(id));
}

void GeneticAlgorithm::printGenome(uint16_t genome) {
	printf("binary: ");
	printBinary(genome, getGenomeLength());
	printf(" values:");
	for (int gene=0; gene < getNumberGenes(); gene++) {
		printf(" %f", getValue(genome, gene));
	}
	printf("\n");
}

void GeneticAlgorithm::printPopulation() {
	printf("population\n");
	for (int id=0; id < getPopulationSize(); id++) {
		printId(id);
	}
}

void GeneticAlgorithm::printGeneLengths() {
	printf("gene length\n");
	for (int gene=0; gene < getNumberGenes(); gene++) {
		printf("%d\n", getGeneLength(gene));
	}
}

void GeneticAlgorithm::printGenePositions() {
	printf("gene positions\n");
	for (int gene=0; gene < getNumberGenes(); gene++) {
		printf("%d\n", getGenePosition(gene));
	}

}
