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

GeneticAlgorithm::GeneticAlgorithm() {
	// initialize gene positions and masks
	m_geneLengths[0] = 2;
	m_geneLengths[1] = 2;
	m_geneLengths[2] = 2;
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

	// random initialization of population
	uint32_t maxGenome = 1 << getGenomeLength();
	printf("max genome: %d\n", maxGenome);
	for (int id=0; id < getPopulationSize() ; id++) {
		setGenome(id, maxGenome*float(rand())/MAX_RAND);
	}
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
			if (getFitness(i) > getFitness(j+1)) {
				swapId(j, j+1);
			}
		}
	}
}

uint8_t GeneticAlgorithm::selectParent() {
	// return random id in top half of array
	return rand()*0.5f*getPopulationSize()/MAX_RAND;
}

void GeneticAlgorithm::reproduce() {
	// sort parents by fitness
	bubbleSort();
	// draw parents for each child
	for (int childId = 0; childId < getPopulationSize(); childId++) {
		uint16_t genome1 = getGenome(selectParent());
		uint16_t genome2 = getGenome(selectParent());
		uint8_t crossoverPoint = getGenomeLength()*rand()/MAX_RAND;
		uint16_t crossoverMask = (1 << crossoverPoint) - 1;
		setGenome(childId, (genome1 & crossoverMask) | (genome2 & (~crossoverMask) ));
		//TODO mutation
	}
}


void GeneticAlgorithm::printBinary(uint16_t val, uint8_t n) {
	for (int i=n-1; i>=0; i--) {
		printf("%d", ((1 << i) & val) >> i);
	}
}

void GeneticAlgorithm::printId(uint8_t id) {
	printf("%d, %d: ", id, getGenome(id));
	printBinary(getGenome(id), getGenomeLength());
	printf("\n");
}

uint16_t GeneticAlgorithm::getGene(uint16_t id, uint8_t index) {
	return getGeneMask(index) & getGenome(id);
}
