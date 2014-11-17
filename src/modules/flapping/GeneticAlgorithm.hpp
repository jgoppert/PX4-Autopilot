#ifndef GENETIC_ALGORITHM_HPP__
#define GENETIC_ALGORITHM_HPP__

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

#include <inttypes.h>

class GeneticAlgorithm {
private:
	// critial parameters
	static const uint8_t m_populationSize = 5;
	static const uint8_t m_genomeLength = 8;
	static const uint8_t m_numberGenes = 2;
	float m_parentPercentage;

	// member variables
	float m_fitnessArray[m_populationSize];
	uint16_t m_genomeArray[m_populationSize];
	uint16_t m_nextGenomeArray[m_populationSize];
	uint8_t m_geneLengths[m_numberGenes];
	uint8_t m_genePositions[m_numberGenes];
	uint16_t m_geneMasks[m_numberGenes];
	uint8_t m_currentId;
	uint16_t m_bestGenome;
	float m_bestFitness;
	uint32_t m_generation;
	float visitedFitness[1 << m_genomeLength];
public:
	// constructor
	GeneticAlgorithm();

	// main methods
	void swapId(uint8_t id1, uint8_t id2);
	void bubbleSort();
	uint8_t selectParent();
	void reproduce();
	uint16_t mutate(uint16_t genome); 
	bool testNext(float lastFitness);

	// print methods
	void printBinary(uint16_t genome, uint8_t n);
	void printGenome(uint16_t genome);
	void printId(uint8_t id);
	void printPopulation();
	void printGeneLengths();
	void printGenePositions();

	// accessors
	uint16_t getGene(uint16_t genome, uint8_t gene); 
	float getValue(uint16_t genome, uint8_t gene); 
	uint8_t getPopulationSize() { return m_populationSize; }
	uint8_t getGenomeLength() { return m_genomeLength; }
	uint8_t getNumberGenes() { return m_numberGenes; }
	uint8_t getGeneMask(uint8_t id) { return m_geneMasks[id]; }
	uint8_t getGeneLength(uint8_t id) { return m_geneLengths[id]; }
	uint8_t getGenePosition(uint8_t id) { return m_genePositions[id]; }
	uint8_t getCurrentId() { return m_currentId; }
	float getFitness(uint8_t id) { return m_fitnessArray[id]; }
	uint16_t getGenome(uint8_t id) { return m_genomeArray[id]; }
	uint16_t getNextGenome(uint8_t id) {
		return m_nextGenomeArray[id];
	}
	float getBestFitness() { return m_bestFitness; }
	uint16_t getBestGenome() { return m_bestGenome; }
	uint32_t getGeneration() { return m_generation; }
	float getVisitedFitness(uint16_t genome) { return visitedFitness[genome]; }

	// setters
	void setCurrentId( uint8_t id ) { m_currentId = id; }
	void setFitness(uint8_t id, float fitness) {
		m_fitnessArray[id] = fitness;
	}
	void setGenome(uint8_t id, uint16_t genome) {
		m_genomeArray[id] = genome;
	}
	void setNextGenome(uint8_t id, uint16_t genome) {
		m_nextGenomeArray[id] = genome;
	}
	void setBestFitness(float fitness) { m_bestFitness = fitness; }
	void setBestGenome(uint16_t genome) { m_bestGenome = genome; }
	void setGeneration(uint32_t generation) {
		m_generation = generation; }
	void incGeneration() { ++m_generation; }
	void setVisitedFitness(uint16_t genome, float fitness) { visitedFitness[genome] = fitness; }
};

#endif /// GENETIC_ALGORITHM_HPP__
