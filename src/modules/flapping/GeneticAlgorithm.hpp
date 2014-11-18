#ifndef GENETIC_ALGORITHM_HPP__
#define GENETIC_ALGORITHM_HPP__

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
		m_fitnessArray[m_genomeTestId] = fitness;
	}

	/**
	 * Increment test
	 */
	void nextTest() {
		printf("T ");
		printId(m_genomeTestId);
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
	}

	/**
	 * Print the entire population
	 */
	void printPopulation() {
		printf("***************************************************\n");
		printf("\nG: %d\n", m_generation);
		printf("BEST TRANS-GENERATION\n");
		m_bestGenome.print();
		printf(" (%4.2f)\n", double(m_bestFitness));
		printf("***************************************************\n");
		for (id_t genomeId=0;
		     genomeId < m_populationSize;
		     genomeId++) {
			printf("P ");
			printId(genomeId);
		}
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
		for (id_t genomeId=0;
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
		printf(", (%4.2f)", double(m_fitnessArray[genomeId]));
		printf("\n");
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

#endif GENETIC_ALGORITHM_HPP__
