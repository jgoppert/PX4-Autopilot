#include "GeneticAlgorithm.hpp"

const uint8_t Genome::m_numberGenes = 2;
const uint8_t Genome::m_geneLengths[] = {4,4};
const uint8_t Genome::m_genomeLength = 8;
// change with geneLengths

const uint8_t Genome::m_genePositions[] = {0,4};

const float GeneticAlgorithm::m_nullFitness = -1e5;
