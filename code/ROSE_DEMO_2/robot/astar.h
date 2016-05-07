// Written by:	Ajay Srivastava, Srihari Chekuri
// Tested by: 	Ajay Srivastava, Srihari Chekuri

#ifndef ASTAR_H
#define ASTAR_H

#include <armadillo>
#include <vector>

#include "actions.h"

class AStar
{
	public:
		AStar(arma::mat map, arma::vec &goal);
		~AStar(void);
		void compute(arma::vec &start, std::vector<MotionAction> &path);
		bool complete(void);
		bool impossible(void);

		arma::mat map;
		arma::vec goal;

		// stuff for the decision making capability
		bool isComplete;
		bool isImpossible;
};

#endif