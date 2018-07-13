/*
 * LQR.cpp
 *
 *  Created: 5 Jun 2018
 *  Author: Sebastijan Veselic
 *  Last edit: 12.6
 *  Last comment: 12.6.
 */


#ifndef _GOLEM_DATA_iLQR_iLQR_H_
#define _GOLEM_DATA_iLQR_iLQR_H_
#pragma once
/*
//------------------------------------------------------------------------------

#include <Golem/Plugin/Library.h>
#include <Golem/Plugin/Renderer.h>
#include <Golem/App/GraphRenderer.h>
#include <Golem/Planner/Profile.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Plugin/Ctrl.h>
#include <Golem/Plugin/DataI.h>
#include <Golem/Plugin/PlannerI.h>
*/

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <fstream>
#include <SFML/Graphics.hpp>

/*


//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {
namespace data {


//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR LQR:
public:

*/

/* SIZE DESCRIPTION
 *
 * A: R^n.n
 * B: R^n.m
 * Q: R^n.n
 * R: R^m.m
 * K: R^m.n
 *
 * N: 3, M: 2 We will use this for now (05.06)
 */

class Containers
{
	public:
		enum Constants

		{
			/** Don't need a zero for now, may become useful later */
			PLACEHOLDER,
			/** For vector initialisation  */
			SINGLE,
			/** For the N parameter */
			PARAM_M,
			/** For the M parameter */
			PARAM_N,
			/** Number of main typedefs, used in util.cpp */
			MATRIX_NUM = 7,
			/** Number of points in trajectory */
			TRAJECTORY_SIZE = 15,
		};

	public:

		typedef Eigen::Matrix<double, PARAM_N, PARAM_M> ControlMatrix; /** *B*  Effect of input control over the system */
		typedef Eigen::Matrix<double, PARAM_M, PARAM_N> ControlMatrixTranspose; /** *BT* Transpose of effect of input control over system */
		typedef Eigen::Matrix<double, PARAM_M, PARAM_N> FeedbackMatrix; /** *K* Feedback control matrix */
		typedef Eigen::Matrix<double, PARAM_N, PARAM_N> StateCostMatrix; /** *Q* State cost matrix. Initialised as an identity matrix and transformed */
		typedef Eigen::Matrix<double, PARAM_M, PARAM_M> InputCostMatrix; /** *R* Input cost matrix. Initialised as an identity matrix  and transformed */
		typedef Eigen::Matrix<double, PARAM_N, PARAM_N> PMatrix; /** *P* Result of computing the Discrete algebraic Riccatti equation */
		typedef Eigen::Matrix<double, PARAM_N, PARAM_N> TransitionMatrix; /** *A* Transition dynamics of the system */
		typedef Eigen::Matrix<double, PARAM_N, SINGLE> StateVector; /** *x* States of our SS model */
		typedef Eigen::Matrix<double, PARAM_M, SINGLE> InputVector; /** *u* Input control of our SS model */

		/* x and u are further divided into:
		 * xDot, yDot - solution of the SS model equation (xDot = A*x + B*u)
		 * xNaught, uNaught - feasible trajectory (this is related to xG and uG where the latter two are essentially a specific case of a feasible traj?
		 * xBar, uBar - parametrised over time (x - xNaught) and (u - uNaught)
		 */


		typedef std::map<std::string, StateVector> WaypMap;
		typedef std::map<std::string, InputVector> InputMap;
		typedef std::map<std::string, double> NearNeighMap;

		/** Wayp corresponds to numbers represented as strings (e.g. waypoint "5"), Traj corresponds to names of trajectories (e.g. "A") */
		typedef std::vector<int> WaypSeq;
		typedef std::vector<std::string> TrajSeq;
		typedef std::vector<std::string> WaypSeqHist;
		typedef std::vector<InputVector> InputHistory;
		typedef std::vector<StateVector> SystemHistory;

		/** TrajPoint can be a std::string where we would have each trajectory in a given scene
		 * marked as a name (e.g. A, B, C) after that, we would add the current waypoint in the
		 * name (e.g. 1 2 3 ...). This would give us information on the trajectory and current waypoint
		 * std::string CurrentWaypoint = std::to_string(400)
		 * std::string CurrentTrajectory = "B";
		 *
		 *This gives us the FeedbackID. Each trajectory name and waypoint is associated with a specific K matrix.
		 * std::string FeedbackID = CurrentTrajectory + CurrentWaypoint;
		 *
		 */


	/** Inititalise trajectory */
		struct WaypSelect
			{ // We will use VectorXd to avoid issues of hardcoding if there will be more states used inside.
			Eigen::Matrix<double, PARAM_M, SINGLE> Inputs; /** corresponding to xDot, yDot */
			Eigen::Matrix<double, PARAM_N, SINGLE> States; /** corresponding to x, y, theta */
			};

		struct System
			{
			Eigen::Matrix<double, PARAM_M, SINGLE> InputBar;
			Eigen::Matrix<double, PARAM_N, SINGLE> StateBar;
			};


		struct SystemPrediction
			{
			 Eigen::Matrix<double, PARAM_M, SINGLE> InputHat;
			 Eigen::Matrix<double, PARAM_N, SINGLE> StateHat;
			};

	/** Create maps */

	typedef std::map<std::string, FeedbackMatrix> KMap;
	typedef std::map<std::string, InputVector> uStarMap;
	typedef std::map<std::string, StateVector> xDotMap;

	void CreateTrajectory (const int &TrajectoryCreator, struct WaypSelect Waypoints[Containers::TRAJECTORY_SIZE]);

	const typedef double WIDTH;
	const typedef double HEIGHT;

	/** We obtain window data that is used a constant and normalisation factor */
	const typedef double NORM_CONST;

};

class iLQR

{
	public:

	Containers::PMatrix SolveDARE(
		    const Eigen::Ref<const Eigen::MatrixXd>& A,
		    const Eigen::Ref<const Eigen::MatrixXd>& B,
		    const Eigen::Ref<const Eigen::MatrixXd>& Q,
		    const Eigen::Ref<const Eigen::MatrixXd>& R);

	Containers::PMatrix SolveCARE(
		    const Eigen::Ref<const Eigen::MatrixXd>& A,
		    const Eigen::Ref<const Eigen::MatrixXd>& B,
		    const Eigen::Ref<const Eigen::MatrixXd>& Q,
			const Eigen::Ref<const Eigen::MatrixXd>& R);

	Eigen::MatrixXd old_SolveDARE(
		    const Containers::TransitionMatrix& A,
		    const Containers::ControlMatrix& B,
		    const Containers::StateCostMatrix& Q,
		    const Containers::InputCostMatrix& R);

	void OptimalControl (const Containers::StateVector &x, const Containers::InputVector &uNaught, const Containers::FeedbackMatrix &K, Containers::InputVector &uStar);

	void ReferenceChange (Containers::InputVector &uBar,  Containers::StateVector &xBar,
						  const Containers::InputVector &uNaught, const Containers::StateVector &xNaught,
						  const Containers::InputVector &u, const  Containers::StateVector &x);



	double CostFunction (const Containers::StateVector &xBar, const Containers::StateCostMatrix &Q,
						 const Containers::StateCostMatrix &Qt, const Containers::InputVector &uBar, const Containers::StateVector &xBarGoal,
						 const Containers::InputCostMatrix &Rt,  double TrajPoint, double GoalState, int SumVar, double SummedCost);

	void ComputeK (const Containers::InputCostMatrix &Rt, const Containers::ControlMatrix &B,
				   const Containers::PMatrix &P, Containers::FeedbackMatrix &K,
				   const Containers::TransitionMatrix &A, const int Discrete);

	Containers::StateCostMatrix UpdateQ (const Containers::StateCostMatrix &Q, double TrajPoint);
	Containers::InputCostMatrix UpdateR (const Containers::InputCostMatrix &R, double TrajPoint);

    void Prediction (const Containers::TransitionMatrix &A, const Containers::ControlMatrix &B,
											 	    const Containers::InputVector &uUser, const Containers::StateVector &x,
												    const Containers::StateVector &TrajectoryState, const Containers::InputVector &TrajectoryInput,
													struct Containers::SystemPrediction &Points);

    void PredictionCostFunc(const Containers::StateVector &xHat, const Containers::InputVector &uHat,
    					    const Containers::InputCostMatrix &Rt, const Containers::StateCostMatrix &Qt,
						    double &PredictionCost);

};


/* This class should ideally be in its own header file */
/** Class with utility functions that might become more numerous later on.
 * Now there's just a function that checks whether the matrix size is compatible with the matrix that's written as an argument
 * MatrixInput represents one of the possible matrices that are used in LQR, MatrixType is one of the names given to the matrices
 * that is found within the Containers class.
 */

class Util
{
	public:

	std::string SizeCheck(const Eigen::MatrixXd &MatrixInput, const std::string &MatrixType);

	/* Helper functions that are called individually within the relevant cpp files and return a constant
	 * when we check for matrix sizes. This might be the most efficient way of doing it?
	 */
    const std::string& ConMat() {static std::string conmat("ControlMatrix"); return conmat;} /** B */
    const std::string& ConMatT() {static std::string conmatT("ControlMatrixTranspose"); return conmatT;} /** BT */
    const std::string& FedMat() {static std::string fedmat("FeedbackMatrix"); return fedmat;} /** K */
    const std::string& StCoMat() {static std::string stcomat("StateCostMatrix"); return stcomat;} /** Q */
    const std::string& InpCoMat() {static std::string inpcomat("InputCostMatrix"); return inpcomat;} /** R */
    const std::string& PMat() {static std::string pmat("PMatrix"); return pmat;} /** P */
    const std::string& TranMat() {static std::string tranmat("TransitionMatrix"); return tranmat;} /** A */
    const std::string& InpVec() {static std::string inpvec("InputVector"); return inpvec;} /** u */
    const std::string& StVec() {static std::string stvec("StateVector"); return stvec;} /** x */

};

class FileParser
{
public:

	void ReadFile(std::ifstream& TrajFile, Containers::WaypMap& CompleteWaypoints, Containers::InputMap& CompleteInputs);
	void GetKeys(const Containers::WaypMap& CompleteWaypoints, Containers::WaypSeq& WaypContainer, Containers::TrajSeq& TrajContainer);
};

class WaypointCompute
{
public:

	void PrepareContainers(Containers::WaypSeq& WaypContainer, Containers::TrajSeq& TrajContainer);
	std::string HistorySeqCheck(Containers::WaypSeqHist& UserHistory, const Containers::NearNeighMap& PotentialCandidates);
	std::string NearestWaypoint(const Containers::StateVector& x,
								Containers::WaypMap& CompleteWaypoints,
								const Containers::WaypSeq& WaypContainer,
								const Containers::TrajSeq& TrajContainer,
								Containers::WaypSeqHist& UserHistory,
								std::string currwayp);

};

class InputSFML
{
public:

	Containers::InputVector UserInput(Containers::WIDTH Width, Containers::HEIGHT Height, Containers::InputVector &uUser, Containers::NORM_CONST NormFactor);
};

//------------------------------------------------------------------------------

#endif _GOLEM_DATA_iLQR_iLQR_H_

