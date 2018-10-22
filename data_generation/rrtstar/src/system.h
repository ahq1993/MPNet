/*! 
 * \file system.h 
 *
 * This serves as a template to start a new system file. 
 * It should not be included as is. 
 */ 

#ifndef __RRTS_SYSTEM_H_
#define __RRTS_SYSTEM_H_

#include  <list>



/*!
 * \brief State Class.
 *
 * A more elaborate description of the State class
 */
class State {
    
public:
    
    
    /*!
     * \brief State assingment operator.
     *
     * A more elaborate description of the State assignment operator
     */
    State& operator= (const State &stateIn);
    
    /*!
     * \brief State bracket operator.
     *
     * A more elaborate description of the State bracket operator
     */
    double& operator[] (const int i);
};


/*!
 * \brief Trajectory Class.
 *
 * A more elaborate description of the Trajectory class
 */
class Trajectory {
    
public:
    
    
    /*!
     * \brief Trajecotory assignment operator.
     *
     * A more elaborate description.
     */
    Trajectory& operator= (const Trajectory &trajectoryIn);
    
    /*!
     * \brief Returns a reference to the end state of this trajectory.
     *
     * A more elaborate description.
     */
    State& getEndState ();
    
    /*!
     * \brief Returns a reference to the end state of this trajectory (constant).
     *
     * A more elaborate description.
     */
    State& getEndState () const;
    
    
    /*!
     * \brief Returns the cost of this trajectory.
     *
     * A more elaborate description.
     */
    double evaluateCost ();
};


/*!
 * \brief System Class.
 *
 * A more elaborate description of the System class
 */
class System {
    
public:
    
    /*!
     * \brief Returns the dimensionality of the Euclidean space.
     *
     * A more elaborate description.
     */
    int getNumDimensions ();
    
    /*!
     * \brief Returns a reference to the root state.
     *
     * A more elaborate description.
     */
    State & getRootState ();
    
    /*!
     * \brief Returns the statekey for the given state.
     *
     * A more elaborate description.
     *
     * \param stateIn the given state
     * \param stateKey the key to the state. An array of dimension getNumDimensions()
     *
     */
    int getStateKey (State& stateIn, double* stateKey);
    
    /*!
     * \brief Returns true of the given state reaches the target.
     *
     * A more elaborate description.
     */
    bool isReachingTarget (State& stateIn);
    
    /*!
     * \brief Returns a sample state.
     *
     * A more elaborate description.
     *
     * \param randomStateOut
     *
     */
    int sampleState (State& randomStateOut, double (&node)[2],double px, double py);
    
    
    /*!
     * \brief Returns a the cost of the trajectory that connects stateFromIn and
     *        stateTowardsIn. The trajectory is also returned in trajectoryOut.
     *
     * A more elaborate description.
     * 
     * \param stateFromIn Initial state
     * \param stateTowardsIn Final state
     * \param trajectoryOut Trajectory that starts the from the initial state and 
     *                      reaches near the final state.
     * \param exactConnectionOut Set to true if the initial and the final states
     *                           can be connected exactly.
     *
     */
    int extendTo (State& stateFromIn, State& stateTowardsIn, 
                          Trajectory& trajectoryOut, bool& exactConnectionOut);
    
    /*!
     * \brief Returns the cost of the trajectory that connects stateFromIn and StateTowardsIn.
     *
     * A more elaborate description.
     *
     * \param stateFromIn Initial state
     * \param stateTowardsIn Final state
     * \param exactConnectionOut Set to true if the initial and the final states
     *                           can be connected exactly.
     *
     */
    double evaluateExtensionCost (State& stateFromIn, State& stateTowardsIn, bool& exactConnectionOut);
    
    /*!
     * \brief Returns a lower bound on the cost to go starting from stateIn
     *
     * A more elaborate description.
     *
     * \param stateIn Starting state
     *
     */
    double evaluateCostToGo (State& stateIn);
    
    /*!
     * \brief Returns the trajectory as a list of double arrays, each with dimension getNumDimensions.
     *
     * A more elaborate description.
     *
     * \param stateFromIn Initial state
     * \param stateToIn Final state
     * \param trajectoryOut The list of double arrays that represent the trajectory
     *
     */
    int getTrajectory (State& stateFromIn, State& stateToIn, std::list< double* > & trajectoryOut);

	// p-rrt*
    int get_pfstates (State& state);
};

#endif
