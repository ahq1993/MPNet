/*! 
 * \file system_single_integrator.h 
 */ 

#ifndef __RRTS_SYSTEM_SINGLE_INTEGRATOR_H_
#define __RRTS_SYSTEM_SINGLE_INTEGRATOR_H_

#include <list>



namespace SingleIntegrator {

    
    /*!
     * \brief region class
     *
     * More elaborate description
     */
    class region {
        
        int numDimensions;
        
    public:    
        
        /*!
         * \brief Cartesian coordinates of the center of the region
         *
         * More elaborate description
         */
        double *center;
        
        /*!
         * \brief Size of the region in cartesian coordinates
         *
         * More elaborate description
         */
        double *size;
        double radius;
        /*!
         * \brief region constructor
         *
         * More elaborate description
         */
        region ();
        
        /*!
         * \brief region destructor
         *
         * More elaborate description
         */
        ~region ();
        
        /*!
         * \brief Sets the dimensionality of the region
         *
         * More elaborate description
         *
         * \param numDimensionsIn New number of dimensions.
         *
         */
        int setNumDimensions (int numDimensionsIn);
    };
    

    
    /*!
     * \brief State Class.
     *
     * A more elaborate description of the State class
     */
    class State {
        
        int numDimensions;
        double *x;

        int setNumDimensions (int numDimensions);
        
    public:

        /*!
         * \brief State constructor
         *
         * More elaborate description
         */
        State ();
        
        /*!
         * \brief State desctructor
         *
         * More elaborate description
         */
        ~State ();
        
        /*!
         * \brief State copy constructor
         *
         * More elaborate description
         */
        State (const State& stateIn);
        
        /*!
         * \brief State assignment operator
         *
         * More elaborate description
         */
        State& operator= (const State& stateIn);
        
        /*!
         * \brief State bracket operator
         *
         * More elaborate description
         */
        double& operator[] (const int i) {return x[i];}
        
        friend class System;
        friend class Trajectory;
    };
    
    
    
    /*!
     * \brief Trajectory Class.
     *
     * A more elaborate description of the State class
     */
    class Trajectory {
        
        State *endState; 
        double totalVariation;  
        
    public:    

        /*!
         * \brief Trajectory constructor
         *
         * More elaborate description
         */
        Trajectory ();
        
        /*!
         * \brief Trajectory destructor
         *
         * More elaborate description
         */
        ~Trajectory ();
        
        /*!
         * \brief Trajectory copy constructor
         *
         * More elaborate description
         *
         * \param trajectoryIn The trajectory to be copied.
         *
         */
        Trajectory (const Trajectory& trajectoryIn);
        
        /*!
         * \brief Trajectory assignment constructor
         *
         * More elaborate description
         *
         * \param trajectoryIn the trajectory to be copied.
         *
         */
        Trajectory& operator= (const Trajectory& trajectoryIn);
        
        /*!
         * \brief Returns a reference to the end state of this trajectory.
         *
         * More elaborate description
         */
        State& getEndState () {return *endState;}
        
        /*!
         * \brief Returns a reference to the end state of this trajectory (constant).
         *
         * More elaborate description
         */
        State& getEndState () const {return *endState;}
        
        /*!
         * \brief Returns the cost of this trajectory.
         *
         * More elaborate description
         */
        double evaluateCost ();
        
        friend class System;
    };
    
    
    
    /*!
     * \brief System Class.
     *
     * A more elaborate description of the State class
     */
    class System {
        
        int numDimensions;
        bool IsInCollision (double *stateIn);
        
        State rootState;
        
    public:    
        
        /*!
         * \brief The operating region
         *
         * More elaborate description
         */
        region regionOperating;
        
        /*!
         * \brief The goal region
         *
         * More elaborate description
         */
        region regionGoal;
        
        /*!
         * \brief The list of all obstacles
         *
         * More elaborate description
         */
        std::list<region*> obstacles;
        
        /*!
         * \brief System constructor
         *
         * More elaborate description
         */
        System ();
        
        /*!
         * \brief System destructor
         *
         * More elaborate description
         */
        ~System ();
        
        int setNumDimensions (int numDimensionsIn);
        
        /*!
         * \brief Returns the dimensionality of the Euclidean space.
         *
         * A more elaborate description.
         */
        int getNumDimensions () {return numDimensions;}
        
        /*!
         * \brief Returns a reference to the root state.
         *
         * A more elaborate description.
         */
        State& getRootState () {return rootState;}
        
        /*!
         * \brief Returns the statekey for the given state.
         *
         * A more elaborate description.
         *
         * \param stateIn the given state
         * \param stateKey the key to the state. An array of dimension getNumDimensions()
         *
         */
        int getStateKey (State &stateIn, double *stateKey);
        
        /*!
         * \brief Returns true of the given state reaches the target.
         *
         * A more elaborate description.
         */
        bool isReachingTarget (State &stateIn);
        
        /*!
         * \brief Returns a sample state.
         *
         * A more elaborate description.
         *
         * \param randomStateOut
         *
         */
        int sampleState (State &randomStateOut, double (&node)[2], double px, double py); 
       

        
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
        int extendTo (State &stateFromIn, State &stateTowardsIn, 
                      Trajectory &trajectoryOut, bool &exactConnectionOut); 
        int steerTo (State &stateFromIn, State &stateTowardsIn);
        int optimization (double* stateA, double* stateC);
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
        double evaluateExtensionCost (State &stateFromIn, State &stateTowardsIn, bool &exactConnectionOut);
        
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
        int getTrajectory (State& stateFromIn, State& stateToIn, std::list<double*>& trajectoryOut);
        

		// p-rrt*
        int pfstates (State& state);
    };
}


#endif
