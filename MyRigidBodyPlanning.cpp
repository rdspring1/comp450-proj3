#include <iostream>
#include <fstream>
#include <cmath>

#include <boost/bind.hpp>

// Including SimpleSetup.h will pull in MOST of what you need to plan
#include <ompl/geometric/SimpleSetup.h>
// Except for the state space definitions and any planners
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/base/spaces/SO2StateSpace.h>
#include "randomtree.h"

// This is our state validity checker.  It says every state is valid.
bool stateAlwaysValid(const ompl::base::State* /*state*/)
{
    return true;
}

// TODO
bool stateValidR2(const ompl::base::State* state, double minB, double maxB, int environment, int robot)
{
    const ompl::base::RealVectorStateSpace::StateType* r2state;
    r2state = state->as<ompl::base::RealVectorStateSpace::StateType>();

    double x = r2state->values[0];
    double y = r2state->values[1];

    if (x >= -0.5 && x <= 0.5 && y >= -0.5 && y <= 0.5) return false;

    return true;
}

// TODO
bool stateValidSE2(const ompl::base::State* state, double minB, double maxB, int environment, int robot)
{
    const ompl::base::CompoundState* cstate;
    cstate = state->as<ompl::base::CompoundState>();
    const ompl::base::RealVectorStateSpace::StateType* r2state;
    r2state = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
    const ompl::base::SO2StateSpace::StateType* so2state;
    so2state = cstate->as<ompl::base::SO2StateSpace::StateType>(1);

    double x = r2state->values[0];
    double y = r2state->values[1];
    double theta = so2state->value;    

    double x1 = x + 0.5*cos(theta);
    double y1 = y + 0.5*sin(theta);
    double x2 = x - 0.5*cos(theta);
    double y2 = y - 0.5*sin(theta);

    // check if line seg intersects all 4 sides of obstacle
    // check if new end points are still within bounds
    double x3 = -0.5; double y3 = -0.5;

    return true;
}


void planWithSimpleSetupR2(int environment, int robot)
{
    // Step 1) Create the state (configuration) space for your system
    // For a robot that can translate in the plane, we can use R^2 directly
    // We also need to set bounds on R^2
    ompl::base::StateSpacePtr r2(new ompl::base::RealVectorStateSpace(2));

    // We need to set bounds on R^2
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-2); // x and y have a minimum of -2
    bounds.setHigh(2); // x and y have a maximum of 2

    // Cast the r2 pointer to the derived type, then set the bounds
    r2->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    // Step 2) Create the SimpleSetup container for the motion planning problem.
    // this container ensures that everything is initialized properly before
    // trying to solve the motion planning problem.
    // ALWAYS USE SIMPLE SETUP!  There is no loss of functionality when using
    // this class.
    ompl::geometric::SimpleSetup ss(r2);

    // Step 3) Setup the StateValidityChecker
    // This is a function that takes a state and returns whether the state is a
    // valid configuration of the system or not.  For geometric planning, this
    // is a collision checker
    ss.setStateValidityChecker(stateAlwaysValid);
    //ss.setStateValidityChecker(boost::bind(stateValidR2, _1, -1, 1, environment, robot));

    // Step 4) Specify the start and goal states
    // ScopedState creates the correct state instance for the state space
    ompl::base::ScopedState<> start(r2);
    start[0] = -0.9;
    start[1] = -0.9;

    ompl::base::ScopedState<> goal(r2);
    goal[0] = 0.9;
    goal[1] = 0.9;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // Step 5) (optional) Specify a planning algorithm to use
    //ss.setPlanner();
    ompl::base::PlannerPtr planner(new ompl::geometric::randomtree(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // Step 6) Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
        ss.simplifySolution();

        // print the path to screen
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric& path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

        // print path to file
        std::ofstream fout("path.txt");
        fout << "R2" << std::endl;
        fout << environment << std::endl;
        fout << robot << std::endl;
        path.printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

void planWithSimpleSetupSE2(int environment, int robot)
{
    // Step 1) Create the state (configuration) space for your system
    // In this instance, we will plan for a unit-length line segment
    // that both translates and rotates in the plane.
    // The state space can be easily composed of simpler state spaces
    ompl::base::StateSpacePtr se2;

    ompl::base::StateSpacePtr r2(new ompl::base::RealVectorStateSpace(2));
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-2);
    bounds.setHigh(2);
    r2->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());

    se2 = r2 + so2;

    // Step 2) Create the SimpleSetup container for the motion planning problem.
    // this container ensures that everything is initialized properly before
    // trying to solve the motion planning problem using OMPL.
    // ALWAYS USE SIMPLE SETUP!  There is no loss of functionality when using
    // this class.
    ompl::geometric::SimpleSetup ss(se2);

    // Step 3) Setup the StateValidityChecker
    // This is a function that takes a state and returns whether the state is a
    // valid configuration of the system or not.  For geometric planning, this
    // is a collision checker
    ss.setStateValidityChecker(stateAlwaysValid);
    //ss.setStateValidityChecker(boost::bind(stateValidSE2, _1, -2, 2, environment, robot));

    // Step 4) Specify the start and goal states
    // ScopedState creates the correct state instance for the state space
    // You can index into the components of the state easily with ScopedState
    // The indexes correspond to the order that the StateSpace components were
    // added into the StateSpace
    ompl::base::ScopedState<> start(se2);
    start[0] = -0.9;
    start[1] = -0.9;
    start[2] = 0.0;
    ompl::base::ScopedState<> goal(se2);
    goal[0] = 0.9;
    goal[1] = 0.9;
    goal[2] = 0.0;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // Step 5) (optional) Specify a planning algorithm to use
    //ompl::base::PlannerPtr planner(new ompl::geometric::PRM(ss.getSpaceInformation()));
    //ss.setPlanner(planner);
    // #include <ompl/geometric/planners/prm/PRM.h>
    ompl::base::PlannerPtr planner(new ompl::geometric::randomtree(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // Step 6) Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
        ss.simplifySolution();

        // print the path to screen
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric& path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

        // print path to file
        std::ofstream fout("path.txt");
        fout << "SE2" << std::endl;
        fout << environment << std::endl;
        fout << robot << std::endl;
        path.printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int, char **)
{
    int choice;
    do
    {
        std::cout << "Plan for: "<< std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A circle in 2D" << std::endl;
        std::cout << " (3) A square in 2D" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 3);
 
    int environment;
    do
    {
        std::cout << "Pick Environment: "<< std::endl;
        std::cout << " (1) Go Around" << std::endl;
        std::cout << " (2) Narrow Path" << std::endl;

        std::cin >> environment;
    } while (environment < 1 || environment > 2);

    switch(choice)
    {
        case 1:
            planWithSimpleSetupR2(environment, choice);
            break;
        case 2:
            planWithSimpleSetupR2(environment, choice);
            break;
        case 3:
            planWithSimpleSetupSE2(environment, choice);
            break;
    }
    return 0;
}
