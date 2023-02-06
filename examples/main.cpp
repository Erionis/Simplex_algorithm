// #define PRINT

#include <algorithm>
#include <iostream>
#include <vector>

#include "../include/LinearConstrainSystem.hpp"
#include "../include/Tableau.hpp"



int main() {

    // creating a new instance of LinearConstrainSystem<double>
    LinearConstrainSystem<double> lcs;

    // adding following constrains to system:
    lcs.add_constrain({ 1, 0, 1}, 5, LinearConstrainSystem<double>::ConstrainType::EQ);
    lcs.add_constrain({ 0, 1, 1}, 10, LinearConstrainSystem<double>::ConstrainType::GE);
    lcs.add_constrain({ 1, 1, 0}, 20, LinearConstrainSystem<double>::ConstrainType::LE);
    
    // checking if the system is feasible
    lcs.is_feasible();

    // vector for objective function coefficients
    std::vector<double> c = { 1, -1, 3 };

    // vector for optimal solution
    std::vector<double> solution;

    // executing optimization of constrain system with objective function c*x
    LinearConstrainSystem<double>::SolutionType result = lcs.optimize(solution, c, LinearConstrainSystem<double>::OptimizationType::MAX);

    return 0;
}