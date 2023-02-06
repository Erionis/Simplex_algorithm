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
    lcs.add_constrain({ 2, 1 }, 8, LinearConstrainSystem<double>::ConstrainType::LE);
    lcs.add_constrain({ 1, 2 }, 9, LinearConstrainSystem<double>::ConstrainType::LE);
    lcs.add_constrain({ 1, 1 }, 5, LinearConstrainSystem<double>::ConstrainType::LE);
    
    // checking if the system is feasible
    lcs.is_feasible();
    
    // vector for objective function coefficients
    std::vector<double> c = { -5, -7 };

    // vector for optimal solution
    std::vector<double> solution;

    // executing optimization of constrain system with objective function c*x
    LinearConstrainSystem<double>::SolutionType result = lcs.optimize(solution, c, LinearConstrainSystem<double>::OptimizationType::MIN);

    return 0;
}