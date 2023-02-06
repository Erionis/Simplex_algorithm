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

    lcs.add_constrain({ 1, -4 }, 8, LinearConstrainSystem<double>::ConstrainType::LE);
    lcs.add_constrain({ -1, 1 }, 6, LinearConstrainSystem<double>::ConstrainType::LE);
    lcs.add_constrain({ -3, 2 }, 5, LinearConstrainSystem<double>::ConstrainType::LE);

    // vector for objective function coefficients
    std::vector<double> c = { 2, 5 };

    // vector for optimal solution
    std::vector<double> solution;

    // executing optimization of constrain system with objective function c*x
    LinearConstrainSystem<double>::SolutionType result = lcs.optimize(solution, c, LinearConstrainSystem<double>::OptimizationType::MAX);

    return 0;
}