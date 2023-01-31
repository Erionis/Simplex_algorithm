// #define PRINT

#include <algorithm>
#include <iostream>
#include <vector>

#include "LinearConstrainSystem.hpp"
#include "Tableau.hpp"

int main() {

    // creo una nuova istanza di LinearConstrainSystem<double>
    LinearConstrainSystem<double> lcs;

    // aggiungo i seguenti vincoli al sistema:

    lcs.add_constrain({ 2, -1, 3}, 4, LinearConstrainSystem<double>::ConstrainType::GE);
    lcs.add_constrain({ 1, 2, 0}, 6, LinearConstrainSystem<double>::ConstrainType::GE);
    lcs.add_constrain({ 3, -1, 2}, 7, LinearConstrainSystem<double>::ConstrainType::LE);
    lcs.add_constrain({ -1, 5, 1}, 6, LinearConstrainSystem<double>::ConstrainType::EQ);

    // controllo che il sistema sia ammissibile
    lcs.is_feasible();

    //vettore di coefficienti della funzione obiettivo
    std::vector<double> c = { 2, -3, 5 };

    //vettore per la soluzione ottima
    std::vector<double> solution;

    // eseguo l'ottimizzazione del sistema di vincoli con la funzione obiettivo c*x
    LinearConstrainSystem<double>::SolutionType result = lcs.optimize(solution, c, LinearConstrainSystem<double>::OptimizationType::MAX);

    return 0;
}
