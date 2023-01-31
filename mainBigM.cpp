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
    lcs.add_constrain({ 1, 0, 1}, 5, LinearConstrainSystem<double>::ConstrainType::EQ);
    lcs.add_constrain({ 0, 1, 1}, 10, LinearConstrainSystem<double>::ConstrainType::GE);
    lcs.add_constrain({ 1, 1, 0}, 20, LinearConstrainSystem<double>::ConstrainType::LE);
    
    // controllo che il sistema sia ammissibile
    lcs.is_feasible();


    // vettore di coefficienti della funzione obiettivo
    std::vector<double> c = { 1, -1, 3 };

    // vettore per la soluzione ottima
    std::vector<double> solution;

    // eseguo l'ottimizzazione del sistema di vincoli con la funzione obiettivo c*x
    LinearConstrainSystem<double>::SolutionType result = lcs.optimize(solution, c, LinearConstrainSystem<double>::OptimizationType::MAX);


    return 0;
}
