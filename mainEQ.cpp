
#include <algorithm>
#include <iostream>
#include <vector>

#include "LinearConstrainSystem.hpp"
#include "Tableau.hpp"



int main() {
    // creo una nuova istanza di LinearConstrainSystem<double>
    LinearConstrainSystem<double> lcs;

    // aggiungo i seguenti vincoli al sistema:

    lcs.add_constrain({ 2, 0, 3 }, 1, LinearConstrainSystem<double>::ConstrainType::EQ);
    lcs.add_constrain({ 3, 2, -1 }, 5, LinearConstrainSystem<double>::ConstrainType::EQ);
    
    // controllo che il sistema sia ammissibile
    lcs.is_feasible();

    // vettore di coefficienti della funzione obiettivo
    std::vector<double> c = { 1, -2, 0 };

    // vettore per la soluzione ottima
    std::vector<double> solution;

    // esegue l'ottimizzazione del sistema di vincoli con la funzione obiettivo c*x
    LinearConstrainSystem<double>::SolutionType result = lcs.optimize(solution, c, LinearConstrainSystem<double>::OptimizationType::MIN);
    
    lcs.print_result(result, solution);


    return 0;
}