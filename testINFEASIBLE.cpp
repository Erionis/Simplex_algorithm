#include <iostream>
#include <vector>
#include <algorithm>

#include "LinearConstrainSystem.hpp"
#include "simplex_functions.hpp"
#include "print_functions.hpp"


int main() {
    // creo una nuova istanza di LinearConstrainSystem<double>
    LinearConstrainSystem<double> lcs(2,2,0);

    // aggiungo i seguenti vincoli al sistema:

    lcs.add_constrain({ 1, 1 }, 2, LinearConstrainSystem<double>::ConstrainType::LE);
    lcs.add_constrain({ 1, -3 }, 3, LinearConstrainSystem<double>::ConstrainType::GE);


    lcs.print_tableau();

    // vettore di coefficienti della funzione obiettivo
    std::vector<double> c = { -2, -1 };

    // vettore per la soluzione ottima
    std::vector<double> solution;

    // esegue l'ottimizzazione del sistema di vincoli con la funzione obiettivo c*x
    LinearConstrainSystem<double>::SolutionType result = lcs.optimize(solution, c, LinearConstrainSystem<double>::OptimizationType::MIN);

    // stampo il risultato dell'ottimizzazione
    lcs.print_result(result, solution);

    return 0;
}