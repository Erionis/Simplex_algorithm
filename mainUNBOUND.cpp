
#include <algorithm>
#include <iostream>
#include <vector>


#include "LinearConstrainSystem.hpp"
#include "Tableau.hpp"


int main() {
    // creo una nuova istanza di LinearConstrainSystem<double>
    LinearConstrainSystem<double> lcs;

    // aggiungo i seguenti vincoli al sistema:

    lcs.add_constrain({ 1, -4 }, 8, LinearConstrainSystem<double>::ConstrainType::LE);
    lcs.add_constrain({ -1, 1 }, 6, LinearConstrainSystem<double>::ConstrainType::LE);
    lcs.add_constrain({ -3, 2 }, 5, LinearConstrainSystem<double>::ConstrainType::LE);


    lcs.is_feasible();

    // vettore di coefficienti della funzione obiettivo
    std::vector<double> c = { 2, 5 };

    // vettore per la soluzione ottima
    std::vector<double> solution;

    // esegue l'ottimizzazione del sistema di vincoli con la funzione obiettivo c*x
    LinearConstrainSystem<double>::SolutionType result = lcs.optimize(solution, c, LinearConstrainSystem<double>::OptimizationType::MAX);



    return 0;
}