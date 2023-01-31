// #define PRINT

#include <algorithm>
#include <iostream>
#include <vector>


#include "LinearConstrainSystem.hpp"
#include "Tableau.hpp"


int main() {
    // crea una nuova istanza di LinearConstrainSystem<double>
    LinearConstrainSystem<double> lcs;

    // aggiunge i seguenti vincoli al sistema:

    // lcs.add_constrain({ 1, 1 }, 2, LinearConstrainSystem<double>::ConstrainType::EQ);
    // lcs.add_constrain({ 1, -3 }, 3, LinearConstrainSystem<double>::ConstrainType::GE);

    // lcs.add_constrain({ 2, 3 }, 6, LinearConstrainSystem<double>::ConstrainType::LE);
    // lcs.add_constrain({ 3, 5 }, 15, LinearConstrainSystem<double>::ConstrainType::GE);


    
    lcs.add_constrain({ 2, 3 }, 1200, LinearConstrainSystem<double>::ConstrainType::GE);
    lcs.add_constrain({ 1, 1 }, 400, LinearConstrainSystem<double>::ConstrainType::LE);
    lcs.add_constrain({ 2, 1.5 }, 900, LinearConstrainSystem<double>::ConstrainType::GE);

    // controllo che il sistema sia ammissibile
    lcs.is_feasible();

    // crea un vettore di coefficienti della funzione obiettivo
    std::vector<double> c = { -2, -1 };

    // crea un vettore per la soluzione ottima
    std::vector<double> solution;

    // esegue l'ottimizzazione del sistema di vincoli con la funzione obiettivo c*x
    LinearConstrainSystem<double>::SolutionType result = lcs.optimize(solution, c, LinearConstrainSystem<double>::OptimizationType::MIN);

    return 0;
}