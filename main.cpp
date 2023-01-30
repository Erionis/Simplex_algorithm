
#include <algorithm>
#include <iostream>
#include <vector>

#include "LinearConstrainSystem.hpp"
#include "Tableau.hpp"



int main() {

    // creo una nuova istanza di LinearConstrainSystem<double>
    LinearConstrainSystem<double> lcs;

    // aggiungo i seguenti vincoli al sistema:
    lcs.add_constrain({ 1, -1, 2, -1 }, -2, LinearConstrainSystem<double>::ConstrainType::GE);
    lcs.add_constrain({ 2, 2, 1, -4 }, 4, LinearConstrainSystem<double>::ConstrainType::LE);
    lcs.add_constrain({ 1, -1, 2, 0 }, 2, LinearConstrainSystem<double>::ConstrainType::LE);

    // controllo che il sistema sia ammissibile
    lcs.is_feasible();  /// mettere booleano

    // vettore di coefficienti della funzione obiettivo
    std::vector<double> c = {  2, 3, -1, -3 };

    // vettore per la soluzione ottima
    std::vector<double> solution;

    // eseguo l'ottimizzazione del sistema di vincoli con la funzione obiettivo c*x
    LinearConstrainSystem<double>::SolutionType result = lcs.optimize(solution, c, LinearConstrainSystem<double>::OptimizationType::MAX);

    // stampo il risultato dell'ottimizzazione
    lcs.print_result(result, solution);

    return 0;
}
