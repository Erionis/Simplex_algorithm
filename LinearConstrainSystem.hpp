#ifndef LINEARCONSTRAINSYSTEM_HPP
#define LINEARCONSTRAINSYSTEM_HPP

#include "Tableau.hpp"

/**
 * @brief Struct che rappresenta un sistema lineare di vincoli
 * 
 * @tparam T generico
 */
template<typename T>
struct LinearConstrainSystem {

    enum class SolutionType {
        BOUNDED, // trovata una soluzione ottima
        UNBOUNDED // l'insieme delle soluzioni non è limitato superiormente
    };
    enum class ConstrainType {
        EQ, // ==
        LE, // <=
        GE, // >=
    };
    enum class OptimizationType { MIN, MAX };

    struct Constrain {
        // cefficienti del vincolo
        std::vector<T> a;
        // termine noto
        T b;
        // tipo di vincolo
        ConstrainType type;
        // costruttore vuoto
        Constrain() {}
        // costruttore
        Constrain(std::vector<T> a, T b, ConstrainType type) : a(a), b(b), type(type) {}
    };

    // vettore di variabili di tipo constrain
    std::vector<Constrain> constrains;
    // crea un oggetto Tableau
    Tableau<T> tab; 

    // costruttore vuoto
    LinearConstrainSystem() {}

    // Aggiunge il vincolo a*x type b, e.g., a*x <= b
    LinearConstrainSystem& add_constrain(const std::vector<T>& a, const T& b, const ConstrainType type) {  // posso mettere inline qui??
        // aggiungo il vincolo al vettore dei vincoli
        constrains.emplace_back(a, b, type);
        return *this;
    }
    // aggiorna le informazioni riguardo al sistema di vincoli e crea il tableau iniziale
    void update();   
    // testa se il sistema di vincoli è soddisfacibile
    bool is_feasible();
    // ottimizza c*x rispetto al sistema di vincoli con x 
    SolutionType optimize(std::vector<T>& solution, std::vector<T>& c, const OptimizationType type);
    // metodo per stampare i risultati ottenuti
    void print_result(SolutionType type, std::vector<T>& solution) const;

  private:
    // metodo per aggiornare le informazioni utili al tableau
    void update_tableau_info();
    // metodo per controllare che i dati messi in input sono corretti
    void check_valid_constrains();
    // metodo per controllare che i dati messi in input sono corretti
    void check_valid_objFunc(std::vector<T>& c, const OptimizationType type);
};



/**
 * @brief metodo per aggiornare le informazioni ricevute in input nel tableau
 * 
 * @tparam T generico
 */
template<typename T>
void LinearConstrainSystem<T>::update_tableau_info(){

    // aggiorno il numero di vincoli nel tableau
    tab.num_constrains = constrains.size();
    // aggiorno il numero di varaibili decisionali nel tableau
    tab.num_variables = constrains[0].a.size(); 
    // aggiorno il numero di variabili aggiuntive del sistema
    for (const auto& constrain : constrains) {
        
        switch (constrain.type) {
            case ConstrainType::LE:
                // aggiungo una variabile di slack
                tab.slack_variables++;
                break;
            case ConstrainType::GE:
                // aggiungo una variabile di surplus e una variabile artificiale
                tab.surplus_variables++;
                tab.artificial_variables++;
                break;
            case ConstrainType::EQ:
                // aggiungo una variabile artificiale
                tab.artificial_variables++;
                break;
        }
    }
}


/**
 * @brief metodo per aggiornare e aggiungere i vincoli al tableau
 * 
 * @tparam T generico
 */
template<typename T>
void LinearConstrainSystem<T>::update() {
    // controlo i dati forniti dall'utente
    check_valid_constrains();
    // aggiorno le informazioni inserite dall'utente nel Tableau
    update_tableau_info();
    // creo il tableau iniziale
    tab.create_initial_tableau(constrains);
}


/**
 * @brief metodo per controllare se i vincoli dati in input sono accettabili
 * 
 * @tparam T 
 */
template <typename T>
void LinearConstrainSystem<T>::check_valid_constrains() {
    // guardo alla lunghezza del primo vincolo come elemnto di paragone
    unsigned int expected_num_variables = constrains[0].a.size();

    for (auto const& constrain : constrains) {

        if (constrain.a.size() != expected_num_variables) {

            throw std::invalid_argument("All constrains must have the same number of variables");
        }

        if (constrain.type != ConstrainType::LE && 
            constrain.type != ConstrainType::EQ && 
            constrain.type != ConstrainType::GE) 
            {
            throw std::invalid_argument("Invalid constrain type");
        }
    }
}


/**
 * @brief metodo per controllare se i coefficienti della funzione obiettivo e il tipo di ottimizzazione sono accettabili 
 * 
 * @tparam T generico
 * @param c vettore dei coefficienti della funzione obiettivo
 * @param type tipo di ottimizzazione
 */
template <typename T>
void LinearConstrainSystem<T>::check_valid_objFunc(std::vector<T>& c, const OptimizationType type) {
    // Eccezioni:
    // Verifico che il numero di coefficienti delle variabili decisionali sia uguale al numero di variabili decisionali
    if (c.size() != tab.num_variables) {
        throw std::invalid_argument("Wrong number of variables in objective function");
    }
    // Verifico che il tipo di ottimizzazione sia corretto
    if (type != OptimizationType::MAX && type != OptimizationType::MIN) {
        throw std::invalid_argument("Invalid optimization type");
    }
}

/*
/// @brief metodo per valutare se il sistema di vincoli è Infeasible
/// @tparam T generico
/// @return vero o falso
template <typename T>
bool LinearConstrainSystem<T>::is_feasible() {

    // creo una copia del tableau costruito finora
    Tableau<T> tab_copy(tab);

    // 1. Crea una nuova variabile di decisione chiamata "dummy"
    DecisionVariable<T> dummy;
    dummy.name = "dummy";
    decision_variables.push_back(dummy);

    // 2. Aggiungi una nuova riga al tableau per la variabile dummy
    ConstrainRow<T> dummy_row;
    dummy_row.coefficients.resize(decision_variables.size(), 0);
    dummy_row.coefficients[decision_variables.size()-1] = 1;
    dummy_row.relation = ConstrainRelation::EQ;
    dummy_row.value = 0;
    tableau.push_back(dummy_row);

    // 3. Modifica la funzione obiettivo per includere la variabile dummy
    objective_function.coefficients.resize(decision_variables.size(), 0);
    objective_function.coefficients[decision_variables.size()-1] = 1;

    // 4. Lancia il metodo optimize() sulla funzione obiettivo modificata
    OptimizationType opt_type = OptimizationType::MINIMIZE;
    std::vector<T> variable_values;
    std::vector<T> objective_values;
    SolutionType solution = optimize(variable_values, objective_values, opt_type);

    // 5. Se la soluzione ottenuta ha un valore zero per la variabile dummy, il sistema è feasible
    if (variable_values[variable_values.size()-1] == 0) {
        return true;
    } else {
        return false;
    }
}
*/


/**
 * @brief metodo che otimizza c*x applicando "pivot" al Tableau
 * 
 * @tparam T generico
 * @param solution vettore che conterrà la soluzione
 * @param c vettore dei coefficienti della funzione obiettivo
 * @param type tipo di ottimizzazione
 * @return LinearConstrainSystem<T>::SolutionType 
 */
template<typename T>
typename LinearConstrainSystem<T>::SolutionType LinearConstrainSystem<T>::optimize(std::vector<T>& solution,   std::vector<T>& c, const OptimizationType type) {
    // constrollo che i dati inseriti in input siano corretti
    check_valid_objFunc(c, type);
    // creo una copia del tableau costruito finora
    Tableau<T> tab_copy(tab);
    // aggiungo la riga della funzione obiettivo al Tableau
    tab_copy.add_objFunc_tableau(c, type);

    // FASE DELL'ALGORITMO DEL SIMPLESSO:

    // Eseguo il metodo pivot fintanto che non viene interrotto
    bool hasSimplexFinished = false;
    while (!hasSimplexFinished) {

        // ottengo l'indice della variabile entrante
        int pivot_column = tab_copy.find_pivot_column();
        // Se è uguale a -1 non esistono più variabili da introdurre in base e interrompo il ciclo
        if (pivot_column == -1 ) {            
            std::cout << "----End Simplex----" << std::endl;
            hasSimplexFinished = true; 
            break;  // NON RIESCO A TROVARE UN MODO DI FALRO SENZA IL BREAK!
        }

        // ottengo l'indice della variabile in uscita
        int pivot_row = tab_copy.find_pivot_row(pivot_column);
        // Se uguale a -1 allora il sistema è illimitato
        if (pivot_row == -1) {
            return SolutionType::UNBOUNDED;            
        }
        // Fase di "pivot" dell'algoritmo del simplesso
        tab_copy.pivot(pivot_row, pivot_column);
    }


    // scrivo in solution le soluzioni trovate
    solution.resize(tab_copy.num_variables); 

    for (size_t i = 0; i < tab_copy.num_variables; ++i) {
        // indice in cui si trovano le variabili decisionali
        size_t decision_variable = tab_copy.get_decVars_index() + i;
        // cerco nel vettore di base gli indici corrispondenti alle variabili decisionali
        auto index = std::find(tab_copy.base.begin(), tab_copy.base.end(), decision_variable); 

        if (index != tab_copy.base.end()) {
            // indice della riga del tableau che corrisponde alla variabile di base trovata.
            size_t row = index - tab_copy.base.begin();
            // prende il valore che sta all'ultimo posto della riga, cioè nella colonna dei termini noti, e salvo in solution
            solution[i] = tab_copy.tableau[row].back();
        }
    }
    // salvo il valore di z alla fine del vettore solution
    solution.emplace_back(tab_copy.tableau[tab_copy.num_constrains].back());

    return SolutionType::BOUNDED;
}


/**
 * @brief metodo per stampare la soluzione ottima del problema di ottimizzazione
 * 
 * @tparam T generico
 * @param type tipo di ottimizzazione
 * @param solution vettore che contenente la soluzione
 */
template<typename T>  // SISTEMARE STAMPANDO ANCHE I VINCOLI E LA F OBIETTIVO
void LinearConstrainSystem<T>::print_result(SolutionType type, std::vector<T>& solution) const {

    std::cout << std::endl;
    // Caso BOUNDED
    if (type == LinearConstrainSystem<double>::SolutionType::BOUNDED) {

        std::cout << "Final Tableau: " << std::endl;
        tab.print_tableau();
        std::cout << "Final Base: " << std::endl;   
        tab.print_base();     
        std::cout << std::endl;  
        std::cout << "Bounded solution found:" << std::endl;

        for (size_t i = 0; i < solution.size()-1; i++) {
            
            std::cout << "x" << i + 1 << " = " << solution[i] << std::endl;           
        }
        std::cout << std::endl; 

        std::cout<< "Optimal value z= "<< solution.back() << std::endl;

    // Caso UNBOUNDED
    } else {
        std::cout << "UNBOUNDED SOLUTION" << std::endl;
    }
}


#endif

