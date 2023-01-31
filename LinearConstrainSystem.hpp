#ifndef __LINEARCONSTRAINSYSTEM_HPP__
#define __LINEARCONSTRAINSYSTEM_HPP__

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
        // coefficienti del vincolo
        std::vector<T> a;
        // termine noto
        T b;
        // tipo di vincolo
        ConstrainType type;
        // costruttore vuoto
        Constrain() {}
        // costruttore di inizializzazione
        Constrain(std::vector<T> a, T b, ConstrainType type) : a(a), b(b), type(type) {}
        // costruttore di copia
        Constrain(const Constrain& orig) : a(orig.a), b(orig.b), type(orig.type) {}
    };

    // vettore di oggetti di tipo Constrain
    std::vector<Constrain> constrains;
    // creo un oggetto di tipo Tableau
    Tableau<T> tab; 
    // flag che tiene conto se l'utente ha già constollato se il sistema è ammissibile
    bool feasibility_test{false};

    // costruttore vuoto
    LinearConstrainSystem() {}
    // costruttore di copia
    LinearConstrainSystem(const LinearConstrainSystem& orig) : constrains(orig.constrains), tab(orig.tab) {}

    // Aggiunge il vincolo a*x type b, e.g., a*x <= b
    inline LinearConstrainSystem& add_constrain(const std::vector<T>& a, const T& b, const ConstrainType type){ 
        // aggiungo il vincolo al vettore dei vincoli
        constrains.emplace_back(a, b, type);
        return *this;
    }

    // metodo per vedere se il sistema è ammissibile  // MI TORNA BOOL MA NON LO USO MAI
    bool is_feasible();
    // ottimizza c*x rispetto al sistema di vincoli con x 
    SolutionType optimize(std::vector<T>& solution, const std::vector<T>& c, const OptimizationType type);
    // metodo per stampare i risultati ottenuti
    void print_result(SolutionType type, std::vector<T>& solution) const;
    // metodo per stampare il problema di ottimizzazione ricevuto in input
    void print_Lcs(const std::vector<T>& c, const OptimizationType type) const;

  private:
    // metodo per aggiornare le informazioni utili per la costruzione del tableau
    void update_tableau_info();
    // metodo per controllare che i vincoli in input siano accettabili
    void check_valid_constrains() const;
    // metodo per controllare che la funzione obiettivo in input sia accettabile
    void check_valid_objFunc(const std::vector<T>& c, const OptimizationType type) const;
};



/**
 * @brief metodo per aggiornare le informazioni ricevute in input nel tableau
 * 
 * @tparam T generico
 */
template<typename T>
void LinearConstrainSystem<T>::update_tableau_info() {

    // aggiorno il numero di vincoli nel tableau
    tab.num_constrains = constrains.size();
    // aggiorno il numero di variabili decisionali nel tableau
    tab.num_variables = constrains[0].a.size(); 
    // aggiorno il numero di variabili aggiuntive del sistema
    for (const auto& constrain : constrains) {
        
        switch (constrain.type) {
            case ConstrainType::LE:

                if (constrain.b < 0){
                    tab.surplus_variables++;
                    tab.artificial_variables++;
                } else {
                    tab.slack_variables++;
                }

                break;
            case ConstrainType::GE:

                if (constrain.b < 0){
                    tab.slack_variables++;
                } else {
                tab.surplus_variables++;
                tab.artificial_variables++;                    
                }
                break;
            case ConstrainType::EQ:
                // aggiungo una variabile artificiale
                tab.artificial_variables++;
                break;
        }
    }
}


/**
 * @brief metodo per verificare se i vincoli dati in input sono accettabili
 * 
 * @tparam T generico
 */
template <typename T>
void LinearConstrainSystem<T>::check_valid_constrains() const {
    // guardo alla lunghezza del primo vincolo come elemento di paragone
    unsigned int expected_num_variables = constrains[0].a.size();

    for (auto const& constrain : constrains) {

        if (constrain.a.size() != expected_num_variables) {

            throw std::invalid_argument("All constrains must have the same number of variables");
        }

        if (constrain.type != ConstrainType::LE && 
            constrain.type != ConstrainType::EQ &&   // da capire se serve
            constrain.type != ConstrainType::GE) 
            {
            throw std::invalid_argument("Invalid constrain type");
        }
    }
    // controllare che gianfry metta dei numeri e non altro!! 
}


/**
 * @brief metodo per controllare se i coefficienti della funzione obiettivo e il tipo di ottimizzazione sono accettabili 
 * 
 * @tparam T generico
 * @param c vettore dei coefficienti della funzione obiettivo
 * @param type tipo di ottimizzazione
 */
template <typename T>
void LinearConstrainSystem<T>::check_valid_objFunc(const std::vector<T>& c, const OptimizationType type) const {

    // Verifico che il numero di coefficienti delle variabili decisionali sia uguale al numero di variabili decisionali
    if (c.size() != tab.num_variables) {
        throw std::invalid_argument("Wrong number of variables in objective function");
    }
    // Verifico che il tipo di ottimizzazione sia corretto
    if (type != OptimizationType::MAX && type != OptimizationType::MIN) {
        throw std::invalid_argument("Invalid optimization type");
    }
}


/**
 * @brief metodo per stabilire se il sistema di vincoli è ammissibile
 * 
 * @tparam T generico
 * @return true se il sistema di vincoli è ammissibile
 * @return false se il sistema di vincoli non è ammissibile
 */
template <typename T>
bool LinearConstrainSystem<T>::is_feasible() {   /// METTI BOOLEANO

    // controllo che i valori forniti in ingresso siano accettabili
    check_valid_constrains();
    // aggiorno le informazioni ricevute finora dall'utente
    update_tableau_info();

    // Creo una copia del LinearConstrainsystem costruito finora
    LinearConstrainSystem<T> copy(*this);

    // aggiorno la dimensione dei vincoli forniti finora per fare spazio alla variabile dummy
    for (size_t i=0; i< copy.constrains.size(); ++i) { 
        copy.constrains[i].a.push_back(0);
    }
    // creo i coefficienti del vicnolo che si riferisce alla variabile dummy
    std::vector<T> a_dummy(copy.constrains[0].a.size()-1, 0);
    a_dummy.emplace_back(1);
    // aggiungo il vincolo al sistema di vincoli
    copy.add_constrain(a_dummy, 0 ,ConstrainType::EQ);
    // aggiorno le informazioni della variabile dummy nel Tableau
    copy.tab.num_constrains++;
    copy.tab.num_variables++;
    copy.tab.artificial_variables++;
    // creo il Tableau iniziale
    copy.tab.create_initial_tableau(copy.constrains);
    // creo la funzione obiettivo dummy
    std::vector<T> c(copy.tab.num_variables - 1,0);
    c.emplace_back(1);
    // la aggiungo al Tableau
    copy.tab.add_objFunc_tableau(c, LinearConstrainSystem<double>::OptimizationType::MAX);

    // Eseguo il metodo pivot del simplesso fintanto che non viene interrotto
    bool hasSimplexFinished = false;
    while (!hasSimplexFinished ) {
        // ottengo l'indice della variabile entrante
        int pivot_column = copy.tab.find_pivot_column();
        // Se è uguale a -1 non esistono più variabili da introdurre in base e interrompo il ciclo
        if (pivot_column == -1 ) {    
            #ifdef PRINT        
            std::cout << "----End Simplex----" << std::endl<<std::endl;
            #endif // PRINT
            hasSimplexFinished = true; 
        }
        if (hasSimplexFinished == false) {
            // ottengo l'indice della variabile in uscita
            int pivot_row = copy.tab.find_pivot_row(pivot_column);
            // Fase di "pivot" dell'algoritmo del simplesso
            copy.tab.pivot(pivot_row, pivot_column);
        }
    }
    #ifdef PRINT 
    std::cout<< "FEASIBILITY TEST: "<< std::endl<<std::endl;  
    #endif // PRINT

    T solution = copy.tab.tableau[copy.tab.num_constrains].back();
    // se z<0 allora il sistema non è ammissibile
    if (solution < 0) {
        throw std::runtime_error("The linear constraint system is INFEASIBLE.");
        return false;
    // altrimenti è ammissibile
    } else {
        #ifdef PRINT
        std::cout<< "The system is FEASIBLE!"<< std::endl<<std::endl;
        #endif // PRINT
        // aggiorno la flag del test di ammissibilità
        feasibility_test = true;
        return true;
    }
}


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
typename LinearConstrainSystem<T>::SolutionType LinearConstrainSystem<T>::optimize(std::vector<T>& solution, const  std::vector<T>& c, const OptimizationType type) {

    LinearConstrainSystem<T>::SolutionType sol_type; // variabile that will be returned
    // se l'utente non ha ancora eseguito isfeasible() lo eseguo
    if (feasibility_test == false) {
        is_feasible();
    }
    // constollo i dati in ingresso
    check_valid_objFunc(c, type);
    // Creo una copia del LinearConstrainSystem costruito finora
    LinearConstrainSystem<T> copy(*this);
    // creo il tableau iniziale
    copy.tab.create_initial_tableau(copy.constrains);
    // aggiungo la riga della funzione obiettivo al Tableau
    copy.tab.add_objFunc_tableau(c, type);

    // FASE DELL'ALGORITMO DEL SIMPLESSO:
    // Eseguo il metodo pivot fintanto che non viene interrotto
    bool hasSimplexFinished = false;
    while (!hasSimplexFinished) {

        // ottengo l'indice della variabile entrante
        int pivot_column = copy.tab.find_pivot_column();
        // Se è uguale a -1 non esistono più variabili da introdurre in base e interrompo il ciclo
        if (pivot_column == -1 ) {            
            #ifdef PRINT
            std::cout << "----End Simplex----" << std::endl<<std::endl;
            #endif // PRINT
            hasSimplexFinished = true; 
        }
        if (hasSimplexFinished == false) {
            // ottengo l'indice della variabile in uscita
            int pivot_row = copy.tab.find_pivot_row(pivot_column);
            // Se la riga pivot è uguale a -1 allora il sistema è illimitato
            if (pivot_row == -1) {
                sol_type = SolutionType::UNBOUNDED; // update and then return the variable
                #ifdef PRINT
                print_Lcs(c,type);
                print_result(sol_type, solution);  
                #endif // PRINT
                return sol_type;   
            }
            // Fase di "pivot" dell'algoritmo del simplesso
            copy.tab.pivot(pivot_row, pivot_column);
        }
    }
    // scrivo in solution le soluzioni trovate
    solution.resize(copy.tab.num_variables); 

    for (size_t i = 0; i < copy.tab.num_variables; ++i) {
        // indice in cui si trovano le variabili decisionali
        size_t decision_variable = copy.tab.get_decVars_index() + i;
        // cerco nel vettore di base gli indici corrispondenti alle variabili decisionali
        auto index = std::find(copy.tab.base.begin(), copy.tab.base.end(), decision_variable); 

        if (index != copy.tab.base.end()) {
            // indice della riga del tableau che corrisponde alla variabile di base trovata.
            size_t row = index - copy.tab.base.begin();
            // prende il valore che sta all'ultimo posto della riga, cioè nella colonna dei termini noti, e salvo in solution
            solution[i] = copy.tab.tableau[row].back();
        }
    }
    // salvo il valore di z alla fine del vettore solution
    solution.emplace_back(copy.tab.tableau[copy.tab.num_constrains].back());
    // stampo il probleam di ottimizzazione

    sol_type = SolutionType::BOUNDED; // update and then return the variable
    
    print_Lcs(c,type);
    print_result(sol_type, solution);  

    return sol_type;
}


/**
 * @brief  metodo per stampare il problema di ottimizzazione ricevuto in input
 * 
 * @tparam T generico
 * @param c vettore dei coefficienti della funzione obiettivo
 * @param type tipo di ottimizzazione
 */
template<typename T>
void LinearConstrainSystem<T>::print_Lcs(const std::vector<T>& c, const OptimizationType type) const {

    std::cout<< "Optimization problem: "<< std::endl<< std::endl;

    std::cout << (type == OptimizationType::MIN ? "Minimize: " : "Maximize: ") << c[0] << "x1";
    for (size_t i = 1; i < c.size(); ++i) {
        if (c[i] >= 0) {
            std::cout << " + " << c[i] << "x" << (i + 1);
        } else {
            std::cout << " - " << -c[i] << "x" << (i + 1);
        }
    }
    std::cout << std::endl << "Subject to:" << std::endl;

    for (const auto &constrain : constrains) {
        std::cout << constrain.a[0]<< "x1";
        for (size_t i = 1; i < constrain.a.size(); ++i) {
            if (constrain.a[i] >= 0) {
                std::cout << " + " << constrain.a[i] << "x" << (i + 1);
            } else {
                std::cout << " - " << -constrain.a[i] << "x" << (i + 1);
            }
        }

        switch (constrain.type) {
            case ConstrainType::LE:
                std::cout << " <= ";
                break;
            case ConstrainType::GE:
                std::cout << " >= ";
                break;
            case ConstrainType::EQ:
                std::cout << " = ";
                break;
        }
        std::cout << constrain.b << std::endl;
    }

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

    // Caso BOUNDED
    if (type == LinearConstrainSystem<double>::SolutionType::BOUNDED) {
        
        std::cout << std::endl;  
        std::cout << "Bounded solution found:" << std::endl;

        for (size_t i = 0; i < solution.size()-1; i++) {
            
            std::cout << "x" << i + 1 << " = " << solution[i] << std::endl;           
        }
        std::cout << std::endl; 

        std::cout<< "Optimal value z= "<< solution.back() << std::endl<< std::endl;

    // Caso UNBOUNDED
    } else {
        std::cout << "UNBOUNDED SOLUTION" << std::endl<< std::endl;
    }
}

#endif // __LINEARCONSTRAINSYSTEM_HPP__

