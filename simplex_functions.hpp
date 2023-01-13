#ifndef __simplex_functions_hpp__
#define __simplex_functions_hpp__


/// @brief metodo per aggiungere i vincoli al Tableau
/// @tparam T generico
/// @param a vettore dei coefficienti delle variabili decisionali
/// @param b termine noto del vincolo
/// @param type Tipo di ottimizzazione
/// @return oggetto di tipo LinearConstrainSystem
template <typename T>
LinearConstrainSystem<T>& LinearConstrainSystem<T>::add_constrain(const std::vector<T>& a, const T& b, const ConstrainType type) {

    // Eccezioni:
    // Verifico che il numero di coefficienti di a sia uguale al numero di variabili decisionali
    if (a.size() != num_variables) {
        throw std::invalid_argument("Wrong number of variables in constrain");
    }
    // Verifico che il tipo di vincolo sia valido
    if (type != ConstrainType::LE && type != ConstrainType::GE && type != ConstrainType::EQ) {
        throw std::invalid_argument("Invalid type of constraint");
    }

    // creo un vettore con i coefficienti cambiati di segno di a
    std::vector<T> a_neg(a.size());
    std::transform(a.begin(), a.end(), a_neg.begin(), std::negate<T>()); 

    // numero totale di colonne che avrà il Tableau
    int tot_columns = num_constrains + num_variables + EQ_cases + 1;
              
    // creo una riga nel Tableau
    tableau.emplace_back(tot_columns, 0);

    // contatore per memorizzare in che riga siamo 
    int current_row = tableau.size() - 1;    

    switch (type) {

        // Nel caso di vincolo "LE" mi basta aggiungere una variabile artificiale con coeff 1, 
        // i coefficienti delle variabili decisionali e il termine noto, nella riga corrente
        case ConstrainType::LE: {

            add_row_tableau(a, b, current_row);
            break;
        }

        // Nel caso di vincolo "GE" mi basta aggiungere una variabile artificiale con coeff 1, 
        // i coefficienti delle variabili decisionali e il termine noto ma cambiati di segno
        case ConstrainType::GE: {

            add_row_tableau( a_neg, b*(-1), current_row);
            break;
        }

        // Nel caso di vincolo "EQ" aggiungo 2 righe al Tableau: la prima considera il vincolo come se fosse LE 
        // mentre la seconda considera il vicnolo come se fosse GE
        case ConstrainType::EQ: {

            add_row_tableau(a, b, current_row);

            tableau.emplace_back(tot_columns, 0);
            
            add_row_tableau(a_neg, b*(-1), current_row + 1);

            break;
        }
    }
    return *this;
}


/// @brief  Metodo per aggiungere una riga al Tableau contenete le informazioni del vincolo
/// @tparam T generico
/// @param a vettore dei coefficienti delle variabili decisionali
/// @param b termine noto del vincolo
/// @param current_row intero corrispondente all'indice della riga del Tableau che si sta aggiungendo
template<typename T>
void LinearConstrainSystem<T>::add_row_tableau(const std::vector<T>& a, const T& b, int current_row){

    // Il numero totale di righe che avrà il Tableau è uguale al numero totale di variabili artificiali del problema
    int tot_rows = artificial_variables;

    // per tutte le variabili artificiali        
    for (int row_index = 0; row_index < tot_rows; ++row_index) {

        // se l'indice della variabile artificiale è uguale all'indice della riga corrente
        if (row_index == current_row) {
            // assegno coefficiente 1 alla variabile artificiale
            tableau[current_row][current_row] = 1;
            // inserisco i coefficienti del vettore a e il termine noto b nella posizione corretta
            for (size_t i = 0; i < num_variables; ++i)  {

                tableau[current_row][artificial_variables + i] = a[i];
                tableau[current_row].back() = b;
            }
        } 
    }            
    // aggiorno gli elementi in base
    base.emplace_back(current_row);
}


/// @brief metodo per aggiungere la riga della funzione obiettivo al Tableau
/// @tparam T generico
/// @param c vettore dei coefficienti della funzione obiettivo
/// @param type Tipo di ottimizzazione
template<typename T>
void LinearConstrainSystem<T>::add_ObjFunc_Tableau(std::vector<T>& c, const OptimizationType type) {

    // Numero di colonne totali del Tableau
    int tot_columns = num_constrains + num_variables + EQ_cases + 1;
    // Indice di riga della funzione obiettivo
    int ObjFunc_row = artificial_variables;
    // creo una nuova riga nel Tableau            
    tableau.emplace_back(tot_columns, 0);

    switch (type) {

        // Caso di MINIMIZZAZIONE
        case OptimizationType::MIN: {
            // Aggiungo nella nuova riga i coefficienti della funzione obiettivo nella posizione corretta
            for (size_t i = 0; i < num_variables; ++i)  {

                tableau[ObjFunc_row][ObjFunc_row + i] = c[i];
            }
            break;
        }

        // Caso di MASSIMIZZAZIONE
        case OptimizationType::MAX: {
            // Aggiungo nella nuova riga i coefficienti della funzione obiettivo cambiati di segno
            for (size_t i = 0; i < num_variables; ++i)  {
                
                tableau[ObjFunc_row][ObjFunc_row + i] = c[i]*(-1);
            }
            break;
        }
    }
    // Aggiungo il termine noto uguale a 0 della funzione obiettivo
    tableau[ObjFunc_row].back()= 0;

    std::cout << "---Start Simplex---" << std::endl;
    std::cout << "Tableau iniziale: " << std::endl;
    print_tableau();
    std::cout << "Base iniziale: " << std::endl;
    print_base();
}


/// @brief metodo per valutare se il sistema di vincoli è Infeasible
/// @tparam T generico
/// @return vero o falso
template <typename T>
bool LinearConstrainSystem<T>::is_feasible() {

    // Una volta ottenuto il tableau finale, controllo se ci sono righe del tableau con coefficienti 
    // delle variabili decisionali tutti uguali a 0 e il valore nella colonna dei termini noti  positivo

    // Numero totale di righe del Tableau
    int num_articial_vars = artificial_variables;
    // per tutte le righe tranne quella della funzione obiettivo
    for (int row_index = 0; row_index < num_articial_vars; ++row_index) {

        // Recupero l'indice della variabile decisionale corrente dal vettore di base
        int variable = base[row_index];
        // le variabili artificiali, per come è stato costruito il Tableau all'inizio avranno 
        // indici minori del numero di righe del Tableau!
        // Se la variabile decisionale corrente è una variabile artificiale, ignoro la riga
        if (variable < num_articial_vars) continue;

        bool has_decision_variables = true;
        // controllo se ci sono altri coefficienti delle variabili decisionali diversi da zero nella riga corrente
        for (int j = 0; j < num_variables; ++j) {
            // se ci sono altri coefficienti delle variabili decisionali diversi da zero
            if (j != variable - num_articial_vars && tableau[row_index][num_articial_vars + j] != 0) {
                has_decision_variables = true;
                break;
            }
        }
        // Se non ci sono altri coefficienti diversi da zero e il valore nella colonna dei termini noti è positivo, il sistema è infeasible
        if (!has_decision_variables && tableau[row_index].back() > 0) {
            return false;
        }
    }

    // Se esiste una riga del Tableau con il termine noto negativo, il sistema è infeasible
    for (const auto& row : tableau) {
        if (row.back() < 0) {

            return false;
        }
    }
    // altrimenti il sistema è feasible
    return true;  
}


/// @brief metodo che appliuca la fase "pivot" dell'algoritmo del simplesso
/// @tparam T generico
/// @param pivot_row indice di riga della variabile uscente
/// @param pivot_column indice di colonna della variabile entrante
template <typename T>
void LinearConstrainSystem<T>::pivot(int pivot_row, int pivot_column) {

    // Aggiorno gli indici delle variabili di base
    base[pivot_row] = pivot_column;
    print_base();
    // Numero di colonne del Tableau
    int tot_columns = num_constrains + num_variables + EQ_cases + 1;
    // Numero di righe del tableau;
    int tot_rows = tableau.size();
    // elemento pivot
    T pivot_element = tableau[pivot_row][pivot_column];

    // Divido tutti gli elementi della riga pivot per l'elemento pivot
    for (auto& element : tableau[pivot_row]) {

        element /= pivot_element;
    }

    // Sostituidco le righe che non sono la riga pivot, sottraendo ad esse un opportuno multiplo della riga pivot
    for (int row_index = 0; row_index < tot_rows; ++row_index) {
        // se non mi trovo sulla pivot row
        if (row_index != pivot_row) {

            T factor = tableau[row_index][pivot_column];

            for (int col_index = 0; col_index < tot_columns; ++col_index) {

                // eseguo la combinazione lineare delle rgihe per portare gli altri elemnti della pivot column a 0
                tableau[row_index][col_index] -= factor * tableau[pivot_row][col_index];
            }
        }
    }
    print_tableau();
}


/// @brief metodo che otimizza c*x applicando "pivot" al Tableau
/// @tparam T generico
/// @param solution vettore che conterrà la soluzione
/// @param c vettore dei coefficienti della funzione obiettivo
/// @param type tipo di ottimizzazione
/// @return oggetto di tipo SolutionType
template<typename T>
typename LinearConstrainSystem<T>::SolutionType LinearConstrainSystem<T>::optimize(std::vector<T>& solution,   std::vector<T>& c, const OptimizationType type) {

    // Eccezioni:
    // Verifico che il numero di coefficienti delle variabili decisionali sia uguale al numero di variabili decisionali
    if (c.size() != num_variables) {
        throw std::invalid_argument("Wrong number of variables in objective function");
    }
    // Verifico che il tipo di ottimizzazione sia corretto
    if (type != OptimizationType::MAX && type != OptimizationType::MIN) {
        throw std::invalid_argument("Invalid optimization type");
    }

    // Fase di preparazione del Tableau:
    // aggiungo la riga della funzione obiettivo al Tableau
    add_ObjFunc_Tableau(c, type);

    // Fase dell'algoritmo del Simplesso:
    // Eseguo il metodo pivot fintanto che non viene interrotto
    while (true) {

        // ottengo l'indice della variabile entrante
        int pivot_column = find_pivot_column(c);
        // Se è uguale a -1 non esistono più variabili da introdurre in base e interrompo il ciclo
        if (pivot_column == -1 ) {
            
            std::cout << "----End Simplex----" << std::endl;
            break; 
        }

        // ottengo l'indice della variabile in uscita
        int pivot_row = find_pivot_row(pivot_column);
        // Se uguale a -1 allora il sistema è illimitato ed esco dal programma
        if (pivot_row == -1) {
            std::cout << "The system is UNBOUNDED!" << std::endl;            
            exit(0);
        }

        // Fase di "pivot" dell'algoritmo del simplesso
        pivot(pivot_row, pivot_column);
    }

    // se il sistema è Infeasible il programma termina
    if (is_feasible() ==  false) {
        std::cout << "The system is INFEASIBLE!" << std::endl;
        exit(0);
    }

    // A questo punto il sistema di vincoli è feasible

    // scrivo in solution le soluzioni trovate
    solution.resize(num_variables); 

    for (size_t i = 0; i < num_variables; ++i) {
        // indice in cui si trovano le variabili decisionali
        int decision_variable = artificial_variables + i;
        // cerco nel vettore di base gli indici corrispondenti alle variabili decisionali
        auto index = std::find(base.begin(), base.end(), decision_variable); 

        if (index != base.end()) {
            // indice della riga del tableau che corrisponde alla variabile di base trovata.
            int row = index - base.begin();
            // prende il valore che sta all'ultimo posto della riga, cioè nella colonna dei termini noti, e salvo in solution
            solution[i] = tableau[row].back();
        }
    }

    return SolutionType::BOUNDED;
}


/// @brief metodo per trovare l'indice della variabile entrante
/// @tparam T generico
/// @param c vettore dei coefficienti della funzione obiettivo
/// @return L'indice della colonna associata alla variabile entrante (int)
template <typename T>
int LinearConstrainSystem<T>::find_pivot_column( std::vector<T>& c) {

    // assegno inizialmente indice -1 per gestire i casi particolari
    int pivot_column = -1;
    // pivot_value mi serve come elemento di confronto per trovare il minimo valore negativo nella riga della funzione obiettivo 
    T pivot_value = 0; 
    // Numero di colonne totali del Tableau
    int tot_columns = num_constrains + num_variables + EQ_cases + 1;
    // indice di riga della funzone obiettivo
    int ObjFunc_row = artificial_variables;

    // scorriamo tutti i coefficienti della riga della funzione obiettivo (escluso il suo termine noto) 
    // cercando l'elemento più piccolo tra quelli negativi
    for (size_t col_index = 0; col_index < tot_columns - 1; ++col_index) {

        // se il coefficiente della funzione obiettivo è negativo, controlliamo se è il minimo trovato finora
        if ( tableau[ObjFunc_row][col_index] < 0 && tableau[ObjFunc_row][col_index] < pivot_value ){

            // aggiorno l'inidce di pivot column
            pivot_column = col_index;
            // aggiorno il minimo valore trovato finora
            pivot_value = tableau[ObjFunc_row][col_index];
        }
    }

    std::cout<<"Pivot column entrante: "<< pivot_column<< std::endl;

    return pivot_column;
}


/// @brief metodo per individuare l'inidice della variabile in uscita dalla base
/// @tparam T generico
/// @param pivot_column indice della variabile entrante in base
/// @return ritorna l'indice della riga associata alla variabile uscente
template <typename T>
int LinearConstrainSystem<T>::find_pivot_row(int pivot_column) {

    // Inizializzo l'indice della variabile di base scelta a -1 per gestire i casi particolari
    int pivot_row = -1;
    // imposto un valore molto alto per il rapporto tra coefficienti, che mi servirà per individuare il più piccolo tra i rapporti
    T min_ratio = 10000;
    // riga della funzone obiettivo
    int ObjFunc_row = artificial_variables;

    // Per ogni riga tranne quella della funzione obiettivo
    for (int row_index = 0; row_index < ObjFunc_row; ++row_index)  {

        // Se il coefficiente nella riga corrente del tableau è positivo
        if (tableau[row_index][pivot_column] > 0) {

            // Calcolo il rapporto tra il termine noto e il coefficiente nella riga corrente del tableau
            T ratio = tableau[row_index].back() / tableau[row_index][pivot_column];

            // Se il rapporto è minore del rapporto più piccolo trovato finora
            if (ratio < min_ratio) {
                // aggiorno l'indice della variabile di base scelta 
                pivot_row = row_index;
                // aggiorno il rapporto più piccolo
                min_ratio = ratio;
            }
        }
    }

    std::cout << "Pivot row uscente: " << pivot_row << std::endl;
    std::cout << std::endl;
    // Restituisco l'indice della variabile di base scelta
    return pivot_row;
}


#endif // __simplex_functions_hpp__