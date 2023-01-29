#ifndef TABLEAU_HPP
#define TABLEAU_HPP


#include <algorithm>
#include <iostream>
#include <vector>
#include <limits>


template<typename T>
struct LinearConstrainSystem;

/**
 * @brief Struct rappresentante il Tableau e tutte le funzioni ad essso collegate
 * 
 * @tparam T generico
 */
template<typename T>
struct Tableau {
    
    // campo dati per il tableau
    std::vector<std::vector<T>> tableau;
    // indici delle variabili di base
    std::vector<size_t> base;
    // Numero di incognite
    size_t num_variables{0};
    // Numero di vincoli
    size_t num_constrains{0};
    // Numero di variabili artificiali
    size_t slack_variables{0};   
    // Numero di variabili artificiali
    size_t surplus_variables{0};   
    // Numero di variabili artificiali
    size_t artificial_variables{0};   


    // costruttore vuoto
    Tableau() {}
    // costruttore di copia
    Tableau(const Tableau<T>& other);

    // metodo per ottenere il numero di colonne del tableau
    inline size_t get_total_columns() { return num_variables + slack_variables + surplus_variables + artificial_variables + 1; }
    // metodo per ottenere l'indice delle colonne dedicate alle variabili decisionali
    inline size_t get_decVars_index(){ return slack_variables + surplus_variables + artificial_variables; }
    // metodo per inizializzare il tableau nella forma canonica
    void create_initial_tableau(std::vector<typename LinearConstrainSystem<T>::Constrain>& constrains);
    // metodo per aggiungere la funzione obiettivo al Tableau
    void add_objFunc_tableau(std::vector<T>& c, const typename LinearConstrainSystem<T>::OptimizationType type);
    // metodo per stampare il Tableau
    void print_tableau() const;
    // stampa i valori in base
    void print_base() const;    
    // metodo per aggiungere una riga al Tableau nel caso LE
    void add_LE_row_tableau(const std::vector<T>& a, const T& b, size_t current_row);
    // metodo per aggiungere una riga al Tableau nel caso GE
    void add_GE_row_tableau(const std::vector<T>& a, const T& b, size_t current_row);  
    // metodo per aggiungere una riga al Tableau nel caso EQ
    void add_EQ_row_tableau(const std::vector<T>& a, const T& b, size_t current_row);      
    // metodo per individuare la variabile entrante
    int find_pivot_column();  // CAPIRE QUANDO SI DEVE METTERE CONST
    // metodo per individuare la variabile uscente
    int find_pivot_row(int pivot_column);
    // metodo per effettuare un pivot
    void pivot(int pivot_row, int pivot_column);

 private: // SCEGLIERE DOVE METTERE PRIVATE

    // definisco Big-M come un valore molto grande
    double BIG_M = 1e9; // CAPIRE MEGLIO COME DEFINIRLO
    // indici (i,j) della posizone delle variabili artificiali nel tableau
    std::vector<std::pair<size_t, size_t>> artificial_var_indices;
};

/**
 * @brief Costruttore di copia
 * 
 * @tparam T generico
 * @param orig oggetto originale che viene copiato
 */
template<typename T>
Tableau<T>::Tableau(const Tableau<T>& orig) {
    tableau = orig.tableau;
    base = orig.base;
    artificial_var_indices = orig.artificial_var_indices;
    num_variables = orig.num_variables;
    num_constrains = orig.num_constrains;
    slack_variables = orig.slack_variables;
    surplus_variables = orig.surplus_variables;
    artificial_variables = orig.artificial_variables;
    BIG_M = orig.BIG_M;
}


/**
 * @brief metodo per inserire i vincoli del sistema nel Tableau
 * 
 * @tparam T generico
 * @param constrains vettore di oggetti di tipo Constrain rappresentanti i vincoli del sistema 
 */
template<typename T>
void Tableau<T>::create_initial_tableau(std::vector<typename LinearConstrainSystem<T>::Constrain>& constrains) {

    // per ogni vincolo nel vettore constrains
    for(auto const &constrain: constrains){

        // creo la riga corrispondente nel Tablau
        tableau.emplace_back(get_total_columns(), 0);
        // contatore per memorizzare in che riga siamo 
        size_t current_row = tableau.size() - 1;    

        switch (constrain.type) {

            // Nel caso di vincolo "LE" mi basta aggiungere una variabile artificiale con coeff 1, 
            // i coefficienti delle variabili decisionali e il termine noto, nella riga corrente
            case LinearConstrainSystem<T>::ConstrainType::LE: {

                if (constrain.b < 0){
                    // creo un vettore con i coefficienti cambiati di segno di a
                    std::vector<T> a_neg(constrain.a.size());
                    std::transform(constrain.a.begin(), constrain.a.end(), a_neg.begin(), std::negate<T>()); 
                
                    add_GE_row_tableau(a_neg, constrain.b*(-1), current_row);    
                } else {
                    add_LE_row_tableau(constrain.a, constrain.b, current_row);
                }

                break;
            }

            // Nel caso di vincolo "GE" mi basta aggiungere una variabile artificiale con coeff 1, 
            // i coefficienti delle variabili decisionali e il termine noto ma cambiati di segno
            case LinearConstrainSystem<T>::ConstrainType::GE: {

                if (constrain.b < 0){
                    // creo un vettore con i coefficienti cambiati di segno di a
                    std::vector<T> a_neg(constrain.a.size());
                    std::transform(constrain.a.begin(), constrain.a.end(), a_neg.begin(), std::negate<T>()); 

                    add_LE_row_tableau(a_neg, constrain.b*(-1), current_row);    
                } else {
                    add_GE_row_tableau(constrain.a, constrain.b, current_row);
                }
                
                break;
            }

            // Nel caso di vincolo "EQ" aggiungo 2 righe al Tableau: la prima considera il vincolo come se fosse LE 
            // mentre la seconda considera il vicnolo come se fosse GE
            case LinearConstrainSystem<T>::ConstrainType::EQ: {

                if (constrain.b < 0){
                    // creo un vettore con i coefficienti cambiati di segno di a
                    std::vector<T> a_neg(constrain.a.size());
                    std::transform(constrain.a.begin(), constrain.a.end(), a_neg.begin(), std::negate<T>()); 

                    add_EQ_row_tableau(a_neg, constrain.b*(-1), current_row);    
                } else {
                    add_EQ_row_tableau(constrain.a, constrain.b, current_row);
                }

                break;
            }
        }        
    }
}


/**
 * @brief metodo per aggiungere un vincolo di tipo "minore o uguale" in una riga del Tableau
 * 
 * @tparam T generico
 * @param a vettore dei coefficienti delle variabili decisionali del vincolo
 * @param b termine noto del vincolo
 * @param current_row indice di riga corrente del tableau
 */
template<typename T>
void Tableau<T>::add_LE_row_tableau(const std::vector<T>& a, const T& b, size_t current_row){
    // se non mi trovo nella prima riga
    if(current_row>0){
        // per tutte le colonne dedicate alle variabili aggiuntive
        for (size_t i = 0; i < get_decVars_index(); ++i) {
            // nel caso in cui nella riga precedente c'è un elemento -1
            if (tableau[current_row-1][i] == 1 && tableau[current_row-1][i+1] == -1) {
                // aggiungo il coefficiente della variabile di slack
                tableau[current_row][i+2] = 1;
                // inserisco la variabile in base
                base.emplace_back(i+2);
                break;
            // nel caso in cui nella riga precedente c'è un elemento 1
            }else if(tableau[current_row-1][i] == 1) {
                // aggiungo il coefficiente della variabile di slack                
                tableau[current_row][i+1] = 1;
                // inserisco la variabile in base                
                base.emplace_back(i+1);
                break;
            }
        }
    // se mi trovo nella prima riga
    }else{
        // aggiungo il coefficiente della variabile di slack
        tableau[current_row][0] = 1;
        // inserisco la variabile in base
        base.emplace_back(0);
    }

    // inserisco i coefficienti del vettore a nella posizione corretta
    for (size_t i = 0; i < num_variables; ++i)  {
        tableau[current_row][get_decVars_index() + i] = a[i];
    }
    // aggiungo il termine noto 
    tableau[current_row].back() = b;
}


/**
 * @brief metodo per aggiungere un vincolo di tipo "maggiore o uguale" in una riga del Tableau
 * 
 * @tparam T generico
 * @param a vettore dei coefficienti delle variabili decisionali del vincolo
 * @param b termine noto del vincolo
 * @param current_row indice di riga corrente del tableau
 */
template<typename T>
void Tableau<T>::add_GE_row_tableau(const std::vector<T>& a, const T& b, size_t current_row){

    if(current_row>0){

        for (size_t i = 0; i < get_decVars_index(); ++i) {

            if (tableau[current_row-1][i] == 1 && tableau[current_row-1][i+1] == -1) {
                tableau[current_row][i+2] = 1;
                tableau[current_row][i+3] = -1;
                base.emplace_back(i+2);
                artificial_var_indices.emplace_back(std::make_pair(current_row, i+2));
                break;
            }else if(tableau[current_row-1][i] == 1) {
                tableau[current_row][i+1] = 1;
                tableau[current_row][i+2] = -1;
                base.emplace_back(i+1);
                artificial_var_indices.emplace_back(std::make_pair(current_row, i+1));
                break;
            }
        }
    }else{
        tableau[current_row][0] = 1;
        tableau[current_row][1] = -1;
        base.emplace_back(0);
        artificial_var_indices.emplace_back(std::make_pair(current_row, 0));
    }

    // inserisco i coefficienti del vettore a e il termine noto b nella posizione corretta
    for (size_t i = 0; i < num_variables; ++i)  {

        tableau[current_row][get_decVars_index() + i] = a[i];
    } 
    // aggiungo il termine noto  
    tableau[current_row].back() = b; 
}


/**
 * @brief metodo per aggiungere un vincolo di tipo "maggiore o uguale" in una riga del Tableau
 * 
 * @tparam T generico
 * @param a vettore dei coefficienti delle variabili decisionali del vincolo
 * @param b termine noto del vincolo
 * @param current_row indice di riga corrente del tableau
 */
template<typename T>
void Tableau<T>::add_EQ_row_tableau(const std::vector<T>& a, const T& b, size_t current_row){


    if(current_row>0){

        for (size_t i = 0; i < get_decVars_index(); ++i) {

            if (tableau[current_row-1][i] == 1 && tableau[current_row-1][i+1] == -1) {
                tableau[current_row][i+2] = 1;
                base.emplace_back(i+2);
                artificial_var_indices.emplace_back(std::make_pair(current_row, i+2));
                break;
            }else if(tableau[current_row-1][i] == 1) {
                tableau[current_row][i+1] = 1;
                base.emplace_back(i+1);
                artificial_var_indices.emplace_back(std::make_pair(current_row, i+1));
                break;
            }
        }
    }else{
        tableau[current_row][0] = 1;
        base.emplace_back(0);
                artificial_var_indices.emplace_back(std::make_pair(current_row, 0));
    }


    // inserisco i coefficienti del vettore a nella posizione corretta
    for (size_t i = 0; i < num_variables; ++i)  {

        tableau[current_row][get_decVars_index() + i] = a[i];
    }
    // aggiungo il termine noto  
    tableau[current_row].back() = b;
}


/**
 * @brief metodo per aggiungere la riga della funzione obiettivo nel Tableau col "Big-M method"
 * 
 * @tparam T generico
 * @param c vettore dei coefficienti della funzione obiettivo
 * @param type tipo di ottimizzazione
 */
template<typename T>
void Tableau<T>::add_objFunc_tableau(std::vector<T>& c, const typename LinearConstrainSystem<T>::OptimizationType type) {

    // Indice di riga della funzione obiettivo
    size_t ObjFunc_row = num_constrains;
    // creo una nuova riga nel Tableau            
    tableau.emplace_back(get_total_columns(), 0);
    // metto i valori Big-M corrsipondenti alle variabili artificiali nella riga della funzione obiettivo

    switch (type) {

        // Caso di MINIMIZZAZIONE
        case LinearConstrainSystem<T>::OptimizationType::MIN: {
            // Aggiungo nella nuova riga i coefficienti della funzione obiettivo nella posizione corretta
            for (size_t i = 0; i < num_variables; ++i)  {

                tableau[ObjFunc_row][get_decVars_index() + i] = c[i];
            }
            break;
        }

        // Caso di MASSIMIZZAZIONE
        case LinearConstrainSystem<T>::OptimizationType::MAX: {
            // Aggiungo nella nuova riga i coefficienti della funzione obiettivo cambiati di segno
            for (size_t i = 0; i < num_variables; ++i)  {
                
                tableau[ObjFunc_row][get_decVars_index() + i] = c[i]*(-1);
            }
            break;
        }
    }
    // Aggiungo il termine noto uguale a 0 della funzione obiettivo
    tableau[ObjFunc_row].back()= 0;

    // Fase del "Big-M method"

    // aggiungo il valore alle variabili artificiali nella riga della funziona obiettivo
    for (const auto& indeces : artificial_var_indices) {
        tableau[ObjFunc_row][indeces.second] = BIG_M;
    }
    print_tableau();
    // e gli elimino facendo le opportune combinazioni lineari sulla funziona obiettivo
    for (const auto& indeces : artificial_var_indices) {
        T factor = tableau[ObjFunc_row][indeces.second];
        for (size_t col_index = 0; col_index < get_total_columns(); ++col_index) {
            // eseguo la combinazione lineare delle rgihe per portare gli altri elemnti della pivot column a 0
            tableau[ObjFunc_row][col_index] -= factor * tableau[indeces.first][col_index];
        }        
    }

    // da questo punto può partire l'algorimto del simplesso
    std::cout << "---Start Simplex---" << std::endl;
    std::cout << "Tableau iniziale: " << std::endl;
    print_tableau();
    std::cout << "Base iniziale: " << std::endl;
    print_base();
}


/**
 * @brief metodo che implementa la fase "pivot" dell'algoritmo del simplesso
 * 
 * @tparam T generico
 * @param pivot_row indice di riga della variabile uscente
 * @param pivot_column indice di colonna della variabile entrante
 */
template <typename T>
void Tableau<T>::pivot(int pivot_row, int pivot_column) {

    // Aggiorno gli indici delle variabili di base
    base[pivot_row] = pivot_column;
    print_base();

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

            for (size_t col_index = 0; col_index < get_total_columns(); ++col_index) {

                // eseguo la combinazione lineare delle rgihe per portare gli altri elemnti della pivot column a 0
                tableau[row_index][col_index] -= factor * tableau[pivot_row][col_index];
            }
        }
    }
    print_tableau();
}


/**
 * @brief metodo per individuare l'indice di colonna della variabile entrante
 * 
 * @tparam T generico
 * @return int indice della colonna associata alla variabile entrante
 */
template <typename T>
int Tableau<T>::find_pivot_column() {

    // assegno inizialmente indice -1 per gestire i casi particolari
    int pivot_column = -1;
    // pivot_value mi serve come elemento di confronto per trovare il minimo valore negativo nella riga della funzione obiettivo 
    T pivot_value = 0; 

    // indice di riga della funzone obiettivo
    size_t ObjFunc_row = num_constrains;

    // scorriamo tutti i coefficienti della riga della funzione obiettivo (escluso il suo termine noto) 
    // cercando l'elemento più piccolo tra quelli negativi
    for (size_t col_index = 0; col_index < get_total_columns() - 1; ++col_index) {

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


/**
 * @brief metodo per individuare l'inidice della variabile in uscita dalla base
 * 
 * @tparam T generico
 * @param pivot_column indice della variabile entrante in base
 * @return int indice della riga associata alla variabile uscente
 */
template <typename T>
int Tableau<T>::find_pivot_row(int pivot_column) {

    // Inizializzo l'indice della variabile di base scelta a -1 per gestire i casi particolari
    int pivot_row = -1;
    // imposto un valore molto alto per il rapporto tra coefficienti, che mi servirà per individuare il più piccolo tra i rapporti
    T min_ratio = std::numeric_limits<T>::max();
    // riga della funzone obiettivo
    size_t ObjFunc_row = num_constrains;

    // Per ogni riga tranne quella della funzione obiettivo
    for (size_t row_index = 0; row_index < ObjFunc_row; ++row_index)  {

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


/**
 * @brief metodo per stampare gli elementi in base
 * 
 * @tparam T generico
 */
template<typename T>
void Tableau<T>::print_base() const {

    std::cout << "Base: ";
    for (const auto& index : base) {
        std::cout << index << " ";
    }
    std::cout << std::endl;    
    std::cout << std::endl;  
}


/**
 * @brief metodo per stampare il Tableau
 * 
 * @tparam T generico
 */
template<typename T>
void Tableau<T>::print_tableau() const {

    std::cout << std::endl;
    for (const auto& row : tableau) {
        for (const auto& element : row) {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;    
}

#endif