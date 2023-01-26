#ifndef SIMPLEX_FUNCTIONS_HPP
#define SIMPLEX_FUNCTIONS_HPP


#include <algorithm>
#include <iostream>
#include <vector>


template<typename T>
struct LinearConstrainSystem;


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
    size_t artificial_variables{0};   

    enum class OptimizationType { MIN, MAX };

    // costruttore vuoto
    Tableau() {}

    // metodo per inizializzare il tableau nella forma canonica
    void create_initial_tableau(std::vector<typename LinearConstrainSystem<T>::Constrain>& constrains);

    // metodo per aggiungere la funzione obiettivo al Tableau
    void add_objFunc_tableau(std::vector<T>& c, const typename LinearConstrainSystem<T>::OptimizationType type);

    // metodo per aggiungere una riga al Tableau
    void add_row_tableau(const std::vector<T>& a, const T& b, int current_row);    

    // metodo per individuare la variabile entrante
    int find_pivot_column(std::vector<T>& c);

    // metodo per individuare la variabile uscente
    int find_pivot_row(int pivot_column);

    // metodo per effettuare un pivot
    void pivot(int pivot_row, int pivot_column);

    // metodo per stampare il Tableau
    void print_tableau() const;

    // stampa i valori in base
    void print_base() const;
};


/// @brief 
/// @tparam T 
/// @param constrains 
template<typename T>
void Tableau<T>::create_initial_tableau(std::vector<typename LinearConstrainSystem<T>::Constrain>& constrains) {
    // numero totale di colonne che avrà il Tableau
    size_t tot_columns = num_variables + artificial_variables + 1;

    // per ogni vincolo nel vettore constrains
    for(auto const &constrain: constrains){

        // creo la riga corrispondente nel Tablau
        tableau.emplace_back(tot_columns, 0);
        print_tableau();
        // contatore per memorizzare in che riga siamo 
        size_t current_row = tableau.size() - 1;    

        switch (constrain.type) {

            // Nel caso di vincolo "LE" mi basta aggiungere una variabile artificiale con coeff 1, 
            // i coefficienti delle variabili decisionali e il termine noto, nella riga corrente
            case LinearConstrainSystem<T>::ConstrainType::LE: {

                add_row_tableau(constrain.a, constrain.b, current_row);

                break;
            }

            // Nel caso di vincolo "GE" mi basta aggiungere una variabile artificiale con coeff 1, 
            // i coefficienti delle variabili decisionali e il termine noto ma cambiati di segno
            case LinearConstrainSystem<T>::ConstrainType::GE: {

                // creo un vettore con i coefficienti cambiati di segno di a
                std::vector<T> a_neg(constrain.a.size());
                std::transform(constrain.a.begin(), constrain.a.end(), a_neg.begin(), std::negate<T>()); 

                add_row_tableau( a_neg, constrain.b*(-1), current_row);

                break;
            }

            // Nel caso di vincolo "EQ" aggiungo 2 righe al Tableau: la prima considera il vincolo come se fosse LE 
            // mentre la seconda considera il vicnolo come se fosse GE
            case LinearConstrainSystem<T>::ConstrainType::EQ: {

                add_row_tableau(constrain.a, constrain.b, current_row);
                tableau.emplace_back(tot_columns, 0);
                current_row++;

                // creo un vettore con i coefficienti cambiati di segno di a
                std::vector<T> a_neg(constrain.a.size());
                std::transform(constrain.a.begin(), constrain.a.end(), a_neg.begin(), std::negate<T>()); 
                add_row_tableau(a_neg, constrain.b*(-1), current_row);

                break;
            }
        }        
    }   
}


/// @brief  Metodo per aggiungere una riga al Tableau contenete le informazioni del vincolo
/// @tparam T generico
/// @param a vettore dei coefficienti delle variabili decisionali
/// @param b termine noto del vincolo
/// @param current_row intero corrispondente all'indice della riga del Tableau che si sta aggiungendo
template<typename T>
void Tableau<T>::add_row_tableau(const std::vector<T>& a, const T& b, int current_row){

    // assegno coefficiente 1 alla variabile artificiale
    tableau[current_row][current_row] = 1;
    // inserisco i coefficienti del vettore a e il termine noto b nella posizione corretta
    for (size_t i = 0; i < num_variables; ++i)  {

        tableau[current_row][artificial_variables + i] = a[i];
        tableau[current_row].back() = b;
    }  
    // aggiorno gli elementi in base
    base.emplace_back(current_row);
}


/// @brief metodo per aggiungere la riga della funzione obiettivo al Tableau
/// @tparam T generico
/// @param c vettore dei coefficienti della funzione obiettivo
/// @param type Tipo di ottimizzazione
template<typename T>
void Tableau<T>::add_objFunc_tableau(std::vector<T>& c, const typename LinearConstrainSystem<T>::OptimizationType type) {

    // Numero di colonne totali del Tableau
    size_t tot_columns = artificial_variables + num_variables + 1;
    // Indice di riga della funzione obiettivo
    size_t ObjFunc_row = artificial_variables;
    // creo una nuova riga nel Tableau            
    tableau.emplace_back(tot_columns, 0);

    switch (type) {

        // Caso di MINIMIZZAZIONE
        case LinearConstrainSystem<T>::OptimizationType::MIN: {
            // Aggiungo nella nuova riga i coefficienti della funzione obiettivo nella posizione corretta
            for (size_t i = 0; i < num_variables; ++i)  {

                tableau[ObjFunc_row][ObjFunc_row + i] = c[i];
            }
            break;
        }

        // Caso di MASSIMIZZAZIONE
        case LinearConstrainSystem<T>::OptimizationType::MAX: {
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


/// @brief metodo che appliuca la fase "pivot" dell'algoritmo del simplesso
/// @tparam T generico
/// @param pivot_row indice di riga della variabile uscente
/// @param pivot_column indice di colonna della variabile entrante
template <typename T>
void Tableau<T>::pivot(int pivot_row, int pivot_column) {

    // Aggiorno gli indici delle variabili di base
    base[pivot_row] = pivot_column;
    print_base();
    // Numero di colonne del Tableau
    size_t tot_columns = artificial_variables + num_variables + 1;
    // Numero di righe del tableau;
    size_t tot_rows = tableau.size();
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

            for (size_t col_index = 0; col_index < tot_columns; ++col_index) {

                // eseguo la combinazione lineare delle rgihe per portare gli altri elemnti della pivot column a 0
                tableau[row_index][col_index] -= factor * tableau[pivot_row][col_index];
            }
        }
    }
    print_tableau();
}


/// @brief metodo per trovare l'indice della variabile entrante
/// @tparam T generico
/// @param c vettore dei coefficienti della funzione obiettivo
/// @return L'indice della colonna associata alla variabile entrante (int)
template <typename T>
int Tableau<T>::find_pivot_column( std::vector<T>& c) {

    // assegno inizialmente indice -1 per gestire i casi particolari
    int pivot_column = -1;
    // pivot_value mi serve come elemento di confronto per trovare il minimo valore negativo nella riga della funzione obiettivo 
    T pivot_value = 0; 
    // Numero di colonne totali del Tableau
    size_t tot_columns = artificial_variables + num_variables +  1;
    // indice di riga della funzone obiettivo
    size_t ObjFunc_row = artificial_variables;

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
int Tableau<T>::find_pivot_row(int pivot_column) {

    // Inizializzo l'indice della variabile di base scelta a -1 per gestire i casi particolari
    int pivot_row = -1;
    // imposto un valore molto alto per il rapporto tra coefficienti, che mi servirà per individuare il più piccolo tra i rapporti
    T min_ratio = 10000;
    // riga della funzone obiettivo
    size_t ObjFunc_row = artificial_variables;

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


/// @brief metodo per stampare gli elementi in base
/// @tparam T generico
template<typename T>
void Tableau<T>::print_base() const {

    std::cout << "Base: ";
    for (const auto& index : base) {
        std::cout << index << " ";
    }
    std::cout << std::endl;    
    std::cout << std::endl;  
}


/// @brief metodo per stampare il Tableau
/// @tparam T generico
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