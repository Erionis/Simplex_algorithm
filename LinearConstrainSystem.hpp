#ifndef __LinearConstrainSystem_hpp__
#define __LinearConstrainSystem_hpp__

#include <algorithm>
#include <iostream>
#include <vector>


template<typename T>
struct LinearConstrainSystem {

    // campo dati per il tableau
    std::vector<std::vector<T>> tableau;

    // indici delle variabili di base
    std::vector<int> base;

    // Numero di incognite
    int num_variables;

    // Numero di vincoli
    int num_constrains;

    // Numero di casi EQ
    int EQ_cases = 0;

    // Numero totale di variabili artificiali usate
    int artificial_variables = num_constrains + EQ_cases;

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

    // costruttore della struct
    LinearConstrainSystem(int num_constrains, int num_variables, int EQ_cases) 
        : num_constrains{num_constrains}, num_variables{num_variables}, EQ_cases{EQ_cases} {}

    // Aggiunge il vincolo a*x type b, e.g., a*x <= b
    LinearConstrainSystem& add_constrain(const std::vector<T>& a, const T& b, const ConstrainType type);

    // testa se il sistema di vincoli è soddisfacibile
    bool is_feasible() ;

    // ottimizza c*x rispetto al sistema di vincoli con x 
    SolutionType optimize(std::vector<T>& solution,  std::vector<T>& c, const OptimizationType type);

    // metodo per stampare il Tableau
    void print_tableau() const;

    // metodo per stampare elementi della base
    void print_base() const;

    // metodo per stampare i risultati ottenuti
    void print_result(SolutionType type, std::vector<T>& solution) const;

  private:

    // metodo per aggiungere al Tableau una riga associata ad un vincolo 
    void add_row_tableau(const std::vector<T>& a, const T& b, int current_row);

    // metodo per aggiungere la riga della funzione obiettivo al Tableau
    void add_ObjFunc_Tableau(std::vector<T>& c, const OptimizationType type);

    // metodo per individuare la variabile entrante
    int find_pivot_column(std::vector<T>& c);

    // metodo per individuare la variabile uscente
    int find_pivot_row(int pivot_column);
    
    // metodo per la fase di pivot
    void pivot(int pivot_row, int pivot_column);
};

#endif //__LinearConstrainSystem_hpp__