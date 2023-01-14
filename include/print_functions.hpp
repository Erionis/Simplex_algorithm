#ifndef PRINT_FUNCTIONS_HPP
#define PRINT_FUNCTIONS_HPP

#include <vector>

#include "LinearConstrainSystem.hpp"

/// @brief metodo per stampare gli elementi in base
/// @tparam T generico
template<typename T>
void LinearConstrainSystem<T>::print_base() const {

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
void LinearConstrainSystem<T>::print_tableau() const {

    std::cout << std::endl;
    for (const auto& row : tableau) {
        for (const auto& element : row) {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;    
}

/// @brief metodo per stampare la soluzione ottima della funzione obiettivo
/// @tparam T generico
template<typename T>
void LinearConstrainSystem<T>::print_result(SolutionType type, std::vector<T>& solution) const {

    // indice della riga della funzione obiettivo
    int ObjFunc_row = artificial_variables;

    std::cout << std::endl;
    // Caso BOUNDED
    if (type == LinearConstrainSystem<double>::SolutionType::BOUNDED) {

        std::cout << "Final Tableau: " << std::endl;
        print_tableau();
        std::cout << "Final Base: " << std::endl;   
        print_base();     
        std::cout << std::endl;  
        std::cout << "Bounded solution found:" << std::endl;

        for (int i = 0; i < solution.size(); i++) {
            
            std::cout << "x" << i + 1 << " = " << solution[i] << std::endl;           
        }
        std::cout << std::endl; 

        std::cout<< "Optimal value z= "<< tableau[ObjFunc_row].back() << std::endl;

    // Caso UNBOUNDED
    } else {
        std::cout << "UNBOUNDED SOLUTION" << std::endl;
    }
}

#endif