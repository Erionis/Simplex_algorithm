#ifndef __TABLEAU_HPP__
#define __TABLEAU_HPP__


#include <algorithm>
#include <iostream>
#include <vector>
#include <limits>


template<typename T>
struct LinearConstrainSystem;



/**
 * @brief class for Tableau and linked functions
 * 
 * @tparam T 
 */
template<typename T>
class Tableau {
    
    // members of the tableau for saving its data

    std::vector<std::vector<T>> tableau;    //!< tableau matrix
    std::vector<size_t> base;               //!< vector for base variable index
    size_t num_variables{0};                //!< number of variables
    size_t num_constrains{0};               //!< number of constrains
    size_t slack_variables{0};              //!< number of slack variables
    size_t surplus_variables{0};            //!< number of surplus variables
    size_t artificial_variables{0};         //!< number of artificial variables
    double BIG_M = 1e9;                     //!< defining Big_M with a very large value
    
    std::vector<std::pair<size_t, size_t>> artificial_var_indices;  //!< indexes (i,j) for position of artificial variables inside tableau

    // empty constructor
    Tableau() {}
    // copy constructor 
    Tableau(const Tableau<T>& orig);

    /**
     * @brief method to get number of columns in tableau
     */
    inline size_t get_total_columns() { return num_variables + slack_variables + surplus_variables + artificial_variables + 1; }
    /**
     * @brief method to get index of columns for decisional variables
    */
    inline size_t get_decVars_index() { return slack_variables + surplus_variables + artificial_variables; }
    
    // method to add system constrains in Tableau
    void create_initial_tableau(std::vector<typename LinearConstrainSystem<T>::Constrain>& constrains);
    // method to add objective function row with "Big-M" method
    void add_objFunc_tableau(const std::vector<T>& c, const typename LinearConstrainSystem<T>::OptimizationType type);
    // method to add a row to tableau when the case is LE
    void add_LE_row_tableau(const std::vector<T>& a, const T& b, size_t current_row);
    // method to add a row to tableau when the case is GE
    void add_GE_row_tableau(const std::vector<T>& a, const T& b, size_t current_row);  
    // method to add a row to tableau when the case is EQ
    void add_EQ_row_tableau(const std::vector<T>& a, const T& b, size_t current_row);      
    // method to identify base entering variable
    int find_pivot_column(); 
    // method to identify base exiting variable
    int find_pivot_row(int pivot_column);
    // method to perform pivot operation
    void pivot(int pivot_row, int pivot_column);

    #ifdef PRINT
    /**
     * @brief method to print the tableau
    */
    void print_tableau() const;
    /**
     * @brief method to print base values
    */
    void print_base() const;    
    #endif // PRINT

    friend struct LinearConstrainSystem<T>;
};


/**
 * @brief Copy constructor
 * 
 * @tparam T 
 * @param orig original object to be copied
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
 * @brief method to add system constrains in Tableau
 * 
 * @tparam T
 * @param constrains vector of Constrain objects to represent system constrains
 */
template<typename T>
void Tableau<T>::create_initial_tableau(std::vector<typename LinearConstrainSystem<T>::Constrain>& constrains) {

    // for every constrain in Constrain vector
    for(auto const &constrain: constrains){

        // creating corresponding row in tableau
        tableau.emplace_back(get_total_columns(), 0);
        // counter to save current row
        size_t current_row = tableau.size() - 1;    

        switch (constrain.type) {

            case LinearConstrainSystem<T>::ConstrainType::LE: {
                // if constant term is negative
                if (constrain.b < 0){
                    // creating a vector with opposite coefficients of a
                    std::vector<T> a_neg(constrain.a.size());
                    std::transform(constrain.a.begin(), constrain.a.end(), a_neg.begin(), std::negate<T>()); 
                    // considering the case as if it was GE
                    add_GE_row_tableau(a_neg, constrain.b*(-1), current_row);    
                } else {
                    add_LE_row_tableau(constrain.a, constrain.b, current_row);
                }

                break;
            }

            case LinearConstrainSystem<T>::ConstrainType::GE: {
                // if constant term is negative 
                if (constrain.b < 0){
                    // creating a vector with opposite coefficients of a
                    std::vector<T> a_neg(constrain.a.size());
                    std::transform(constrain.a.begin(), constrain.a.end(), a_neg.begin(), std::negate<T>()); 
                    // considering the case as if it was LE
                    add_LE_row_tableau(a_neg, constrain.b*(-1), current_row);    
                } else {
                    add_GE_row_tableau(constrain.a, constrain.b, current_row);
                }
                
                break;
            }

            case LinearConstrainSystem<T>::ConstrainType::EQ: {
                // if the constant term is negative
                if (constrain.b < 0){
                    // creating a vector with opposite coefficients of a
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
 * @brief method to add a LE constrain into a Tableau row
 * 
 * @tparam T
 * @param a vector of constrain's decisional variables coefficients
 * @param b constrain's constant term
 * @param current_row current row index inside tableau
 */
template<typename T>
void Tableau<T>::add_LE_row_tableau(const std::vector<T>& a, const T& b, size_t current_row){

    // if not in the first row
    if(current_row>0){
        // for every column reserved to additional variables
        for (size_t i = 0; i < get_decVars_index(); ++i) {
            // if in previous row there is an element equal to -1
            if (tableau[current_row-1][i] == 1 && tableau[current_row-1][i+1] == -1) {
                // adding coefficient of slack variable
                tableau[current_row][i+2] = 1;
                // adding base variable
                base.emplace_back(i+2);
                break;
            // if in previous row there is an element equal to 1
            }else if(tableau[current_row-1][i] == 1) {
                // adding coefficient of slack variable                
                tableau[current_row][i+1] = 1;
                // adding base variable                  
                base.emplace_back(i+1);
                break;
            }
        }
    // if in the first row
    }else{
        // adding coefficient of slack variable
        tableau[current_row][0] = 1;
        //  adding base variable
        base.emplace_back(0);
    }

    // inserting coefficients of vector a in correct position
    for (size_t i = 0; i < num_variables; ++i)  {
        tableau[current_row][get_decVars_index() + i] = a[i];
    }
    // adding constant term
    tableau[current_row].back() = b;
}


/**
 * @brief method to add a GE constrain into a tableau row
 * 
 * @tparam T
 * @param a vector for constrain's decisional variables coefficients
 * @param b constrain's constant term
 * @param current_row tableau's current row index
 */
template<typename T>
void Tableau<T>::add_GE_row_tableau(const std::vector<T>& a, const T& b, size_t current_row){

    // if not in first row
    if(current_row>0){
        // for every column regarding additional variables
        for (size_t i = 0; i < get_decVars_index(); ++i) {
            // if in the previous row there is an element equal to -1
            if (tableau[current_row-1][i] == 1 && tableau[current_row-1][i+1] == -1) {
                tableau[current_row][i+2] = 1;  // adding artificial variable
                tableau[current_row][i+3] = -1;  // adding surplus variable
                base.emplace_back(i+2);    // updating base
                // saving position of artificial variable
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
    // if in the first row
    }else{
        tableau[current_row][0] = 1;
        tableau[current_row][1] = -1;
        base.emplace_back(0);
        artificial_var_indices.emplace_back(std::make_pair(current_row, 0));
    }
    // inserting coefficients of vector a in correct place
    for (size_t i = 0; i < num_variables; ++i)  {

        tableau[current_row][get_decVars_index() + i] = a[i];
    } 
    // adding constant term  
    tableau[current_row].back() = b; 
}


/**
 * @brief method to add a EQ constrain into a row of tableau
 * 
 * @tparam T
 * @param a vector of constrain's decisional variable coefficients
 * @param b constrain's constant term 
 * @param current_row current row index inside tableau
 */
template<typename T>
void Tableau<T>::add_EQ_row_tableau(const std::vector<T>& a, const T& b, size_t current_row){

    // if not in first row
    if(current_row>0){
        // for every column reserved to additional variables
        for (size_t i = 0; i < get_decVars_index(); ++i) {
            // if there is an element equal to -1 in previous row
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
    // if in first row
    }else{
        tableau[current_row][0] = 1;
        base.emplace_back(0);
                artificial_var_indices.emplace_back(std::make_pair(current_row, 0));
    }

    // inserting coefficients of vector a in correct position
    for (size_t i = 0; i < num_variables; ++i)  {

        tableau[current_row][get_decVars_index() + i] = a[i];
    }
    // adding constant term
    tableau[current_row].back() = b;
}


/**
 * @brief method to add objective function row with "Big-M" method
 * 
 * @tparam T
 * @param c vector of objective function coefficients
 * @param type optimization type
 */
template<typename T>
void Tableau<T>::add_objFunc_tableau(const std::vector<T>& c, const typename LinearConstrainSystem<T>::OptimizationType type) {

    // index of objective function row
    size_t ObjFunc_row = num_constrains;
    // creating a new row inside tableau           
    tableau.emplace_back(get_total_columns(), 0);

    switch (type) {

        // Minimization case
        case LinearConstrainSystem<T>::OptimizationType::MIN: {
            // Adding to new row objective function coefficients in correct position
            for (size_t i = 0; i < num_variables; ++i)  {

                tableau[ObjFunc_row][get_decVars_index() + i] = c[i];
            }
            break;
        }
        // Maximization case
        case LinearConstrainSystem<T>::OptimizationType::MAX: {
            // Adding to new row i objective function opposite coefficients 
            for (size_t i = 0; i < num_variables; ++i)  {
                
                tableau[ObjFunc_row][get_decVars_index() + i] = c[i]*(-1);
            }
            break;
        }
    }
    // Adding constant term as 0 for objective function
    tableau[ObjFunc_row].back()= 0;

    // "Big-M method" phase:
    // adding M value to artificial variables in objective function row
    for (const auto& indeces : artificial_var_indices) {
        tableau[ObjFunc_row][indeces.second] = BIG_M;
    }
    #ifdef PRINT
    print_tableau();
    #endif // PRINT

    // deleting those values performing adequate linear combinations to objective function
    for (const auto& indeces : artificial_var_indices) {
        T factor = tableau[ObjFunc_row][indeces.second];
        for (size_t col_index = 0; col_index < get_total_columns(); ++col_index) {
            // performing linear combinations on rows to make other elements on pivot columns be 0
            tableau[ObjFunc_row][col_index] -= factor * tableau[indeces.first][col_index];
        }        
    }

    // now simplex algorithm can start
    #ifdef PRINT
    std::cout << "---Start Simplex---" << std::endl;
    std::cout << "Initial tableau: " << std::endl;
    print_tableau();
    std::cout << "Initial base: " << std::endl;
    print_base();
    #endif // PRINT
}


/**
 * @brief method for pivot operation
 * 
 * @tparam T
 * @param pivot_row index of base exiting variable row
 * @param pivot_column index of base entering variable column
 */
template <typename T>
void Tableau<T>::pivot(int pivot_row, int pivot_column) {

    // adding indexes of base variables
    base[pivot_row] = pivot_column;
    #ifdef PRINT
    print_base();
    #endif // PRINT
    // number of rows in tableau
    int tot_rows = tableau.size();
    // pivot element
    T pivot_element = tableau[pivot_row][pivot_column];
    // dividing all elements in pivot row by pivot element

    for (auto& element : tableau[pivot_row]) {
        element /= pivot_element;
    }

    // substituting all non-pivot rows subtracting to them an adequate multiple of pivot row
    for (int row_index = 0; row_index < tot_rows; ++row_index) {
        // if not in pivot row
        if (row_index != pivot_row) {
            T factor = tableau[row_index][pivot_column];
            for (size_t col_index = 0; col_index < get_total_columns(); ++col_index) {
                // performing linear combination of row to make other elements in pivot column to be 0
                tableau[row_index][col_index] -= factor * tableau[pivot_row][col_index];
            }
        }
    }

    #ifdef PRINT
    print_tableau();
    #endif // PRINT
}


/**
 * @brief method to determine index of base-entering variable column 
 * 
 * @tparam T
 * @return 'int' index of base-entering variable columns
 */
template <typename T>
int Tableau<T>::find_pivot_column() {

    // initially assigning index as -1 to deal with particular cases
    int pivot_column = -1;
    // pivot_value will be used for comparisons to find minimum negative value in objective function row
    T pivot_value = 0; 
    // index of objective function row
    size_t ObjFunc_row = num_constrains;

    // analyzing all coefficients of objective function row (apart from its constant term) 
    // searching for the minimum among its negative values
    for (size_t col_index = 0; col_index < get_total_columns() - 1; ++col_index) {

        // if objective function coefficient is negative, let's check if it is the minimum so far
        if ( tableau[ObjFunc_row][col_index] < 0 && tableau[ObjFunc_row][col_index] < pivot_value ){

            // updating index of pivot column
            pivot_column = col_index;
            // updating minimum value found so far
            pivot_value = tableau[ObjFunc_row][col_index];
        }
    }
    #ifdef PRINT
    std::cout<<"Pivot column entering: "<< pivot_column<< std::endl;
    #endif // PRINT
    return pivot_column;
}


/**
 * @brief method to find index of base exiting variable
 * 
 * @tparam T
 * @param pivot_column index of base entering variable
 * @return 'int' index of base exiting variable row
 */
template <typename T>
int Tableau<T>::find_pivot_row(int pivot_column) {

    // initially, base variable index is set to -1 to deal with particular cases
    int pivot_row = -1;
    // setting the highest possible value for ratio that will be used to identify the minimum among all ratios
    double min_ratio = std::numeric_limits<T>::max();
    // objective function row
    size_t ObjFunc_row = num_constrains;

    // for every row apart from objective function one
    for (size_t row_index = 0; row_index < ObjFunc_row; ++row_index)  {

        // if coefficient in current tableau row is positive
        if (tableau[row_index][pivot_column] > 0) {

            // calculating ratio between constant term and coefficient in current tableau row
            double ratio = tableau[row_index].back() / tableau[row_index][pivot_column];

            // if ratio is less than minimum ratio found so far
            if (ratio < min_ratio) {
                // updating index of selected base variable 
                pivot_row = row_index;
                // updating minimum ratio value
                min_ratio = ratio;
            }
        }
    }
    #ifdef PRINT
    std::cout << "Pivot row exiting: " << pivot_row << std::endl;
    std::cout << std::endl;
    #endif // PRINT
    // returing index of selected base variable
    return pivot_row;
}

#ifdef PRINT

/**
 * @brief method to print elements in base
 * 
 * @tparam T 
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

#endif // PRINT

#ifdef PRINT

/**
 * @brief method to print Tableau
 * 
 * @tparam T 
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
#endif // PRINT

#endif // __TABLEAU_HPP__