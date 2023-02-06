#ifndef __LINEARCONSTRAINSYSTEM_HPP__
#define __LINEARCONSTRAINSYSTEM_HPP__

#include "Tableau.hpp"

/**
 * @brief Struct to represent a linear constrain system
 * 
 * @tparam T  is the parametric type
 */
template<typename T>
struct LinearConstrainSystem {

    enum class SolutionType {
        BOUNDED, // optimal solution found
        UNBOUNDED // the set of solutions is not bounded 
    };
    enum class ConstrainType {
        EQ, // ==
        LE, // <=
        GE, // >=
    };
    enum class OptimizationType { MIN, MAX };

    // derived type to save a constrain
    struct Constrain {
        // constrain coefficients
        std::vector<T> a;
        // known term
        T b;
        // constrain type
        ConstrainType type;
        // empty constructor
        Constrain() {}
        // initialization constructor
        Constrain(std::vector<T> a, T b, ConstrainType type) : a(a), b(b), type(type) {}
        // copy constructor
        Constrain(const Constrain& orig) : a(orig.a), b(orig.b), type(orig.type) {}
    };

    // empty constructor
    LinearConstrainSystem() {}
    // copy constructor
    LinearConstrainSystem(const LinearConstrainSystem& orig) : constrains(orig.constrains), tab(orig.tab) {}

    // Add constrain a*x type b, e.g., a*x <= b
    inline LinearConstrainSystem& add_constrain(const std::vector<T>& a, const T& b, const ConstrainType type){ 
        // adding constrain to vector of constrains
        constrains.emplace_back(a, b, type);
        return *this;
    }

    // method to check if the system is feasible
    bool is_feasible();
    // method to optimize c*x with respect to the constrain system with x 
    SolutionType optimize(std::vector<T>& solution, const std::vector<T>& c, const OptimizationType type);
    // method to print obtained results
    void print_result(SolutionType type, std::vector<T>& solution) const;
    // method to print the optimization problem given as input
    void print_Lcs(const std::vector<T>& c, const OptimizationType type) const;

  private:

    // vector containing objects of type Constrain
    std::vector<Constrain> constrains;
    // object of the struct Tableau
    Tableau<T> tab; 
    // flag to keep track whether the user has already executed the is_feasible method for a constrain system
    bool feasibility_test{false};
    
    // method to update useful information about Tableau construction
    void update_tableau_info();
    // method to check if input constrain are valid
    void check_valid_constrains() const;
    // method to check if input objective function is valid
    inline void check_valid_objFunc(const std::vector<T>& c, const OptimizationType type) const {
        // verifying that the number of decisional variables coefficients is equal to number of decisional variables
        if (c.size() != tab.num_variables) {
            throw std::invalid_argument("Wrong number of variables in objective function");
        }
    }
};



/**
 * @brief method to update input information inside tableau
 * 
 * @tparam T 
 */
template<typename T>
void LinearConstrainSystem<T>::update_tableau_info() {

    // updating number of constrains in tableau
    tab.num_constrains = constrains.size();
    // updating number of decisional variables in tableau
    tab.num_variables = constrains[0].a.size(); 
    // updating number of added variables in tableau (i.e. slack, surplus, artificial)
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
                // add an artificial variable
                tab.artificial_variables++;
                break;
        }
    }
}


/**
 * @brief method to check if input contrains are valid
 * 
 * @tparam T
 */
template <typename T>
void LinearConstrainSystem<T>::check_valid_constrains() const {
    // taking length of first constrain as comparison
    unsigned int expected_num_variables = constrains[0].a.size();

    for (auto const& constrain : constrains) {

        if (constrain.a.size() != expected_num_variables) {

            throw std::invalid_argument("All constrains must have the same number of variables");
        }
    }
}


/**
 * @brief method to establish if the constrain system is feasible
 * 
 * @tparam T
 * @return true if the system is feasible
 * @return false if the system is infeasible
 */
template <typename T>
bool LinearConstrainSystem<T>::is_feasible() {  

    // checking validity of input values
    check_valid_constrains();
    // updating input information received so far
    update_tableau_info();
    // creating a copy of the linear constrain system received so far
    LinearConstrainSystem<T> copy(*this);

    // updating dimension of constrains inserted so far to make place for the dummy variable
    for (size_t i=0; i< copy.constrains.size(); ++i) { 
        copy.constrains[i].a.push_back(0);
    }
    // creating constrain coefficients referring to dummy variable
    std::vector<T> a_dummy(copy.constrains[0].a.size()-1, 0);
    a_dummy.emplace_back(1);
    // adding constrain to constrain system
    copy.add_constrain(a_dummy, 0 ,ConstrainType::EQ);
    // updating dummy variable information inside tableau
    copy.tab.num_constrains++;
    copy.tab.num_variables++;
    copy.tab.artificial_variables++;
    // creating initial tableau
    copy.tab.create_initial_tableau(copy.constrains);
    // creating dummy objective variable
    std::vector<T> c(copy.tab.num_variables - 1,0);
    c.emplace_back(1);
    // adding dummy objective variable to tableau
    copy.tab.add_objFunc_tableau(c, LinearConstrainSystem<T>::OptimizationType::MAX);

    // executing symplex pivot method until it gets interrupted
    bool hasSimplexFinished = false;
    while (!hasSimplexFinished ) {
        // obtaining index of base-entering variable
        int pivot_column = copy.tab.find_pivot_column();
        // if the index is -1 then there are not variables to enter the base anymore: the loop interrupts
        if (pivot_column == -1 ) {    
            #ifdef PRINT        
            std::cout << "----End Simplex----" << std::endl<<std::endl;
            #endif // PRINT
            hasSimplexFinished = true; 
        }
        if (hasSimplexFinished == false) {
            // obtaining index of base-exiting variable
            int pivot_row = copy.tab.find_pivot_row(pivot_column);
            // pivot executes
            copy.tab.pivot(pivot_row, pivot_column);
        }
    }
    #ifdef PRINT 
    std::cout<< "FEASIBILITY TEST: "<< std::endl<<std::endl;  
    #endif // PRINT

    T solution = copy.tab.tableau[copy.tab.num_constrains].back();
    // if z<0 then the system is infeasible
    if (solution < 0) {
        throw std::runtime_error("The linear constraint system is INFEASIBLE.");
        return false;
    // otherwise it is feasible
    } else {
        #ifdef PRINT
        std::cout<< "The system is FEASIBLE!"<< std::endl<<std::endl;
        #endif // PRINT
        // updating the flag of feasibility_test
        feasibility_test = true;
        return true;
    }
}


/**
 * @brief method to optimize c*x executing pivot method on tableau
 * 
 * @tparam T
 * @param solution vector containing solution
 * @param c vector containing objective function coefficients
 * @param type optimization type
 * @return LinearConstrainSystem<T>::SolutionType 
 */
template<typename T>
typename LinearConstrainSystem<T>::SolutionType LinearConstrainSystem<T>::optimize(std::vector<T>& solution, const  std::vector<T>& c, const OptimizationType type) {
    // variable that will be returned
    LinearConstrainSystem<T>::SolutionType sol_type; 
    // if user has not executed is_feasible then do it
    if (feasibility_test == false) {
        is_feasible();
    }
    // checking input objective function
    check_valid_objFunc(c, type);
    // creating a copy of the LinearConstrainSystem obtained so far
    LinearConstrainSystem<T> copy(*this);
    // creating initial tableau
    copy.tab.create_initial_tableau(copy.constrains);
    // adding objective function row to tableau
    copy.tab.add_objFunc_tableau(c, type);

    // SIMPLEX ALGORITHM PROCEDURE:
    // Executing pivot method until it gets interrupted
    bool hasSimplexFinished = false;
    while (!hasSimplexFinished) {

        // obtaining base-entering variable index
        int pivot_column = copy.tab.find_pivot_column();
        // if pivot column is -1 then there are no variable that can be set in base anymore; symplex is interrupted
        if (pivot_column == -1 ) {            
            #ifdef PRINT
            std::cout << "----End Simplex----" << std::endl<<std::endl;
            #endif // PRINT
            hasSimplexFinished = true; 
        }
        if (hasSimplexFinished == false) {
            // obtaining base-exiting variable index
            int pivot_row = copy.tab.find_pivot_row(pivot_column);
            // if pivot row is -1 then the system is unbounded
            if (pivot_row == -1) {
                sol_type = SolutionType::UNBOUNDED; // update and then return the variable
                print_Lcs(c,type);
                print_result(sol_type, solution);  
                return sol_type;   
            }
            // performing pivot method
            copy.tab.pivot(pivot_row, pivot_column);
        }
    }
    // writing found solution
    solution.resize(copy.tab.num_variables); 

    for (size_t i = 0; i < copy.tab.num_variables; ++i) {
        // getting indexes of decisional variables
        size_t decision_variable = copy.tab.get_decVars_index() + i;
        // searching base vector for indexes of decisional variables
        auto index = std::find(copy.tab.base.begin(), copy.tab.base.end(), decision_variable); 

        if (index != copy.tab.base.end()) {
            // tableau row index corresponding to base variable found
            size_t row = index - copy.tab.base.begin();
            // taking last value of the row (i.e. the known term) and saving it in solution
            solution[i] = copy.tab.tableau[row].back();
        }
    }
    // saving z value at the end of solution vector
    solution.emplace_back(copy.tab.tableau[copy.tab.num_constrains].back());
    // printing optimization problem
    sol_type = SolutionType::BOUNDED;     
    print_Lcs(c,type);
    print_result(sol_type, solution);  

    return sol_type;
}


/**
 * @brief  method to print input optimization problem
 * 
 * @tparam T 
 * @param c coefficients of objective function vector
 * @param type optimization type
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
 * @brief method to print optimal solution found for the optimization problem
 * 
 * @tparam T
 * @param type optimization type
 * @param solution vector containing solution
 */
template<typename T>  
void LinearConstrainSystem<T>::print_result(SolutionType type, std::vector<T>& solution) const {

    // BOUNDED case
    if (type == LinearConstrainSystem<T>::SolutionType::BOUNDED) {
        
        std::cout << std::endl;  
        std::cout << "Bounded solution found:" << std::endl;

        for (size_t i = 0; i < solution.size()-1; i++) {
            
            std::cout << "x" << i + 1 << " = " << solution[i] << std::endl;           
        }
        std::cout << std::endl; 

        std::cout<< "Optimal value z= "<< solution.back() << std::endl<< std::endl;

    // UNBOUNDED case
    } else {
        std::cout << "UNBOUNDED SOLUTION" << std::endl<< std::endl;
    }
}

#endif // __LINEARCONSTRAINSYSTEM_HPP__
