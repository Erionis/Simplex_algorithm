# Advanced and Parallel Programming Exam 2022-2023

Developers: Vittorio Amoruso, Nicola Cortinovis, Erion Islamay and Nicola Zucchia

Project: Sequential Implementation of Simplex Algorithm with tableau and Big-M Method.

Note: Big-M is set to 1e9. Please be aware that input data greater than 1e9 may cause numerical cancellation problems. 
If it is the case we suggest scaling your data before running this program. 

### Repository Structure

* `include` folder containing header files to create instances of linear constrain systems and tableau to perform the Simplex Algorithm

    * `LinearConstrainSystem.hpp` header containing the definition of the struct Linear Constrain System and its associated methods
    * `Tableau.hpp` header containing the defition of the struct Tableau and its associated methods

* `examples` folder containing 4 source files for testing various cases of linear constrain systems

    * `main.cpp` source file for testing a standard maximization problem
    * `mainINFEASIBLE.cpp` source file for testing for an infeasible constrain system
    * `mainMIN.cpp`  source file for testing a minimization problem
    * `mainUNBOUND.cpp` source file for testing an unbounded problem

* `CMakeLists.txt` txt file necessary to compile code with CMake



## Documentation
```bash
doxygen
```

## Compile
```bash
cmake . -G Ninja
ninja
```

