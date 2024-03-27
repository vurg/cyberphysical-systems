#include <iostream>
#include "PrimeChecker.hpp" // includes prime checker header file

int main(int argc, char** argv) {
    // checks for correct number of arguments
    if (argc == 2) {
        int number = std::stoi(argv[1]);
        PrimeChecker pc;
        // Update current name
        std::cout << "Vurgun, Nasit" << number << " is a prime number? " << pc.isPrime(number) << std::endl;
    }
    return 0; // returns 0, indicating program completion
}
