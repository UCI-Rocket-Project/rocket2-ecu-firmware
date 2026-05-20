#ifndef CPP_MAIN_H // Prevents multiple inclusions of this header
#define CPP_MAIN_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" { // Disables C++ name mangling so C code can link to this
#endif

/**
 * @brief Entry point for the C++ application
 * This function initializes the C++ runtime and starts the main application loop.  
 */
void cpp_main(void);

#ifdef __cplusplus
} // Closes extern "C" block
#endif

#endif // CPP_MAIN_H

