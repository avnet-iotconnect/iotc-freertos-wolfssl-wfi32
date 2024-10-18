/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "wolfssl/wolfcrypt/logging.h"

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************
#define APP_VERSION "3.3"

void wolfssl_log(const int logLevel, const char *const logMessage) {
//    if (logLevel <= INFO_LOG) {
        if (0 == strcmp("wolfSSL error occurred, error = -323", logMessage)) {
            return;
        } else if (0 == strcmp("wolfSSL Leaving SSL_get_error, return -323", logMessage)) {
            return;
        } else if (0 == strcmp("wolfSSL Entering SSL_get_error", logMessage)) {
            return;
        } else if (0 == strcmp("wolfSSL Entering SSL_connect()", logMessage)) {
            return;
        }
        printf("WS(%d): %s\n", logLevel, logMessage);
//    }
}

void pic32mzq_debug_print_cb(const char* fmt, ...) {
    (void) fmt;
    printf(".\n");
}

#include "wdrv_pic32mzw_debug.h" // HACK: for WDRV_PIC32MZW_DebugRegisterCallback
int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );

    wolfSSL_SetLoggingCb(wolfssl_log);
    wolfSSL_Debugging_ON();
    
    extern void WDRV_PIC32MZW_DebugRegisterCallback(WDRV_PIC32MZW_DEBUG_PRINT_CALLBACK const pfDebugPrintCallback);
    WDRV_PIC32MZW_DebugRegisterCallback(pic32mzq_debug_print_cb);

    SYS_CONSOLE_PRINT("Application Version Number: %s\r\n",APP_VERSION);
    SYS_CONSOLE_PRINT("Application Build - %s : %s\r\n", __DATE__, __TIME__);
    
    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

