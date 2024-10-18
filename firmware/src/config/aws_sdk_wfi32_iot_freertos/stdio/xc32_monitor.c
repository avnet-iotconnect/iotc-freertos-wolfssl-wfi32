/*******************************************************************************
 Debug Console Source file

  Company:
    Microchip Technology Inc.

  File Name:
    xc32_monitor.c

  Summary:
    debug console Source File

  Description:
    None

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
#include <stddef.h>
#include "definitions.h"

extern int read(int handle, void *buffer, unsigned int len);
extern int write(int handle, void * buffer, size_t count);


int read(int handle, void *buffer, unsigned int len)
{
    int nChars = 0;
    bool success = false;
    if ((handle == 0)  && (len > 0U))
    {
        do
        {
            success = UART3_Read(buffer, 1);
        }while( !success);
        nChars = 1;
    }
    return nChars;
}

#define LINE_BUFFER_SIZE 200
void process_input(const char *input, size_t size) {
    static char line_buffer[LINE_BUFFER_SIZE + 1] = {0};  // +1 for null terminator
    static int buffer_index = 0;  // Tracks the current position in the buffer

    for (size_t i = 0; i < size; i++) {
        if (input[i] == '\n') {
            // Print the buffer if we encounter a newline and reset the buffer
            line_buffer[buffer_index] = '\0';  // Null-terminate the buffer
            if (0 == strncmp("od 100", line_buffer, strlen("od 100"))
                || 0 == strncmp("MAC:", line_buffer, strlen("MAC:"))
               ) {
                buffer_index = 0;
                return;
            }
            if(0 == buffer_index) {
                // empty line
                return;
            }
            bool success = false;
            do
            {
                success = UART3_Write(line_buffer, buffer_index);
            }while( !success);
            
            success = false;
            do
            {
                success = UART3_Write("\n", 1);
            }while( !success);
            buffer_index = 0;  // Reset the buffer index
        } else if (input[i] == '\r') {
            ; // ignore it and do not put it into the line buffer. It is a bug to have \r in the output.
        } else {
            if (buffer_index < LINE_BUFFER_SIZE) {
                // Add character to buffer if there is space
                line_buffer[buffer_index++] = input[i];
            }
            // If buffer is full, continue ignoring characters until newline is encountered
        }
    }
}

#include <string.h>
int write(int handle, void * buffer, size_t count)
{
   if (handle == 1)
   {
#if 1
        process_input((const char *) buffer, count);
#else
       bool success = false;
       do
       {
           success = UART3_Write(buffer, count);
       }while( !success);
#endif
   }
   return (int)count;
}