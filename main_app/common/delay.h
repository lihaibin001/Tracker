#ifndef  _DELAY_H_
#define  _DELAY_H_ 
/**********************************************************************
   Title                    : DELAY.H

   Module Description       : This file has all of the standard global defines
                              for module DELAY

   Author                   : 

   Created                  : 

   Configuration ID         : 

 *********************************************************************/

/*---------------------------------------------------------------------
 *   Instructions for using this module if any:
 *
 *-------------------------------------------------------------------*/

/*********************************************************************/
/* Include files                                                     */
/*********************************************************************/

/*********************************************************************/
/* Constant and Macro Definitions using #define                      */
/*********************************************************************/

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/

/*********************************************************************/
/* Global Variable extern Declarations                               */
/*********************************************************************/

/*********************************************************************/
/* Function Prototypes                                               */
/*********************************************************************/

/**********************************************************************
 *    Function: uDelay
 *  Parameters: number of uSec to delay
 *     Returns: void
 * Description: This routine will DELAY at least the amount of time in uSec
 *              passes in as the DELAY time
 *********************************************************************/
extern void uDelay(uint32_t usec);

/**********************************************************************
 *
 * REVISION RECORDS
 *
 *********************************************************************/
/*********************************************************************/
/*
 *
 *********************************************************************/
#endif //_DELAY_H_