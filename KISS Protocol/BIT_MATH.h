/************************************************/
/* Author      : Ahmed Fathy Abd El-Qadir	  */
/* Last Update : 24-12-2021                     */
/* Version     : V 1.0                          */
/************************************************/

#ifndef BIT_MATH_H
#define BIT_MATH_H

#define SET_BIT(VAR, BIT)  VAR|= (1 << (BIT))
#define CLR_BIT(VAR, BIT)  VAR&=~(1 << (BIT))
#define TOG_BIT(VAR, BIT)  VAR^= (1 << (BIT))
#define GET_BIT(VAR, BIT)  ((VAR>>BIT) & 1)

#endif
