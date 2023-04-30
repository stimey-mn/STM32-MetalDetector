#ifndef __MAIN_H
#define __MAIN_H


#ifndef F_CPU

#ifdef STM32F303X8
#define F_CPU 8000000
#else
#define F_CPU 48000000	// MSI running at 48 MHz
#endif

#endif


#endif /* __MAIN_H */
