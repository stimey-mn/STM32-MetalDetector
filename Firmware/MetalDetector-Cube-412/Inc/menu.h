/*
 * menu.h
 *
 *  Created on: Jan 18, 2020
 *      Author: stimey
 */

#ifndef MENU_H_
#define MENU_H_

// basic menu system for the OLED display and quadrature rotary knob (with center button)
#include "oled.h"

typedef enum {
	MENU_ITEM_LIST_END = 0,		// end of list
	MENU_ITEM_SUBMENU = 1,		// selection calls a sub-menu pointed to by data pointer
	MENU_ITEM_PROC_CALL = 2,	// selection calls a function with MENU_ITEM as it's pointer
	MENU_ITEM_RETURN = 3		// menu item is a return to upper menu / exit from menu system
} TMENU_ACTION_TYPE ;

typedef enum {
	MENU_DISP_LIST_END = 0,		// end of list
	MENU_DISP_STRING = 1,		// data pointer points to a static string
	MENU_DISP_FUNC_CALL= 2		// data pointer points to a function that returns a string
} TMENU_DISP_TYPE;


typedef struct menu_entry {
	TMENU_ACTION_TYPE menu_action_type;
	TMENU_DISP_TYPE menu_display_type;

	void *menu_string_or_proc;
	void *submenu_or_select_proc;
/*

 	union {
		char *menu_static_string;
		MENU_GET_STRING menu_string_proc;
	};

	union {
		// if item_type is PROC_CALL
		TMENU_CALL menu_entry;
		// if item_tyep is SUBMENU
		TMENU_ENTRY *menu;
	};
*/

} TMENU_ENTRY;

typedef int (*MENU_CALL)(TMENU_ENTRY *menu_entry);
typedef char *(MENU_GET_STRING)(TMENU_ENTRY *menu_entry);

void do_menu(TMENU_ENTRY *menu, FontDef Font);

#endif /* MENU_H_ */
