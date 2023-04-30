/*
 * menu.c
 *
 *  Created on: Jan 18, 2020
 *      Author: stimey
 */

#include "menu.h"
#include "oled.h"
#include "sw_quadrature.h"
#include "gpio.h"

void display_menu(TMENU_ENTRY *menu, FontDef Font, unsigned short sel_idx)
{
	// FontDef - font to use on the display
	// sel_idx = currently selected item
	// determine # of lines on the screen
	// automatically adjust first menu line to keep cursor on screen but favor keeping the selection in the center

	unsigned int item_count, idx, disp_line_count, disp_top, upper_half_line_count, lower_half_line_count;
	char * disp_line_text;
	MENU_GET_STRING *get_line_string;

	OLED_COLOR line_color;

	OLED_Fill(Black);

	disp_line_count = OLED_HEIGHT / Font.FontHeight;
	upper_half_line_count = disp_line_count / 2;
	lower_half_line_count = disp_line_count - upper_half_line_count;

	// figure out how many items are in the list
	for(item_count=0; (menu[item_count].menu_action_type!=MENU_ITEM_LIST_END) && (item_count<100); item_count++) /* NOTHING */ ;

	// determine starting line #
	if ((item_count <= disp_line_count ) || 	// if all menu items fit on the screen or
			(sel_idx <= upper_half_line_count)) 	// sel_idx < 1/2 the lines on the screen
		disp_top=0;
	else if (sel_idx < (item_count-lower_half_line_count))
		disp_top=sel_idx - upper_half_line_count;
	else
		disp_top=item_count-disp_line_count;

	for(idx=0; (idx < disp_line_count) && (idx < item_count); idx++)
	{
		if ((idx+disp_top)==sel_idx) line_color=Black;
		else						 line_color=White;

		// render the line item
		switch(menu[idx+disp_top].menu_display_type )
		{
			case MENU_DISP_STRING:
				OLED_WriteString((char *)(menu[idx+disp_top].menu_string_or_proc), Font, line_color);
				break;
			case MENU_DISP_FUNC_CALL:
				// ISO C Hack - declare the function pointer as a void to assign the address - found by google foo
				*(void**)(&get_line_string) = menu[idx+disp_top].menu_string_or_proc;

				disp_line_text = get_line_string((void*)(&menu[idx+disp_top]));

				if (disp_line_text != NULL)
					OLED_WriteString(disp_line_text, Font, line_color);
				break;
			case MENU_DISP_LIST_END:
				// do nothing - end of list
				break;
		}
	}

	OLED_UpdateScreen();
}

void do_menu(TMENU_ENTRY *menu, FontDef Font)
{
	unsigned short sel_idx;
	unsigned short exit_requested = 0;
	int starting_quad_value;

	sel_idx = 0;

	while( !exit_requested)
	{
		display_menu(menu, Font, sel_idx);

		starting_quad_value = quad_get_position();

		while(!quad_ok_pressed())
		{

		}

	};
}
