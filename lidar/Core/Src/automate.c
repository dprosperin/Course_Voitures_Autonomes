/*
 * automate.c
 *
 *  Created on: Nov 14, 2024
 *      Author: davidprosperin
 */
//#include <automate.h>

/*
void automate(void)
{
	static state_automate_t next_state = FLAG_START1;
	static state_automate_t current_state;

	current_state = next_state;

	switch (current_state)
	{
	case FLAG_START1 :
			if (read_byte_from_buffer() == 0xA5)
			{
				next_state = FLAG_START2;
			}
	break;

	case FLAG_START2 :
		if (read_byte_from_buffer() == 0x5A)
		{
			next_state = DATA_AND_SEND_MODE;
		}

	break;
}*/
