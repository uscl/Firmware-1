#include "serial_com_parser.h"
#include <string.h>
#include <stdlib.h>

#ifdef SERIAL_COM_DEBUG
#include <stdio.h>

const char *parser_state[] = {
	"0_UNSYNC",
	"1_SYNC",
	"2_GOT_DIGIT0",
	"3_GOT_DOT",
	"4_GOT_DIGIT1",
	"5_GOT_DIGIT2",
	"6_GOT_CARRIAGE_RETURN"
};
#endif

int serial_com_parser(char c, char *parserbuf, unsigned *parserbuf_index, enum SERIAL_COM_PARSE_STATE *state, float *dist)
{
	int ret = -1;
	char *end;

	switch (*state) {
	case SERIAL_COM_PARSE_STATE0_UNSYNC:
		if (c == '\n') {
			*state = SERIAL_COM_PARSE_STATE1_SYNC;
			(*parserbuf_index) = 0;
		}

		break;

	case SERIAL_COM_PARSE_STATE1_SYNC:
		if (c >= '0' && c <= '9') {
			*state = SERIAL_COM_PARSE_STATE2_GOT_DIGIT0;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;
		}

		break;

	case SERIAL_COM_PARSE_STATE2_GOT_DIGIT0:
		if (c >= '0' && c <= '9') {
			*state = SERIAL_COM_PARSE_STATE2_GOT_DIGIT0;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else if (c == '.') {
			*state = SERIAL_COM_PARSE_STATE3_GOT_DOT;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = SERIAL_COM_PARSE_STATE0_UNSYNC;
		}

		break;

	case SERIAL_COM_PARSE_STATE3_GOT_DOT:
		if (c >= '0' && c <= '9') {
			*state = SERIAL_COM_PARSE_STATE4_GOT_DIGIT1;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = SERIAL_COM_PARSE_STATE0_UNSYNC;
		}

		break;

	case SERIAL_COM_PARSE_STATE4_GOT_DIGIT1:
		if (c >= '0' && c <= '9') {
			*state = SERIAL_COM_PARSE_STATE5_GOT_DIGIT2;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = SERIAL_COM_PARSE_STATE0_UNSYNC;
		}

		break;

	case SERIAL_COM_PARSE_STATE5_GOT_DIGIT2:
		if (c == '\r') {
			*state = SERIAL_COM_PARSE_STATE6_GOT_CARRIAGE_RETURN;

		} else {
			*state = SERIAL_COM_PARSE_STATE0_UNSYNC;
		}

		break;

	case SERIAL_COM_PARSE_STATE6_GOT_CARRIAGE_RETURN:
		if (c == '\n') {
			parserbuf[*parserbuf_index] = '\0';
			*dist = strtod(parserbuf, &end);
			*state = SERIAL_COM_PARSE_STATE1_SYNC;
			*parserbuf_index = 0;
			ret = 0;

		} else {
			*state = SERIAL_COM_PARSE_STATE0_UNSYNC;
		}

		break;
	}

#ifdef SERIAL_COM_DEBUG
	printf("state: SERIAL_COM_PARSE_STATE%s\n", parser_state[*state]);
#endif

	return ret;
}
