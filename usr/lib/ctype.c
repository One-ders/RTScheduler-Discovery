
#include <ctype.h>

int tolower(int c) {
	if ((c>='A') && (c<='Z')) {
		return (c-'A')+'a';
	}
	return c;
}

int toupper(int c) {
	if ((c>='a') && (c<='z')) {
		return (c-'a')+'A';
	}
	return c;
}

#if 0
int isspace(int c) {
	switch (c) {
		case ' ':
		case '\f':
		case '\n':
		case '\r':
		case '\t':
		case '\v':
			return 1;
			break;
		default:
			break;
	}
	return 0;
}
#endif
