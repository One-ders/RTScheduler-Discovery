/* $RT dispatch: , v1.1 2014/04/07 21:44:00 anders Exp $ */

/*
 * Copyright (c) 2014, Anders Franzen.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @(#)io.c
 */
#include <stdarg.h>
#include <string.h>
#include <assert.h>
#include <io.h>
#include <sys.h>

#define TICSPERHOUR	(100*60*60)
#define TICSPERMIN	(100*60)
#define TICSPERSEC	(100)

//#define CHECK_BOUNDS(needed_space) ((p + (needed_space)) > ep)?1:0
#define SPACE_AVAIL() ((ep-p)>0?(ep-p):0)
#define MIN(a,b) ((a<b)?a:b)


char *ts_format(unsigned int tics, char *buf, int bsize) {
	unsigned int h=tics/TICSPERHOUR;
	unsigned int min=(tics-(h*TICSPERHOUR))/TICSPERMIN;
	unsigned int sec=(tics-((h*TICSPERHOUR)+(min*TICSPERMIN)))/TICSPERSEC;
	unsigned int msec=(tics-((h*TICSPERHOUR)+(min*TICSPERMIN)+(sec*TICSPERSEC)))*10;

	sprintf(buf,"%d:%02d:%02d.%03d",h,min,sec,msec);
	return buf;
}

int parse_fmt(const char *fmt, int *field_width, int *zero_fill) {
	int i=0;
	int val=0;
	int ladj=0;
	if ((*fmt)=='0') {
		*zero_fill=1;
		i++;
	} else if ((*fmt)=='-') {
		ladj=1;
		i++;
	}

	if ((fmt[i]>='1')&&(fmt[i]<='9')) {
		val=fmt[i]-'0';
		i++;
		while ((fmt[i]>='0') && (fmt[i]<='9')) {
			val=val*10;
			val+=(fmt[i]-'0');
			i++;
		}
	}
	if (ladj) {
		*field_width=-val;
	} else {
		*field_width=val;
	}
	if (i>3) while(1) {;}
	if (i<0) while(1) {;}
	return i;
}

int vsnprintf(char *sbuf, size_t sbuf_size, const char *fmt, va_list ap) {
	int i=0;
	char numericbuf[16];
	char *p=sbuf;
	char *ep=sbuf+sbuf_size;

	while(1) {
		int c=fmt[i++];
		if (!c) break;
		if (c=='%') {
			int prepend_zero=0, prepend_num=0;
			i+=parse_fmt(&fmt[i],&prepend_num, &prepend_zero);
			switch((c=fmt[i++])) {
				case 's': {
					char *s=va_arg(ap,char *);
					int len=strlen(s);
					if (prepend_num>0) {
						if ((prepend_num-len)>0) {
							int j;
							for(j=0;j<(prepend_num-len);j++) {
								memcpy(p," ",MIN(1,SPACE_AVAIL()));
								p++;
							}
						}
					}
					memcpy(p,s,MIN(len,SPACE_AVAIL()));
					p+=len;

					if (prepend_num<0) {
						prepend_num=-prepend_num;
						if ((prepend_num-len)>0) {
							int j;
							for(j=0;j<(prepend_num-len);j++) {
								memcpy(p," ",MIN(1,SPACE_AVAIL()));
								p++;
							}
						}
					}
					break;
				}
				case 'c': {
					int gruuk=va_arg(ap, int);
					memcpy(p,(char *)&gruuk,MIN(1,SPACE_AVAIL()));
					p++;
					break;
				}
				case 'd': {
					char *s;
					if (prepend_num<0) prepend_num=0;
					s=itoa(va_arg(ap,unsigned int),numericbuf, 16,
								prepend_zero, prepend_num);
					int len=strlen(s);
					memcpy(p,s,MIN(len,SPACE_AVAIL()));
					p+=len;
					break;
				}
				case 'x': {
					char *s;
					if (prepend_num<0) prepend_num=0;
					s=xtoa(va_arg(ap,unsigned int),numericbuf, 16,
							prepend_zero, prepend_num);
					int len=strlen(s);
					memcpy(p,s,MIN(len,SPACE_AVAIL()));
					p+=len;
					break;
				}
				case 't': {
					char *s=ts_format(get_current_tic(),numericbuf, 16);
					int len=strlen(s);
					memcpy(p,s,MIN(len,SPACE_AVAIL()));
					p+=len;
					break;
				}
				case '%': {
					memcpy(p,"%",MIN(1,SPACE_AVAIL()));
					p++;
					break;
				}
				default: {
					memcpy(p,"%",MIN(1,SPACE_AVAIL()));
					p++;
					memcpy(p,(char *)&c,MIN(1,SPACE_AVAIL()));
					p++;
				}
			}
		} else {
			memcpy(p,(char *)&c,MIN(1,SPACE_AVAIL()));
			p++;
		}
	}

	if (SPACE_AVAIL()) {
		*p=0;
	} else {
		if (ep-p) {
			*(--ep)=0;
		}
	}
	return p-sbuf;
}

int dprintf(int fd, const char *fmt, ...) {
	int i=0;
	va_list ap;
	char numericbuf[16];
	int tlen=0;

	va_start(ap,fmt);
	while(1) {
		int c=fmt[i++];
		if (!c) break;
		else if (c=='\n') {
			io_write(fd,"\n",1);
			tlen++;
			continue;
		} else if (c=='%') {
			int prepend_zero=0, prepend_num=0;
			i+=parse_fmt(&fmt[i],&prepend_num, &prepend_zero);
			switch((c=fmt[i++])) {
				case 's': {
					char *s=va_arg(ap,char *);
					int len=strlen(s);
					if (prepend_num>0) {
						if ((prepend_num-len)>0) {
							int j;
							for(j=0;j<(prepend_num-len);j++) {
								io_write(fd," ",1);
								tlen++;
							}
						}
					}
					io_write(fd,s,len);
					tlen+=len;
					if (prepend_num<0) {
						prepend_num=-prepend_num;
						if ((prepend_num-len)>0) {
							int j;
							for(j=0;j<(prepend_num-len);j++) {
								io_write(fd," ",1);
								tlen++;
							}
						}
					}
					break;
				}
				case 'c': {
					int gruuk=va_arg(ap, int);
					io_write(fd,(char *)&gruuk,1);
					tlen++;
					break;
				}
				case 'd': {
					char *s;

					if (prepend_num<0) prepend_num=0;
					s=itoa(va_arg(ap,unsigned int),numericbuf, 16,
								prepend_zero, prepend_num);
					io_write(fd,s,strlen(s));
					tlen+=strlen(s);
					break;
				}
				case 'x': {
					char *s;

					if (prepend_num<0) prepend_num=0;
					s=xtoa(va_arg(ap,unsigned int),numericbuf, 16,
							prepend_zero, prepend_num);
					io_write(fd,s,strlen(s));
					tlen+=strlen(s);
					break;
				}
				case 't': {
					char *s=ts_format(get_current_tic(),numericbuf, 16);
					io_write(fd,s,strlen(s));
					tlen+=strlen(s);
					break;
				}
				case '%': {
					io_write(fd,"%",1);
					tlen++;
					break;
				}
				default: {
					io_write(fd,"%",1);
					io_write(fd,(char *)&c,1);
					tlen+=2;
				}
			}
		} else {
			io_write(fd,(char *)&c,1);
			tlen++;
		}

	}
	va_end(ap);
	return tlen;
}

int sprintf(char *sbuf, const char *fmt, ...) {
	va_list ap;
	int ret;

	va_start(ap, fmt);
	ret = vsnprintf(sbuf, (size_t)__SIZE_MAX__/2, fmt, ap);
	va_end(ap);

	return ret;
}

#define SET_CURSOR(BUF,X,Y) {BUF[0]=0x1b;BUF[1]='[';BUF[2]=X;BUF[3]=';';BUF[4]=Y;BUF[5]='H';BUF[6]=0;}
#define FORW_CURSOR(BUF,X)	{BUF[0]=0x1b;BUF[1]='[';BUF[2]=X;BUF[3]='C';BUF[4]=0;}
#define BACK_CURSOR(BUF,X)	{BUF[0]=0x1b;BUF[1]='[';BUF[2]=X;BUF[3]='D';BUF[4]=0;}
#define UP_CURSOR(BUF,Y)	{BUF[0]=0x1b;BUF[1]='[';BUF[2]=Y;BUF[3]='A';BUF[4]=0;}
#define DOWN_CURSOR(BUF,Y)	{BUF[0]=0x1b;BUF[1]='[';BUF[2]=Y;BUF[3]='B';BUF[4]=0;}
#define GET_CURSOR(BUF)		{BUF[0]=0x1b;BUF[1]='[';BUF[2]='6';BUF[3]='n';BUF[4]=0;}

static int get_cursor(int fd,unsigned int *x, unsigned int *y) {
	char buf[10];
	int row=0;
	int col=0;

	GET_CURSOR(buf);
	io_write(fd,buf,strlen(buf));
	io_read(fd,buf,2);
	while(1) {
		int rc=io_read(fd,buf,1);
		if (rc!=1) return -1;
		if (buf[0]==';') break;
		row=(row*10)+(buf[0]-'0');
	}
	while(1) {
		int rc=io_read(fd,buf,1);
		if (rc!=1) return -1;
		if (buf[0]=='R') break;
		col=(col*10)+(buf[0]-'0');
	}
	printf("x=%d,y=%d\n",col,row);
	return 0;
}

static int forw_cursor(int fd) {
	char buf[10];
	FORW_CURSOR(buf,1+'0');
	io_write(fd,buf,strlen(buf));
	return 0;
}

static int back_cursor(int fd) {
	char buf[10];
	BACK_CURSOR(buf,1+'0');
	io_write(fd,buf,strlen(buf));
	return 0;
}


#define STATE_C_NORMAL 0
#define STATE_C_MULTI  1
#define STATE_C_ARROW  2

#define HISTORY_SIZE_SHIFT 3            // History size is 'power of two'
#define HISTORY_SIZE (1<<HISTORY_SIZE_SHIFT)
#define CMD_LEN 80
static char history[HISTORY_SIZE][CMD_LEN];
static int h_ix;

int readline_r(int fd, char *prompt, char *buf, int buf_size) {
	int bix=0;
	int bix_max=0;
	int tmp_h_ix=h_ix;
	int state=STATE_C_NORMAL;
	char tmpbuf[CMD_LEN];
	int tmp_cmdlen=0;
	int tmp_cursor_pos=0;


	dprintf(fd,prompt);

	while (1) {
		int ch=0;
		int rc=io_read(fd,(char *)&ch,1);
		if (rc!=1) return rc;
		if (ch>127) continue;
		if (state==STATE_C_MULTI) goto handle_special;
		if (state==STATE_C_ARROW) goto handle_arrow;
		switch(ch) {
			case 0x08: case 0x7f: {
				char ochar=' ';
				int i;
				/* backspace */
				if (tmp_h_ix!=h_ix) {
					strcpy(tmpbuf,history[tmp_h_ix&(HISTORY_SIZE-1)]);
//(tmp_h_ix,tmpbuf,&bix_max);
					tmp_h_ix=h_ix;
//					printf("switchbuf, bix=%d\n",bix);
				}
				if (!bix)  break;
				bix--;
				bix_max--;
				back_cursor(fd);
				for(i=bix;i<bix_max;i++) {
					tmpbuf[i]=tmpbuf[i+1];
					io_write(fd,&tmpbuf[i],1);
				}
				tmpbuf[bix_max]=0;
				io_write(fd,&ochar,1);
				for (i=bix;i<bix_max;i++)
					back_cursor(fd);
				back_cursor(fd);
				break;
			}
			case 0xd: // enter
				if (bix_max>bix) {
					int i;
					for(i=bix;i<bix_max;i++) {
						forw_cursor(fd);
					}
					bix=bix_max;
				}
				tmpbuf[bix]=0;
				if ((bix<80)&&(bix>0)&&
					(h_ix==tmp_h_ix)&&
					__builtin_strcmp(tmpbuf,history[(h_ix-1)&(HISTORY_SIZE-1)])) {
					__builtin_strcpy(history[h_ix&7],tmpbuf);
					history[h_ix&(HISTORY_SIZE-1)][bix]=0;
					h_ix++;
				}
				strcpy(buf,tmpbuf);
				return bix;
			case 0x1b:  /* 1b */
				state=STATE_C_MULTI;
				break;
			default: {
				int ins=0;
				if (tmp_h_ix!=h_ix) {
					strcpy(tmpbuf,history[tmp_h_ix&(HISTORY_SIZE-1)]);
					tmp_h_ix=h_ix;
				}

				if (bix<bix_max) {
					__builtin_memmove(&tmpbuf[bix+1],
								&tmpbuf[bix],
								bix_max-bix);
					bix_max++;
					ins=1;
				}

				tmpbuf[bix++]=ch;
				if (bix>=buf_size) {
					tmpbuf[buf_size-1]=0;
					__builtin_strcpy(buf,tmpbuf);
					return buf_size-1;
				}
				if (bix>bix_max) {
					bix_max=bix;
					tmpbuf[bix_max]=0;
				}
				io_write(fd,(char *)&ch,1);
				if (ins) {
					int i;
					io_write(fd,&tmpbuf[bix],bix_max-bix);
					for(i=0;i<bix_max-bix;i++) {
						back_cursor(fd);
					}
				}
			}
		}
		goto next;

handle_special:
		switch(ch) {
			case 0x5b:
				state=STATE_C_ARROW;
				break;
			default:
				printf("unknown control %x\n", ch);
				state=STATE_C_NORMAL;
		}
		goto next;

handle_arrow:
		switch(ch) {
			case 0x41: {/* UP arrow */
				char *hline_ptr;
				char space=' ';
				if ((tmp_h_ix>0) &&
					((tmp_h_ix-1)>(h_ix-(HISTORY_SIZE)))) {
					tmp_h_ix--;

					if (bix_max>bix) { // move cursor to one after the last char
						int i;
						for(i=bix;i<bix_max;i++) {
							forw_cursor(fd);
						}
						bix=bix_max;
					}
					while(bix_max) { // erase the line
						back_cursor(fd);
						io_write(fd,&space,1);
						back_cursor(fd);
						bix_max--;
					}
//					__builtin_strcpy(buf,history[tmp_h_ix&(HISTORY_SIZE-1)]);
					hline_ptr=history[tmp_h_ix&(HISTORY_SIZE-1)];
					bix=strlen(hline_ptr);
//					buf[bix]=0;
					bix_max=bix;
					io_write(fd,hline_ptr,bix);
				}
				state=STATE_C_NORMAL;
				break;
			}
			case 0x42: {/* Down arrow */
				char *hline_ptr;
				char space=' ';
				if (tmp_h_ix<h_ix) {
					tmp_h_ix++;
					if (bix_max>bix) { // possition curson at the end of the line
						int i;
						for(i=bix;i<bix_max;i++) {
							forw_cursor(fd);
						}
						bix=bix_max;
					}
					while(bix_max) { // erase the line
						back_cursor(fd);
						io_write(fd,&space,1);
						back_cursor(fd);
						bix_max--;
					}
					if (tmp_h_ix<h_ix) {
						hline_ptr=history[tmp_h_ix&(HISTORY_SIZE-1)];
						bix=strlen(hline_ptr);
						bix_max=bix;
						io_write(fd,hline_ptr,bix);
					} else {
						bix=strlen(tmpbuf);
						bix_max=bix;
						io_write(fd,tmpbuf,bix);
					}
				}

				state=STATE_C_NORMAL;
				break;
			}
			case 0x43: /* Right arrow */
				if (bix<bix_max) {
					bix++;
					forw_cursor(fd);
				}
				state=STATE_C_NORMAL;
				break;
			case 0x44: /* Left arrow */
				if (bix>0) {
					bix--;
					back_cursor(fd);
				}
				state=STATE_C_NORMAL;
				break;
			default:
				printf("unknown arrow %x\n", ch);
				state=STATE_C_NORMAL;

		}
next:;
	}
	return 0;
}
