/* $RTOs: , v1.1 2014/04/07 21:44:00 anders Exp $ */

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
 * @(#)sys_cmd.c
 */

#include <sys.h>
#include <string.h>
#include <io.h>
#include <sys_env.h>
#include <procps.h>
#include <devls.h>
#include <pwr_mgr_drv.h>

#include <syslog.h>

struct dents {
	char name[32];
};

#define READDIR 10
#define DYNOPEN 11

static int isprint(int c) {
	return (unsigned)c-0x20 < 0x5f;
}


static char toNum(int state) {
        switch(state) {
                case TASK_STATE_IDLE: return 'i';
                case TASK_STATE_RUNNING: return 'r';
                case TASK_STATE_READY: return 'w';
                case TASK_STATE_TIMER: return 't';
                case TASK_STATE_IO: return 'b';
                default: return '?';
        }
}

static int get_procdata(int fd, char *name, struct Env *env) {
	struct procdata pd;
	int rc;
	int fd2;

	if ((fd2=io_control(fd, DYNOPEN, name, 0))<0) {
		fprintf(env->io_fd, "could not open proc data\n");
		return -1;
	}

	rc=io_read(fd2,&pd,sizeof(pd));
	if(rc>0) {
		fprintf(env->io_fd, "task(%3d@%x) %12s, sp=0x%08x, pc=0x%08x, prio=%x, state=%c, atics=%d\n",
		pd.id, pd.addr, pd.name, pd.sp, pd.pc, pd.prio_flags, toNum(pd.state),pd.active_tics);
	}
	io_close(fd2);
	return 0;
}

static int ps_fnc(int argc, char **argv, struct Env *env) {
	int fd=io_open("procps");
	struct dents dents[10];
	int rc,i=0;
	int tic;

	if (fd<0) {
		fprintf(env->io_fd,"could not open ps driver\n");
		return -1;
	}

	rc=io_control(fd,READDIR,dents,10);

	if (rc<0) {
		fprintf(env->io_fd,"directory error\n");
		return 0;
	}

	tic=get_current_tic();
	fprintf(env->io_fd, "uptime: %t, current tic: %d\n", tic);
	while(i<rc) {
		get_procdata(fd,dents[i].name,env);
		i++;
	}
	io_close(fd);
	return 0;
}

static int get_devdata(int fd, char *name, struct Env *env) {
	struct devdata dd;
	int rc;
	int fd2;
	if ((fd2=io_control(fd, DYNOPEN, name, 0))<0) {
		fprintf(env->io_fd, "could not open dev data\n");
		return -1;
	}

	rc=io_read(fd2,&dd,sizeof(dd));
	if(rc>0) {
		int i;
		fprintf(env->io_fd, "%12s: ", name);
		for(i=0;i<dd.numofpids;i++) {
			if (i) {
				fprintf(env->io_fd,", %d",dd.pid[i]);
			} else {
				fprintf(env->io_fd,"%d",dd.pid[i]);
			}
		}
		fprintf(env->io_fd,"\n");
	}
	io_close(fd2);
	return 0;
}


static int lsdrv_fnc(int argc, char **argv, struct Env *env) {
	int fd=io_open("dev");
//	struct devdata d;
	struct dents dents[32];
	int i,rc;
	if (fd<0) {
		fprintf(env->io_fd,"could not open dev\n");
		return -1;
	}
	rc=io_control(fd,READDIR,dents,128);
	fprintf(env->io_fd,"=========== Installed drivers =============\n");
	for(i=0;i<rc;i++) {
		get_devdata(fd,dents[i].name,env);
        }
        fprintf(env->io_fd,"========= End Installed drivers ===========\n");
	io_close(fd);
        return 0;
}

static int kmem_fnc(int argc, char **argv, struct Env *env) {
	int fd=io_open("kmem");
	int rc=0;
	int r=0,w=0;
	int b_mode=0;
	struct getopt_data gd;
	char *nptr;
	int opt;
	unsigned long int address=0;

	if (fd<0) {
		fprintf(env->io_fd,"could not open dev kmem\n");
		return -1;
	}

	getopt_data_init(&gd);

	while((opt=getopt_r(argc,argv,"r:w:b",&gd))!=-1) {
		switch(opt) {
			case 'r':
				nptr=gd.optarg;
				address=strtoul(gd.optarg,&nptr,0);
				if (nptr==gd.optarg) {
					fprintf(env->io_fd, "address value %s, not a number\n",gd.optarg);
					rc = -1;
					goto out;
				}
				r=1;
				fprintf(env->io_fd,"read addr %x\n",address);
				break;
			case 'w':
				nptr=gd.optarg;
				address=strtoul(gd.optarg,&nptr,0);
				if (nptr==gd.optarg) {
					fprintf(env->io_fd, "address value %s, not a number\n",gd.optarg);
					rc = -1;
					goto out;
				}
				w=1;
				fprintf(env->io_fd,"write addr %x\n",address);
				break;
			case 'b':
				b_mode=1;
				break;
			default: {
				fprintf(env->io_fd, "kmem arg error %c, use -r addr len, or -w addr val len\n",opt);
				rc=-1;
				goto out;
			}
		}
	}


	if (r) {  // read command
		unsigned int nbytes=16;
		int i=0,j;
		unsigned int nwords;

		if (gd.optind<argc) {
			nptr=argv[gd.optind];
			nbytes=strtoul(argv[gd.optind],&nptr,0);
			if (nptr==argv[gd.optind]) {
				fprintf(env->io_fd, "nbytes value %s, not a number\n",argv[gd.optind]);
				rc = -1;
				goto out;
			}
		}

		rc=io_lseek(fd,address,SEEK_SET);
		if (rc==-1) {
			goto out;
		}

		if (b_mode) {
			while(i<nbytes) {
				char cstr[]="*________________*";
				fprintf(env->io_fd,"%08x:\t",address+i);
				for(j=0;(j<16)&&i<nbytes;i++,j++) {
					unsigned char vbuf;
					rc=io_read(fd,&vbuf,1);
					if (rc<0) {
						rc=-11;
						goto out;
					}
					fprintf(env->io_fd,"%02x ", vbuf);
					cstr[j+1]=isprint(vbuf)?vbuf:'_';
				}
				fprintf(env->io_fd,"%s\n",cstr);
			}
		} else {
			nwords=((nbytes+3)>>2);
			while(i<nwords) {
				fprintf(env->io_fd,"%08x:\t",address+(i<<2));
				for(j=0;(j<4)&&i<nwords;i++,j++) {
					unsigned int vbuf;
					rc=io_read(fd,&vbuf,4);
					if (rc<0) {
						rc=-11;
						goto out;
					}
					fprintf(env->io_fd,"%08x ", vbuf);
				}
				fprintf(env->io_fd,"\n");
			}
		}
	} else if (w) {
		unsigned long int value;

		if (gd.optind<argc) {
			nptr=argv[gd.optind];
			value=strtoul(argv[gd.optind],&nptr,0);
			if (nptr==argv[gd.optind]) {
				fprintf(env->io_fd, "value %s, not a number\n",argv[gd.optind]);
				rc = -1;
				goto out;
			}
		}

		rc=io_lseek(fd,address,SEEK_SET);
		if (rc==-1) {
			goto out;
		}

		fprintf(env->io_fd,"%08x:\t=%08x",address,value);
		rc=io_write(fd,&value,4);
		if (rc<0) {
			rc=-1;
			goto out;
		}
	} else {
		fprintf(env->io_fd, "kmem: unknown operation\n");
	}
out:
	io_close(fd);
	return rc;
}

static int debug_fnc(int argc, char **argv, struct Env *env) {
	int dbglev;
	if (argc>1) {
		if (__builtin_strcmp(argv[1],"on")==0) {
			dbglev=10;
		} else if (__builtin_strcmp(argv[1],"off")==0) {
			dbglev=0;
		} else {
			fprintf(env->io_fd,"debug <on> | <off>\n");
			return 0;
		}
	} else {
		dbglev=0;
	}
	set_debug_level(dbglev);
	return 0;
}

static int block_fnc(int argc, char **argv, struct Env *env) {
	int rc;
	fprintf(env->io_fd, "blocking %s, ", argv[1]);
	rc=block_task(argv[1]);
	fprintf(env->io_fd, "returned %d\n", rc);

	return 0;
}

static int unblock_fnc(int argc, char **argv, struct Env *env) {
	int rc;
	fprintf(env->io_fd, "unblocking %s, ", argv[1]);
	rc=unblock_task(argv[1]);
	fprintf(env->io_fd, "returned %d\n", rc);

	return 0;
}

static int setprio_fnc(int argc, char **argv, struct Env *env) {
	int rc;
	int prio=strtoul(argv[2],0,0);
	if (prio>MAX_PRIO) {
		fprintf(env->io_fd,"setprio: prio must be between 0-4\n");
		return 0;
	}
	fprintf(env->io_fd, "set prio of %s to %d", argv[1], prio);
	rc=setprio_task(argv[1],prio);
	fprintf(env->io_fd, "returned %d\n", rc);

	return 0;
}

static int reboot_fnc(int argc, char **argv, struct Env *env) {
	fprintf(env->io_fd, "rebooting\n\n\n");
	sleep(100);
	_reboot_(0x5a5aa5a5);
	return 0;
}

static int dump_log_fnc(int argc, char **argv, struct Env *env) {
	int fd=io_open("syslog");
	int rc=0;
	int ok=0;

	if (fd<0) {
		fprintf(env->io_fd,"could not open dev syslog\n");
		return -1;
	}

	ok=io_control(fd,DUMP_BUF,0,0);

	io_close(fd);

	if (ok<0) {
		fprintf(env->io_fd,"bad argument %s\n", argv[1]);
		return -1;
	}
	return 0;
}

#define MAX(a,b) (a<b)?b:a

static int cat_fnc(int argc, char **argv, struct Env *env) {
	int fd;
	int ok=0;
	int maxfd;
	char buf[256];

	buf[0]=0;

	if (argc!=2) {
		fprintf(env->io_fd,"need drv name as argument\n");
		return -1;
	}

	fd=io_open(argv[1]);
	if (fd<0) {
		fprintf(env->io_fd,"could not open dev %s\n",argv[1]);
		return -1;
	}

	maxfd=MAX(fd,env->io_fd);
	while(1) {
		int rc;
		fd_set rfds;

		FD_ZERO(&rfds);
		FD_SET(fd,&rfds);
		FD_SET(env->io_fd,&rfds);

		rc=io_select(maxfd+1,&rfds,0,0,0);
		if (rc<0) {
			fprintf(env->io_fd, "error from select: %d\n",rc);
			continue;
		}

//	 	fprintf(env->io_fd,"got %d from select\n",rc);
		if (FD_ISSET(env->io_fd,&rfds)) {
//	 		fprintf(env->io_fd,"user console io ready\n",rc);
			ok=io_read(env->io_fd,buf,1);
			if (ok) {
				if (buf[0]==0x03) {
					fprintf(fd, "Control C");
					break;
				}
			} else {
				break;
			}
		}

		if (FD_ISSET(fd,&rfds)) {
//	 		fprintf(env->io_fd,"catted dev io ready\n",rc);
			ok=io_read(fd,buf,1);
			if (ok) {
				if (buf[0]==0x04) {
					fprintf(env->io_fd, "Control D");
					break;
				}
				buf[ok]=0;
				if (buf[strlen(buf)-1]== '\n') {
					char *p=&buf[strlen(buf)];
					*p='\r';
					p++;
					*p=0;
					fprintf(env->io_fd, "got linefeed");
				}
				fprintf(env->io_fd, "%s",buf);
			} else {
				break;
			}
		}
	}

	io_close(fd);

	return 0;
}

static int echo_fnc(int argc, char **argv, struct Env *env) {
	int fd=env->io_fd;
	int i;

	if (argc<2) {
		fprintf(env->io_fd,"need some data\n");
		return -1;
	}

	for (i=1; i<argc; i++) {
		if (i+1!=argc) {
			fprintf(fd, "%s ",argv[i]);

		} else {
			fprintf(fd, "%s",argv[i]);
		}
	}

	if (fd!=env->io_fd) {
		io_close(fd);
	}

	return 0;
}

static int ememt_fnc(int argc, char **argv, struct Env *env) {
	int fd=env->io_fd;
	unsigned int i;
	unsigned int addr_start=0xd0000000;
	unsigned int addr_stop= 0xd0800000;

	for (i=addr_start; i<addr_stop; i+=4) {
		unsigned int *a=(unsigned int *)i;
		*a=i;
	}

	for (i=addr_start; i<addr_stop; i+=4) {
		unsigned int *a=(unsigned int *)i;
		if (*a!=i) {
			fprintf(fd,"test failed at %x\n",i);
			return -1;
		}
	}


	fprintf(fd,"A success\n");

	return 0;
}

#define HR_TIMER_SET 0x1001

static int hr_timer_test(int argc, char **argv, struct Env *env) {
	int fd=env->io_fd;
	int hrt;
	int rc=0;
	unsigned int tout=1000000;  // 1000000 uS -> 1 Sec
	int ttime=20; 			// run for 20 sec

	hrt=io_open("hr_timer");
	if (hrt<0) {
		fprintf(fd,"could not open hr timer");
		return 0;
	}

	rc=io_control(hrt, HR_TIMER_SET, &tout,sizeof(tout));
	if (rc<0) {
		fprintf(fd,"could request timeout");
		return 0;
	}

	while(1) {
		io_control(hrt, IO_POLL, (void *)EV_STATE, 0);
		ttime--;
		fprintf(fd, "1 sec timeout, time left %d\n", ttime);
		if (ttime) {
			rc=io_control(hrt, HR_TIMER_SET, &tout,sizeof(tout));
		} else {
			break;
		}
	}

	io_close(hrt);
	fprintf(fd,"Done\n");

	return 0;
}

static int sys_timer_test(int argc, char **argv, struct Env *env) {
	int rc=0;
	int ttime=20; 			// run for 20 sec
	int fd=env->io_fd;

	while(1) {
		sleep(1000);
		ttime--;
		fprintf(fd, "1 sec timeout, time left %d\n", ttime);
		if (!ttime) {
			break;
		}
	}

	fprintf(fd,"Done\n");
	return 0;
}

static int pwr_mgr_test(int argc, char **argv, struct Env *env) {
	int fd=env->io_fd;
	int rc=0;
	int pm_fd;

	if (argc<2) {
		fprintf(fd,"need some arguments\n");
		return -1;
	}

	pm_fd=io_open("pwr_mgr");
	if (pm_fd<0) {
		fprintf(fd, "failed to open pwr_mgr driver, f.ex. get or set\n");
		rc=-1;
		goto out1;
	}
	if (strcmp(argv[1],"get")==0) {
		unsigned int pm;
		unsigned int clk;
		if (io_control(pm_fd, GET_POWER_MODE, &pm, sizeof(pm))<0) {
			fprintf(fd, "get error back from GET_POWER_MODE\n");
			rc=-1;
			goto out;
		}
		if (io_control(pm_fd, GET_SYS_CLOCK, &clk, sizeof(clk))<0) {
			fprintf(fd, "get error back from GET_SYS_CLOCK\n");
			rc=-1;
			goto out;
		}

		fprintf(fd, "power mode control word=%x\n", pm);
		fprintf(fd, "CW bit 0 is %d, meaning %s\n", pm&1, (pm&1)?"high speed":"low speed");
		fprintf(fd, "CW bit 1&2 is %d, meaning %s\n", pm&3, (pm&2)?"idle Wait for irq":
									(pm&3)?"idle Wait for event":
									"idle spin");
		fprintf(fd, "CW bit 3 is %d, meaning %s\n", pm&8, (pm&8)?"Deepsleep":"lightSleep");
		fprintf(fd, "system clock is=%d Mhz\n", clk/(1000*1000));
	} else if (strcmp(argv[1],"set")==0) {
		unsigned int pm;
		if (argc<3) {
			fprintf(fd, "need a power control word as arg\n");
			rc=-1;
			goto out;
		}

		pm=strtoul(argv[2],0,0);
		if (io_control(pm_fd, SET_POWER_MODE, &pm, sizeof(pm))<0) {
			fprintf(fd, "got error for SET_POWER_MODE\n");
			rc=-1;
			goto out;
		}
	} else {
		fprintf(fd, "pwr_mgr: unknow sub command %s\n", argv[1]);
		rc=-1;
	}

out:
	io_close(pm_fd);
out1:

	if (fd!=env->io_fd) {
		io_close(fd);
	}

	return rc;
}




#if 0
extern int fb_test(void *);

static int testprog(int argc, char **argv, struct Env *env) {
	thread_create(fb_test,"groda",6,1,"fb_test");
	return 0;
}
#endif

static struct cmd cmd_root[] = {
		{"help", generic_help_fnc},
		{"ps", ps_fnc},
		{"lsdrv",lsdrv_fnc},
		{"debug", debug_fnc},
		{"block",block_fnc},
		{"unblock",unblock_fnc},
		{"setprio",setprio_fnc},
		{"reboot",reboot_fnc},
		{"kmem",kmem_fnc},
		{"dlog",dump_log_fnc},
		{"cat",cat_fnc},
		{"echo",echo_fnc},
		{"ext_mem_test",ememt_fnc},
		{"hr_timer",hr_timer_test},
		{"sys_timer",sys_timer_test},
		{"pwr_mgr",pwr_mgr_test},
		{0,0}
};

static struct cmd_node my_cmd_node = {
	"",
	cmd_root,
};

void main(void *dum) {
	char buf[256];
	int fd=io_open(dum);
	struct Env env;
	static int u_init=0;
	if (fd<0) {
		init_pkg();
		while(1) {
			sleep(1000000);
		}
		return;
	}

	env.io_fd=fd;
	fprintf(fd,"Starting sys_mon\n");

	if (!u_init) {
		u_init=1;
		install_cmd_node(&my_cmd_node,0);
#ifdef TEST_USB_SERIAL
		thread_create(main,"usb_serial0",12,1,"sys_mon:usb");
#endif
		init_pkg();
	}

	while(1) {
		int rc;
		rc=readline_r(fd,"\n--->",buf,200);
		if (rc>0) {
			struct cmd *cmd;
			int argc;
			char *argv[16];
			if (rc>200) {
				rc=200;
			}
			buf[rc]=0;
			rc=argit(buf,rc,argv);
			if (rc<0) {
				continue;
			}
			argc=rc;
			cmd=lookup_cmd(argv[0],fd);
			if (cmd) {
				int rc;
				fprintf(fd,"\n");
				rc=cmd->fnc(argc,argv,&env);
				if (rc<0) {
					fprintf(fd,"%s returned %d\n",argv[0],rc);
				}
			}
		}
	}
}
