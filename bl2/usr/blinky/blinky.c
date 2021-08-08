
#include "sys.h"
#include "sys_env.h"
#include "io.h"
#include "config.h"

#include "led_drv.h"

#include <string.h>

int blinky(int argc, char **argv, struct Env *env);

static struct cmd cmds[] = {
	{"help", generic_help_fnc},
	{"blink", blinky},
	{0,0}
};

static struct cmd_node cmn = {
	"blink_pkg",
	cmds,
};

int init_blinky() {
	install_cmd_node(&cmn,root_cmd_node);
	return 0;
}


#if 0
void tic_wait(unsigned int tic) {
	while(1) {
		if ((((int)tic)-((int)tq_tic))<=0)  {
			return;
		}
	}
}
#endif


struct blink_data {
	unsigned int led;
	unsigned int delay;
};


void blink(struct blink_data *bd) {
	int fd=io_open(LED_DRV);
	if (fd<0) return;
	while(1) {
		int led_stat;
		int rc=io_control(fd,LED_CTRL_STAT,&led_stat,sizeof(led_stat));
		if (rc<0) return;
		if (led_stat&bd->led) {
			rc=io_control(fd,LED_CTRL_DEACTIVATE,&bd->led,sizeof(bd->led));
		} else {
			rc=io_control(fd,LED_CTRL_ACTIVATE,&bd->led,sizeof(bd->led));
		}

		sleep(bd->delay);
	}
}


#if 0
void blink_loop(struct blink_data *bd) {
	int fd=io_open(LED_DRV);
	if (fd<0) return;
	while(1) {
		int led_stat;
		int rc=io_control(fd,LED_CTRL_STAT,&led_stat,sizeof(led_stat));
		if (rc<0) return;
		if (led_stat&bd->led) {
			rc=io_control(fd,LED_CTRL_DEACTIVATE,&bd->led,sizeof(bd->led));
		} else {
			rc=io_control(fd,LED_CTRL_ACTIVATE,&bd->led,sizeof(bd->led));
		}
		tic_wait((bd->delay/10)+tq_tic);
	}
}
#endif


int blinky(int argc, char **argv, struct Env *env) {
	static int bstate=0;
#if defined(MB997C)
	struct blink_data green={LED_GREEN,1000};
	struct blink_data amber={LED_AMBER,500};
	struct blink_data red={LED_RED,750};
	struct blink_data blue={LED_BLUE,100};
#elif defined(MB1075B)
	struct blink_data green={LED_GREEN,1000};
	struct blink_data red={LED_RED,750};
#else
	return -1;
#endif

        fprintf(env->io_fd, "In starting blink tasks\n");

//        io_open("test_driver");

//        thread_create(sys_mon,"usb_serial0",0,2,"sys_mon_u");
	if (argc!=2) {
		 fprintf(env->io_fd, "need an argument on or off\n");
		 return -1;
	}

	if (strcmp(argv[1],"on")==0) {
		if (bstate==1) {
			fprintf(env->io_fd, "need an argument on or off\n");
			return -1;
		}
		bstate=1;
#if defined(MB997C)
		thread_create(blink,&green,sizeof(green),1,"green");
		thread_create(blink,&amber,sizeof(amber),1,"amber");
		thread_create(blink, &red,sizeof(red),1, "red");
		thread_create(blink, &blue,sizeof(blue),1,"blue");
#elif defined(MB1075B)
		thread_create(blink,&green,sizeof(green),1,"green");
		thread_create(blink, &red,sizeof(red),1, "red");
#endif
//	thread_create(blink_loop,&red,sizeof(red),256,"red_looping");
	} else if (strcmp(argv[1],"off")==0) {
		if (bstate==0) {
			fprintf(env->io_fd, "blinky is not running\n");
			return -1;
		}
		fprintf(env->io_fd, "sorry, thread destruct NOT implemented\n");
	}
	return 0;
}

int init_pkg(void) {
	init_blinky();
	return 0;
}
