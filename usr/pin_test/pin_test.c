
#include "sys.h"
#include "sys_env.h"
#include "io.h"
#include "config.h"

#include "gpio_drv.h"
#include "led_drv.h"

#include <string.h>

int loop_read(int argc, char **argv, struct Env *env);

static struct cmd cmds[] = {
	{"help", generic_help_fnc},
	{"loop_read", loop_read},
	{0,0}
};

static struct cmd_node cmn = {
	"pin_test",
	cmds,
};

int init_pin_test() {
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


#endif
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


int loop_read(int argc, char **argv, struct Env *env) {
	int gpio_fd=io_open(GPIO_DRV);
	int led_fd=io_open(LED_DRV);
	int flags=0;
	int pin_group;
	int pin_no;
	char *pin_no_str;
	int pin=0;
	int pin_stat;
	int rc;
	int led_red=LED_RED;

        fprintf(env->io_fd, "In reading pin and setting red led when hi\n");

	if (argc!=2) {
		 fprintf(env->io_fd, "need argument: PIN_NO\n");
		 return -1;
	}

	if (strncmp(argv[1],"PA",2)==0) {
		pin_group=0x00;
	} else if (strncmp(argv[1],"PB",2)==0) {
		pin_group=0x10;
	} else if (strncmp(argv[1],"PC",2)==0) {
		pin_group=0x20;
	} else if (strncmp(argv[1],"PD",2)==0) {
		pin_group=0x30;
	} else if (strncmp(argv[1],"PE",2)==0) {
		pin_group=0x40;
	} else if (strncmp(argv[1],"PF",2)==0) {
		pin_group=0x50;
	} else if (strncmp(argv[1],"PG",2)==0) {
		pin_group=0x60;
	} else if (strncmp(argv[1],"PH",2)==0) {
		pin_group=0x70;
	} else if (strncmp(argv[1],"PI",2)==0) {
		pin_group=0x80;
	} else {
		fprintf(env->io_fd, "Pin :%s: does not exist\n", argv[1]);
		return -1;
	}

	pin_no_str=&argv[1][2];

	pin_no=strtoul(pin_no_str,0,10);

	if (pin_no>15) {
		fprintf(env->io_fd, "Pin number must be between 0-15\n");
		return -1;
	}

	pin=pin_group|pin_no;

	rc=io_control(gpio_fd, GPIO_BIND_PIN, &pin, sizeof(pin));
	if (rc<0) {
		fprintf(env->io_fd, "Failed to bind pin %x, fault coude %d\n", pin, rc);
		return -1;
	}

	flags=GPIO_DIR(0,GPIO_INPUT);
	flags=GPIO_DRIVE(flags,GPIO_FLOAT);
	flags=GPIO_SPEED(flags,GPIO_SPEED_SLOW);

	rc=io_control(gpio_fd, GPIO_SET_FLAGS, &flags, sizeof(flags));
	if (rc<0) {
		fprintf(env->io_fd, "Failed to set pin %x flags, fault coude %d\n", pin, rc);
		return -1;
	}

	while(1) {
		rc=io_control(gpio_fd, GPIO_SENSE_PIN, &pin_stat, sizeof(pin_stat));
		if (rc<0) {
			fprintf(env->io_fd, "error %d from GPIO_SENSE_PIN\n");
			return -1;
		}

		fprintf(env->io_fd, "pin (%x) state: %d\n", pin, pin_stat);
		if (pin_stat) {
			rc=io_control(led_fd,LED_CTRL_ACTIVATE,&led_red,sizeof(led_red));
		} else {
			rc=io_control(led_fd,LED_CTRL_DEACTIVATE,&led_red,sizeof(led_red));
		}
		rc=io_control(env->io_fd, IO_POLL, (void *)EV_READ,0);
		if (rc>0) {
			fprintf(env->io_fd, "got input on keyboard, leaving");
			break;
		}
	}

	rc=io_control(led_fd,LED_CTRL_DEACTIVATE,&led_red,sizeof(led_red));
	io_close(gpio_fd);
	io_close(led_fd);
	return 0;
}

#if 0
int init_pkg(void) {
	init_pin_test();
	return 0;
}
#endif
