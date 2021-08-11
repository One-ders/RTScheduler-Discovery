#include <sys.h>
#include <string.h>
#include <syslog.h>

#define MAXLOGS	1
#define LOGSIZE (1024*4)
#define I(a) (a&(LOGSIZE-1))
#define BP(i,a) (&lbuf[i][I(a)])

static struct device_handle dh;

struct userdata {
        struct  device_handle dh;
        char    name[32];
        int     busy;
};

static struct userdata ud[16];

static struct userdata *get_userdata(void) {
        int i;
        for(i=0;i<16;i++) {
                if (!ud[i].busy) {
                        ud[i].busy=1;
                        return &ud[i];
                }
        }
        return 0;
}


static void put_userdata(struct userdata *uud) {
        int i;
        for(i=0;i<16;i++) {
                if (&ud[i]==uud) {
                        uud->busy=0;
                }
        }
}



static char lbuf[MAXLOGS][LOGSIZE];

static int log_start_ix[MAXLOGS];
static int log_end_ix  [MAXLOGS];

static int create_syslog() {
	int i;
	for(i=0;i<MAXLOGS;i++) {
		if (!lbuf[i]) {
			break;
		}
	}
	if (i==MAXLOGS) return -1;
//	lbuf[i]=malloc(LOGSIZE);
	log_start_ix[i]=log_end_ix[i]=0;
	return i;
}

static int add_entry(int logi, const char *sbuf, int size) {
	int nend;

	if ((logi<0)||(logi>=MAXLOGS)) {
		return -1;
	}

	size++;  // include 0

	nend=(log_end_ix[logi])+size;
	if ((nend-log_start_ix[logi])>LOGSIZE) {
		log_start_ix[logi]+=((nend-log_start_ix[logi])-LOGSIZE);
	}
	if (I(nend) < I(log_end_ix[logi])) {
		int len_p1=LOGSIZE-I(log_end_ix[logi]);
		int len_p2=I(nend);
		memcpy(BP(logi,log_end_ix[logi]),sbuf,len_p1);
		memcpy(BP(logi,0),&sbuf[len_p1],len_p2);
		log_end_ix[logi]=nend;
	} else {
		memcpy(BP(logi,log_end_ix[logi]),sbuf,size);
		log_end_ix[logi]+=size;
	}
	return size-1;
}

static int dump_logbuf(int logi) {
	sys_printf("syslog dump logbuf called\n");
	int six=log_start_ix[logi];
	while(log_end_ix[logi]>six) {
		int len=strlen(BP(logi, six));
		if ((len+I(six))>LOGSIZE) {
			int len_p1=LOGSIZE-I(six);
			int len_p2=strlen(BP(logi,0));
			io_add_strn(BP(logi,six),len_p1);
			six+=len_p1;
			io_add_strn(BP(logi,0),len_p2);
			six+=len_p2+1;
		} else {
			io_add_strn(BP(logi,six),len);
			six+=len+1;
		}
	}
	return 0;
}

static struct device_handle *syslog_open(void *instance, DRV_CBH callback, void *userdata) {

	sys_printf("syslog open called\n");
	return &dh;
}

static int syslog_control(struct device_handle *dh, int cmd, void *arg1, int arg2) {

	sys_printf("syslog control called\n");
	switch(cmd) {
		case RD_CHAR: {
//			struct userdata *ud=(struct userdata *)dh;
//			return syslog_read_first_entry(0,arg1,arg2);
			break;
		}
		case WR_CHAR: {
//			struct userdata *ud=(struct userdata *)dh;
			return add_entry(0,arg1,arg2);
		}
		case DUMP_BUF: {
//			struct userdata *ud=(struct userdata *)dh;
			dump_logbuf(0);
			break;
		}
		default:
			 return -1;
	}
	return 0;
}

static int syslog_start(void *inst) {
	return 0;
}

static int syslog_close(struct device_handle *udh) {
	put_userdata((struct userdata *)udh);
	return 0;
}

static int syslog_init(void *inst) {
	create_syslog();
	return 0;
}

static struct driver_ops syslog_drv_ops = {
	syslog_open,
	syslog_close,
	syslog_control,
	syslog_init,
	syslog_start,
};

static struct driver syslog_drv = {
	"syslog",
	0,
	&syslog_drv_ops,
};

void init_syslog_drv(void) {
	driver_publish(&syslog_drv);
}

INIT_FUNC(init_syslog_drv);
