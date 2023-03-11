
#define TB_SIZE 512
extern unsigned short tb[TB_SIZE];
extern unsigned short tb_i;
extern unsigned short tb_o;

#define SYSTICK_HANDLER		1
#define GET_USER_FD		2
#define	CLOSE_DRIVERS		3
#define DETACH_DRIVER_FD	4
#define SYS_DRV_WAKEUP		5
#define HANDLE_SYSCALL		6
#define SYS_SLEEP		7
#define SYS_TIMER		8
#define SYS_TIMER_REMOVE	9
#define DEVICE_READY		10
#define SYS_SLEEPON		11
#define SYS_SLEEPON_UPDATE_LIST	12
#define SYS_WAKEUP		13
#define SYS_WAKEUP_FROM_LIST	14
#define DESTROY_THREAD		15
#define PENDSV_HANDLER_C	16
#define SWITCH_ON_RETURN	17
#define SWITCH_NOW		18

#define TB_CNT2IX(a) (a&(TB_SIZE-1))

#ifdef KERNEL_TRACE

#define TRACE_ENTER(a) { \
		tb[TB_CNT2IX(tb_i)]=((a)<<8)|0; \
		tb_i++; \
		if (tb_i>=(tb_o+TB_SIZE)) { \
			tb_o=tb_i-(TB_SIZE-1); \
		} \
}

#define TRACE_EXIT(a,b) { \
		tb[TB_CNT2IX(tb_i)]=((a)<<8)|(b); \
		tb_i++; \
		if (tb_i>=(tb_o+TB_SIZE)) { \
			tb_o=tb_i-(TB_SIZE-1); \
		} \
}

#else
#define TRACE_ENTER(a)

#define TRACE_EXIT(a,b)
#endif
