/* $OBD1_GW: main.c, v1.1 2014/04/07 21:44:00 anders Exp $ */

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
 * @(#)main.c
 */

#include "sys.h"
#include "sys_env.h"
#include "io.h"
#include "asynchio.h"

#include <string.h>

#include "obd1_drv.h"


static struct cmd cmds[] = {
	{"help", generic_help_fnc},
	{0,0}
};

static struct cmd_node cmn = {
	"obd1",
	cmds,
};

#define RSP 0x01
#define RP  0x04
#define HBV 0x40
#define AMP 0x80

#define MV2_DIAG_FACTORY_TEST	0x08
#define MV2_DIAG_DIAG_TEST	0x10
#define MV2_DIAG_ALDL_TEST	0x20

static void dump_data(const char *buf) {

	char *dm;
	if (buf[0]&MV2_DIAG_FACTORY_TEST) {
		dm="Factory test";
	} else if (buf[0]&MV2_DIAG_DIAG_TEST) {
		dm="Diagnostic test";
	} else if (buf[0]&MV2_DIAG_ALDL_TEST) {
		dm="Aldl test";
	} else {
		dm="---";
	}
	printf("\n\n\nData Dump\n");
	printf("Mode Word 2: %x, %cRoad_speed_pulse, %cref_pulse, DM %s, %cHigh_bat_volt, %cAir_meter_pulse\n",
		buf[0], buf[0]&RSP?'+':'-', buf[0]&RP?'+':'-', dm, buf[0]&HBV?'+':'-', buf[0]&AMP?'+':'-');
	printf("PromId: %d, ISSPMP (iac step): %d, Coolant temp: %d (Conv %d c)\n",
			(buf[1]<<8)|buf[2], buf[3], buf[4], (buf[4]*75)-4000);

	printf("MPH: %d, EGR dc: %d, RPM: %d (conv %d), TPS: %d (conv %d %), pulse corr cl: %d, Oxy: %d\n",
		buf[5], buf[6], buf[7], buf[7]*25, buf[8], (buf[8]*100)>>8, buf[9], buf[10]);

	printf("MALFFLG1: %x, (%cSpeedSensor, %cMAT Low, %cTPS Low, %cTPS High, %cCoolant low temp, %cCoolant high temp, %cOxy sensor, %cNo ref pulse\n",
		buf[11], buf[11]&1?'+':'-', buf[11]&2?'+':'-', buf[11]&4?'+':'-', buf[11]&8?'+':'-',
		buf[11]&0x10?'+':'-', buf[11]&0x20?'+':'-', buf[11]&0x40?'+':'-', buf[11]&0x80?'+':'-');

	printf("MALFFLG2: %x, (%cSpark tim err, %cCylSel err, %cMAF low, %cMAF high, %cEgr, %cIntake air temp high\n",
		buf[12], buf[12]&1?'+':'-', buf[12]&2?'+':'-', buf[12]&8?'+':'-',
		buf[12]&0x10?'+':'-', buf[12]&0x20?'+':'-', buf[12]&0x80?'+':'-');

	printf("MALFFLG3: %x, (%cADU err, %cFuel pump err, %cOverVolt, %cCalpack missing, %cPromErr, %cOXY Sens rich, %cOxySens Lean, %cESC failure\n",
		buf[13], buf[13]&0x01?'+':'-', buf[13]&0x02?'+':'-', buf[13]&0x04?'+':'-',
		buf[13]&0x08?'+':'-',buf[13]&0x10?'+':'-', buf[13]&0x20?'+':'-',
		buf[13]&0x40?'+':'-',buf[13]&0x80?'+':'-');

	printf("MALFFLG4: %x\n", buf[14]);

	printf("Fuel/Air: %x\n", buf[15]);

	printf("MAT: %x (conv %d c)\n", buf[16], (buf[16]*75)-4000);

	printf("MCU stat: %x, (%cP/N, %cCruise, %cTCC, %cPS high press)\n",
		buf[17], buf[17]&0x01?'+':'-', buf[17]&0x02?'+':'-', buf[17]&0x04?'+':'-',
		buf[17]&0x08?'+':'-');

	printf("LV8: %x, BLM: %x, ALDLCNTR: %x\n",
		buf[18], buf[19], buf[20]);
	printf("DISPFLOW: %d,%d\n", buf[21], (buf[22]*100)>>8);
	printf("Injector pulse %d.%d ms\n", buf[23], buf[24]);
}

void obd1_gw(void *dum) {
	char buf[32];

	int fd=io_open(OBD_DRV0);
	if (fd<0) {
		return;
	}

	while(1) {
		int rc=io_read(fd,buf,sizeof(buf));

		printf("hej, read returned %d\n",rc);
		dump_data(buf);
	}
}

//int main(void) {
int init_pkg(void) {
	/* create some jobs */
	thread_create(obd1_gw,0,0,1,"obd1_gw");
//	while (1);
	return 0;
}
