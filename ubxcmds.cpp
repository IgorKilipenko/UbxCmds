// ConsoleApplication1.cpp: определяет точку входа для консольного приложения.
//

//#include "stdafx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>

using namespace std;

#define UBXSYNC1 0xB5 /* ubx message sync code 1 */
#define UBXSYNC2 0x62 /* ubx message sync code 2 */
#define UBXCFG 0x06   /* ubx message cfg-??? */

//#define ID_NAVSOL   0x0106      /* ubx message id: nav solution info */
//#define ID_NAVTIME  0x0120      /* ubx message id: nav time gps */
//#define ID_RXMRAW   0x0210      /* ubx message id: raw measurement data */
//#define ID_RXMSFRB  0x0211      /* ubx message id: subframe buffer */
//#define ID_RXMSFRBX 0x0213      /* ubx message id: raw subframe data */
//#define ID_RXMRAWX  0x0215      /* ubx message id: multi-gnss raw meas data */
//#define ID_TRKD5    0x030A      /* ubx message id: trace mesurement data */
//#define ID_TRKMEAS  0x0310      /* ubx message id: trace mesurement data */
//#define ID_TRKSFRBX 0x030F      /* ubx message id: trace subframe buffer */

#define FU1 1 /* ubx message field types */
#define FU2 2
#define FU4 3
#define FI1 4
#define FI2 5
#define FI4 6
#define FR4 7
#define FR8 8
#define FS32 9

/* set fields (little-endian) ------------------------------------------------*/
static void setU1(unsigned char *p, unsigned char u)
{
	*p = u;
}
static void setU2(unsigned char *p, unsigned short u)
{
	memcpy(p, &u, 2);
}
static void setU4(unsigned char *p, unsigned int u)
{
	memcpy(p, &u, 4);
}
static void setI1(unsigned char *p, signed char i)
{
	*p = (unsigned char)i;
}
static void setI2(unsigned char *p, short i)
{
	memcpy(p, &i, 2);
}
static void setI4(unsigned char *p, int i)
{
	memcpy(p, &i, 4);
}
static void setR4(unsigned char *p, float r)
{
	memcpy(p, &r, 4);
}
static void setR8(unsigned char *p, double r)
{
	memcpy(p, &r, 8);
}

static void setcs(unsigned char *buff, int len)
{
	unsigned char cka = 0, ckb = 0;
	int i;

	for (i = 2; i < len - 2; i++)
	{
		cka += buff[i];
		ckb += cka;
	}
	buff[len - 2] = cka;
	buff[len - 1] = ckb;
}

int gen_ubx(const char *msg, unsigned char *buff)
{
	const char *cmd[] = {
		"PRT", "USB", "MSG", "NMEA", "RATE", "CFG", "TP", "NAV2", "DAT", "INF",
		"RST", "RXM", "ANT", "FXN", "SBAS", "LIC", "TM", "TM2", "TMODE", "EKF",
		"GNSS", "ITFM", "LOGFILTER", "NAV5", "NAVX5", "ODO", "PM2", "PWR", "RINV", "SMGR",
		"TMODE2", "TMODE3", "TPS", "TXSLOT", ""};
	const unsigned char id[] = {
		0x00, 0x1B, 0x01, 0x17, 0x08, 0x09, 0x07, 0x1A, 0x06, 0x02,
		0x04, 0x11, 0x13, 0x0E, 0x16, 0x80, 0x10, 0x19, 0x1D, 0x12,
		0x3E, 0x39, 0x47, 0x24, 0x23, 0x1E, 0x3B, 0x57, 0x34, 0x62,
		0x36, 0x71, 0x31, 0x53};
	const int prm[][32] = {
		{FU1, FU1, FU2, FU4, FU4, FU2, FU2, FU2, FU2},	/* PRT */
		{FU2, FU2, FU2, FU2, FU2, FU2, FS32, FS32, FS32}, /* USB */
		{FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1},		  /* MSG */
		{FU1, FU1, FU1, FU1},							  /* NMEA */
		{FU2, FU2, FU2},								  /* RATE */
		{FU4, FU4, FU4, FU1},							  /* CFG */
		{FU4, FU4, FI1, FU1, FU2, FI2, FI2, FI4},		  /* TP */
		{FU1, FU1, FU2, FU1, FU1, FU1, FU1, FI4, FU1, FU1, FU1, FU1, FU1, FU1, FU2, FU2, FU2, FU2,
		 FU2, FU1, FU1, FU2, FU4, FU4},						/* NAV2 */
		{FR8, FR8, FR4, FR4, FR4, FR4, FR4, FR4, FR4},		/* DAT */
		{FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1}, /* INF */
		{FU2, FU1, FU1},									/* RST */
		{FU1, FU1},											/* RXM */
		{FU2, FU2},											/* ANT */
		{FU4, FU4, FU4, FU4, FU4, FU4, FU4, FU4},			/* FXN */
		{FU1, FU1, FU1, FU1, FU4},							/* SBAS */
		{FU2, FU2, FU2, FU2, FU2, FU2},						/* LIC */
		{FU4, FU4, FU4},									/* TM */
		{FU1, FU1, FU2, FU4, FU4},							/* TM2 */
		{FU4, FI4, FI4, FI4, FU4, FU4, FU4},				/* TMODE */
		{FU1, FU1, FU1, FU1, FU4, FU2, FU2, FU1, FU1, FU2}, /* EKF */
		{FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU4},		/* GNSS */
		{FU4, FU4},											/* ITFM */
		{FU1, FU1, FU2, FU2, FU2, FU4},						/* LOGFILTER */
		{FU2, FU1, FU1, FI4, FU4, FI1, FU1, FU2, FU2, FU2, FU2, FU1, FU1, FU1, FU1, FU1, FU1, FU2,
		 FU1, FU1, FU1, FU1, FU1, FU1}, /* NAV5 */
		{FU2, FU2, FU4, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU2, FU1, FU1, FU1, FU1,
		 FU1, FU1, FU1, FU1, FU1, FU1, FU2},						  /* NAVX5 */
		{FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1},				  /* ODO */
		{FU1, FU1, FU1, FU1, FU4, FU4, FU4, FU4, FU2, FU2},			  /* PM2 */
		{FU1, FU1, FU1, FU1, FU4},									  /* PWR */
		{FU1, FU1},													  /* RINV */
		{FU1, FU1, FU2, FU2, FU1, FU1, FU2, FU2, FU2, FU2, FU4},	  /* SMGR */
		{FU1, FU1, FU2, FI4, FI4, FI4, FU4, FU4, FU4},				  /* TMODE2 */
		{FU1, FU1, FU2, FI4, FI4, FI4, FU4, FU4, FU4},				  /* TMODE3 */
		{FU1, FU1, FU1, FU1, FI2, FI2, FU4, FU4, FU4, FU4, FI4, FU4}, /* TPS */
		{FU1, FU1, FU1, FU1, FU4, FU4, FU4, FU4, FU4}				  /* TXSLOT */
	};
	unsigned char *q = buff;
	char mbuff[1024], *args[32], *p;
	int i, j, n, narg = 0;

	strcpy(mbuff, msg);
	for (p = strtok(mbuff, " "); p && narg < 32; p = strtok(NULL, " "))
	{
		args[narg++] = p;
	}
	if (narg < 1 || strncmp(args[0], "CFG-", 4))
		return 0;

	for (i = 0; *cmd[i]; i++)
	{
		if (!strcmp(args[0] + 4, cmd[i]))
			break;
	}
	if (!*cmd[i])
		return 0;

	*q++ = UBXSYNC1;
	*q++ = UBXSYNC2;
	*q++ = UBXCFG;
	*q++ = id[i];
	q += 2;
	for (j = 1; prm[i][j - 1] || j < narg; j++)
	{
		switch (prm[i][j - 1])
		{
		case FU1:
			setU1(q, j < narg ? (unsigned char)atoi(args[j]) : 0);
			q += 1;
			break;
		case FU2:
			setU2(q, j < narg ? (unsigned short)atoi(args[j]) : 0);
			q += 2;
			break;
		case FU4:
			setU4(q, j < narg ? (unsigned int)atoi(args[j]) : 0);
			q += 4;
			break;
		case FI1:
			setI1(q, j < narg ? (signed char)atoi(args[j]) : 0);
			q += 1;
			break;
		case FI2:
			setI2(q, j < narg ? (short)atoi(args[j]) : 0);
			q += 2;
			break;
		case FI4:
			setI4(q, j < narg ? (int)atoi(args[j]) : 0);
			q += 4;
			break;
		case FR4:
			setR4(q, j < narg ? (float)atof(args[j]) : 0);
			q += 4;
			break;
		case FR8:
			setR8(q, j < narg ? (double)atof(args[j]) : 0);
			q += 8;
			break;
		case FS32:
			sprintf((char *)q, "%-32.32s", j < narg ? args[j] : "");
			q += 32;
			break;
			//default: setU1(q, j < narg ? (unsigned char)atoi(args[j]) : 0); q += 1; break;
		}
	}
	n = (int)(q - buff) + 2;
	setU2(buff + 4, (unsigned short)(n - 8));
	setcs(buff, n);

	return n;
}

void printHexMsg(const char *cmd)
{
	unsigned char buff[1024];
	int n = 0;

	n = gen_ubx(cmd + 4, buff);
	char *msg = new char[n];
	//cout << n << endl;
	for (int i = 0; i < n; i++)
	{
		printf("%02X ", buff[i]);
	}
}

void printSrcMsg(const char *cmd)
{
	char mbuff[1024];

	strcpy(mbuff, cmd + 4);
	char *p = strtok(mbuff, " ");
	do
	{
		cout << p;
	} while (p = strtok(NULL, " "));
}

int main()
{
	
	const char *cmdsStart[] = {
		"!UBX CFG-MSG 3 10 0 1 0 0 0 0",
		"!UBX CFG-MSG 3  2 0 1 0 0 0 0",
		"!UBX CFG-MSG 1 32 0 1 0 0 0 0",
		"!UBX CFG-MSG 1 34 0 1 0 0 0 0",
		""};
	const char *cmdsEnd[] = {
		"!UBX CFG-MSG 3 10 0 0 0 0 0 0",
		"!UBX CFG-MSG 3  2 0 0 0 0 0 0",
		"!UBX CFG-MSG 1 32 0 0 0 0 0 0",
		"!UBX CFG-MSG 1 34 0 0 0 0 0 0",
		""};

	for (int i = 0; *cmdsStart[i]; i++)
	{
		printHexMsg(cmdsStart[i]);
		printf("\t( %s )\r\n", cmdsStart[i]);
	}

	cout << endl
		 << "@" << endl
		 << endl;

	for (int i = 0; *cmdsEnd[i]; i++)
	{
		printHexMsg(cmdsEnd[i]);
		printf("\t( %s )\r\n", cmdsEnd[i]);
	}
	
	cout << endl << "Enter cmd: ";
	char cmd[1024];
	do {
		cin.getline(cmd,sizeof(cmd));
		printHexMsg(cmd);
		printf("\t( %s )\r\n", cmd);
	}while(cmd);


	char res;
	cin >> res;
	return 0;
}
