#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "programmer.h"
#include <windows.h>

#define MIN(a,b)        ((a) < (b) ? (a) : (b))
#define MAX(a,b)        ((a) > (b) ? (a) : (b))

typedef struct veh_profile VEHICLE_DATA;
struct veh_profile
{
	float A; // rolling coefficient in kW*sec/meter
	float B; // rotation coefficient in kW*sec^2/meter^2
	float C; // drag coefficient in kW*sec^3/meter^3
	float M; // vehicle source mass in metric tons
	float f; // fixed mass factor in metric tons
	int sourceType; // vehicle source type
	int ID; // vehicle ID
	float theta; // road grade angle
	float vel[4]; // 3-second velocities
	float acc[4]; // 3-seconde accelerations
	int linkID[4]; // 3-second link IDs
	float VSP[4]; // 3-second VSPs
	int mode[4]; // 3-second modes
	float tm[4]; // 3-second time stamps
	int first2sec; // the first 2 seconds vehicle enters the network
	float energy[4], CO2[4], CO[4], HC[4], NOx[4], PM[4]; // 3-second emission data
	int origin; // vehicle's origin zone
	int dest; // vehicle's destination zone
	float tripDist; // vehicle's total trip distance
	int bound; // 1: northbound; 2: southbound: 0 otherwise
	int CACCstatus; // 1: leader; 0: uninitialized; -1: follower
};

const float GRAV = 9.8; // gravity coefficient in meter/second^2
const float INDEX = 2.23693629; // conversion index from m/s to mph
const float meter2mile = 0.000621371;
const float followThreshold = 40; // threshold of following distance
const float vehicleLength = 4.8768 ;
const float targetSpacing = 5;
float DELTA = 0.1; // default small value, later modified in qpx_NET_postOpen()
// for MOVES
float moves[63][41][6]; // moves[regClass][opMode][emissionCategory]
int modeBins[50] = {0}; // opMode distribution
char out1Path[150]; // file "data_sbs.dat"
char out2Path[150]; // file "MOVES_sum.dat"
char out3Path[150]; // file "opMode.dat"
char out4Path[150]; // file "VSP.dat"
char out5Path[150]; // file "TT.dat"
int timeStamp_sbs[10000000]; // second by second vehcle ID
int vehID_sbs[10000000]; // second by second vehcle ID
int linkID_sbs[10000000]; // second by second link ID
float VSP_sbs[10000000]; // second by second VSP
float speed_sbs[10000000]; // second by second speed
float acc_sbs[10000000]; // second by second acc
int vehType_sbs[10000000]; // vehicle type mapping to speed_sbs
int origin_sbs[10000000];
int dest_sbs[10000000];
int counter_sbs; // counter for VSP_sbs
float TT_veh[50000]; // travel time of each vehicle
int counter_arrived = 0;
int counter_released = 0;
// aggregated data
double VMT = 0, VHT = 0, dist_sum = 0, energy_sum = 0, CO2_sum = 0, CO_sum = 0, HC_sum = 0, NOx_sum = 0, PM_sum = 0;

void qpx_NET_postOpen()
{
	int i, j, k, md;
	int reg[13] = {11,21,31,32,41,42,43,51,52,53,54,61,62}; // source types
	char *path, *outPath;
	char inPath[150] = "";
	FILE *fin;
	
	DELTA = 0.5*qpg_CFG_timeStep();
	path = qpg_NET_dataPath();
	outPath = qpg_NET_statsPath();
	strcpy(inPath, path);
	strcpy(out1Path, outPath);
	strcpy(out2Path, outPath);
	strcpy(out3Path, outPath);
	strcpy(out4Path, outPath);
	strcpy(out5Path, outPath);
	strcat(inPath, "/sourceTypes_2005.txt"); 
	strcat(out1Path, "/data_sbs.dat");
	strcat(out2Path, "/MOVES_sum.dat");
	strcat(out3Path, "/opMode.dat");
	strcat(out4Path, "/VSP.dat");
	strcat(out5Path, "/TT.dat");

	fin = fopen(inPath, "r");
	if (fin == NULL)
		qps_GUI_printf("Couldn't open file. (simulation started)");
	fscanf(fin, "%d,", &md); // md: opMode
	for (i = 0; i < 13; i++) // i: regClass
		for (j = 0; j < 41; j++) // j: opMode
			if (md == j)
			{
				fscanf(fin, "%f,%f,%f,%f,%f,%f", &moves[reg[i]][j][0], &moves[reg[i]][j][1], &moves[reg[i]][j][2], &moves[reg[i]][j][3], &moves[reg[i]][j][4], &moves[reg[i]][j][5]);
				fscanf(fin, "%d,", &md);
			}
	for (j = 0; j < 41; j++) {
		for (k = 0; k <= 5; k++) {
			moves[20][j][k] = moves[21][j][k];
		}
	}
	fclose(fin);
}

int opMode(float vsp, float speed, float acc)
{
	speed = speed*INDEX;
	acc = acc*INDEX;
	if (acc <= -2) return 0;
	
	if (speed < 1 && speed >= -1) return 1;
	if (speed < 25 && speed >= 0) 
	{
		if (vsp < 0) return 11;
		if (vsp < 3) return 12;
		if (vsp < 6) return 13;
		if (vsp < 9) return 14;
		if (vsp < 12) return 15;
		return 16;}
	else if (speed < 50) {
		if (vsp < 0) return 21;
		if (vsp < 3) return 22;
		if (vsp < 6) return 23;
		if (vsp < 9) return 24;
		if (vsp < 12) return 25;
		if (vsp < 18) return 27;
		if (vsp < 24) return 28; 
		if (vsp < 30) return 29;
		return 30;}
	else {
		if (vsp < 6) return 33;
		if (vsp < 12) return 35;
		if (vsp < 18) return 37;
		if (vsp < 24) return 38;
		if (vsp < 30) return 39;
		return 40;}
}

void updateVehAttributes(VEHICLE_DATA* myVeh)
{
	if(myVeh->sourceType == 11)
	{
		myVeh->A = 0.0251;
        myVeh->B = 0;
        myVeh->C = 0.000315;
        myVeh->M = 0.285;
		myVeh->f = 0.285;
		//myVeh->regClass = 10;
	}
	else if(myVeh->sourceType == 21 || myVeh->sourceType == 20)
	{
		myVeh->A = 0.156461;
        myVeh->B = 0.002002;
        myVeh->C = 0.000493;
        myVeh->M = 1.4788;
		myVeh->f = 1.4788;
		//myVeh->regClass = 20;
	}
	else if(myVeh->sourceType == 31)
	{
		myVeh->A = 0.22112;
        myVeh->B = 0.002838;
        myVeh->C = 0.000698;
        myVeh->M = 1.86686;
		myVeh->f = 1.86686;
		//myVeh->regClass = 30;
	}
	else if(myVeh->sourceType == 32)
	{
		myVeh->A = 0.235008;
        myVeh->B = 0.003039;
        myVeh->C = 0.000748;
        myVeh->M = 2.05979;
		myVeh->f = 2.05979;
		//myVeh->regClass = 30;
	}
	else if(myVeh->sourceType == 41)
	{
		myVeh->A = 1.29515;
        myVeh->B = 0;
        myVeh->C = 0.003715;
        myVeh->M = 19.5937;
		myVeh->f = 17.1;
		//myVeh->regClass = 48;
	}
	else if(myVeh->sourceType == 42)
	{
		myVeh->A = 1.0944;
        myVeh->B = 0;
        myVeh->C = 0.003587;
        myVeh->M = 16.556;
		myVeh->f = 17.1;
		//myVeh->regClass = 48;
	}
	else if(myVeh->sourceType == 43)
	{
		myVeh->A = 0.746718;
        myVeh->B = 0;
        myVeh->C = 0.002176;
        myVeh->M = 9.06989;
		myVeh->f = 17.1;
		//myVeh->regClass = 46;
	}
	else if(myVeh->sourceType == 51)
	{
		myVeh->A = 1.41705;
        myVeh->B = 0;
        myVeh->C = 0.003572;
        myVeh->M = 20.6845;
		myVeh->f = 17.1;
		//myVeh->regClass = 47;
	}
	else if(myVeh->sourceType == 52)
	{
		myVeh->A = 0.561933;
        myVeh->B = 0;
        myVeh->C = 0.001603;
        myVeh->M = 7.64159;
		myVeh->f = 17.1;
		//myVeh->regClass = 42;
	}
	else if(myVeh->sourceType == 53)
	{
		myVeh->A = 0.498699;
        myVeh->B = 0;
        myVeh->C = 0.001474;
        myVeh->M = 6.25047;
		myVeh->f = 17.1;
		//myVeh->regClass = 41;
	}
	else if(myVeh->sourceType == 54)
	{
		myVeh->A = 0.617371;
        myVeh->B = 0;
        myVeh->C = 0.002105;
        myVeh->M = 6.73483;
		myVeh->f = 17.1;
		//myVeh->regClass = 42;
	}
	else if(myVeh->sourceType == 61)
	{
		myVeh->A = 1.96354;
        myVeh->B = 0;
        myVeh->C = 0.004031;
        myVeh->M = 29.3275;
		myVeh->f = 17.1;
		//myVeh->regClass = 47;
	}
	else //if(myVeh->sourceType == 62)
	{
		myVeh->A = 2.08126;
        myVeh->B = 0;
        myVeh->C = 0.004188;
        myVeh->M = 31.4038;
		myVeh->f = 17.1;
		//myVeh->regClass = 47;
	}
}

void  qpx_VHC_release(VEHICLE* Vp)
{
	VEHICLE_DATA *myVeh = calloc(1, sizeof(VEHICLE_DATA));
	counter_released++;
	// initialize data for MOVES model
	myVeh->ID = qpg_VHC_uniqueID(Vp);
	myVeh->sourceType = qpg_VHC_type(Vp); // emission tables directly related to source types
	myVeh->theta = 0;
	myVeh->first2sec = 1;
	myVeh->origin = qpg_VHC_origin(Vp);
	myVeh->dest = qpg_VHC_destination(Vp);
	myVeh->tripDist = qpg_RTR_distanceRemaining(qpg_VHC_link(Vp), Vp);
	myVeh->CACCstatus = 0;
	updateVehAttributes(myVeh);
	qps_VHC_userdata(Vp, (VEHICLE_DATA *)myVeh);
}

void calEmissions(VEHICLE_DATA* myVeh, float currTime, float vel_c, int linkid)
{
	int i;
	// shift 4-second data
	for (i = 0; i < 3; i++)
	{
		myVeh->vel[i] = myVeh->vel[i+1];
		myVeh->acc[i] = myVeh->acc[i+1];
		myVeh->linkID[i] = myVeh->linkID[i+1];
		myVeh->VSP[i] = myVeh->VSP[i+1];
		myVeh->mode[i] = myVeh->mode[i+1];
		myVeh->tm[i] = myVeh->tm[i+1];
		myVeh->energy[i] = myVeh->energy[i+1];
		myVeh->CO2[i] = myVeh->CO2[i+1];
		myVeh->CO[i] = myVeh->CO[i+1];
		myVeh->HC[i] = myVeh->HC[i+1];
		myVeh->NOx[i] = myVeh->NOx[i+1];
		myVeh->PM[i] = myVeh->PM[i+1];
	}
	myVeh->tm[3] = currTime;
	myVeh->linkID[3] = linkid;
	myVeh->vel[3] = vel_c;
	myVeh->acc[2] = (myVeh->vel[3] - myVeh->vel[1])/2;
	myVeh->VSP[2] = (myVeh->A*myVeh->vel[2] + myVeh->B*pow(myVeh->vel[2],2) + myVeh->C*pow(myVeh->vel[2],3) + myVeh->M*(myVeh->acc[2] + GRAV*sin(myVeh->theta))*myVeh->vel[2])/myVeh->f;
	myVeh->mode[2] = opMode(myVeh->VSP[2], myVeh->vel[2], myVeh->acc[2]);
	if (myVeh->acc[0] < -1 && myVeh->acc[1] < -1 && myVeh->acc[2] < -1) myVeh->mode[2] = 0;
	modeBins[myVeh->mode[2]] += 1;
	myVeh->HC[2] = moves[myVeh->sourceType][myVeh->mode[2]][0];
	myVeh->CO[2] = moves[myVeh->sourceType][myVeh->mode[2]][1];
	myVeh->NOx[2] = moves[myVeh->sourceType][myVeh->mode[2]][2];
	myVeh->CO2[2] = moves[myVeh->sourceType][myVeh->mode[2]][3];
	myVeh->energy[2] = moves[myVeh->sourceType][myVeh->mode[2]][4];
	myVeh->PM[2] = moves[myVeh->sourceType][myVeh->mode[2]][5];

	dist_sum += myVeh->vel[2];
	energy_sum += myVeh->energy[2];
	CO2_sum += myVeh->CO2[2];
	CO_sum += myVeh->CO[2];
	HC_sum += myVeh->HC[2];
	NOx_sum += myVeh->NOx[2];
	PM_sum += myVeh->PM[2];
	timeStamp_sbs[counter_sbs] = (int)(currTime+0.5) - 1;
	speed_sbs[counter_sbs] = myVeh->vel[2];
	acc_sbs[counter_sbs] = myVeh->acc[2];
	VSP_sbs[counter_sbs] = myVeh->VSP[2];
	vehType_sbs[counter_sbs] = myVeh->sourceType;
	vehID_sbs[counter_sbs] = myVeh->ID;
	origin_sbs[counter_sbs] = myVeh->origin;
	dest_sbs[counter_sbs] = myVeh->dest;
	counter_sbs++;
}

float in_qpo_CFM_Speed(LINK* link, VEHICLE* Vp, float CFM_Speed)
{
	float v_lmt = qpg_LNK_speedlimit(link)/2.2369 + 10;
	float currTime = qpg_CFG_simulationTime();
	float recSpeed, currVel = qpg_VHC_speed(Vp);
	int linkid = qpg_LNK_index(link), mode, currZone = qpg_LNK_zone(link);
	float timeStep = qpg_CFG_timeStep();
	float currentClearance = 0, currentSpacing = 0, targetDistance = 0, relativeSpeed = 0;
	VEHICLE* ahead = qpg_VHC_ahead(Vp);
	VEHICLE_DATA *myVeh = (VEHICLE_DATA*)qpg_VHC_userdata(Vp);

	// calculate emissions
	if (fabs(currTime - (int)(currTime+0.5)) > DELTA || currZone == myVeh->origin || currZone == myVeh->dest)
		goto CACC;
	if (myVeh->first2sec < 3)
	{
		myVeh->tm[myVeh->first2sec] = currTime;
		myVeh->linkID[myVeh->first2sec] = linkid;
		myVeh->vel[myVeh->first2sec] = currVel;
		myVeh->first2sec++;
		goto CACC;
	}
	calEmissions(myVeh, currTime, currVel, linkid);

CACC:
	if (linkid == 1 || !ahead) myVeh->CACCstatus = 1;
	else {
		//aheadLinkid = qpg_LNK_index(qpg_VHC_link(ahead));
		//return CFM_Speed;
		if (link == qpg_VHC_link(ahead))
			currentClearance = qpg_VHC_distance(Vp) - qpg_VHC_distance(ahead);
		else if (qpg_LNK_nodeEnd(link) == qpg_LNK_nodeStart(qpg_VHC_link(ahead)))
			currentClearance = qpg_VHC_distance(Vp) + qpg_LNK_length(qpg_VHC_link(ahead)) - qpg_VHC_distance(ahead);
		else currentClearance = 100; // some distance longer than the threshold of following distance 20m
		if (currentClearance <= followThreshold) myVeh->CACCstatus = -1;
		else myVeh->CACCstatus = 1;
	} 
	if (myVeh->CACCstatus >= 0) return CFM_Speed;
	currentSpacing = currentClearance - vehicleLength;
	targetDistance = currentSpacing - targetSpacing;
	if (targetDistance > 0) relativeSpeed = targetDistance/2;
	else relativeSpeed = targetDistance/0.5;
	return MAX(CFM_Speed, qpg_VHC_speed(ahead) + relativeSpeed);
}

float qpo_CFM_leadSpeed(LINK* link, VEHICLE* Vp,  VEHICLE* ahead[])
{
	float CFM_Speed = qpg_CFM_leadSpeed(link, Vp, ahead);
	return in_qpo_CFM_Speed(link, Vp, CFM_Speed);
}

float qpo_CFM_followSpeed(LINK* link, VEHICLE* Vp,  VEHICLE* ahead[])
{
	float CFM_Speed = qpg_CFM_followSpeed(link, Vp, ahead);
	return in_qpo_CFM_Speed(link, Vp, CFM_Speed);
}

void qpx_VHC_arrive(VEHICLE* Vp, LINK* link, ZONE* zone)
{
	VEHICLE_DATA *myVeh = (VEHICLE_DATA*)qpg_VHC_userdata(Vp);
	TT_veh[counter_arrived] = qpg_VHC_existTime(Vp);
	VMT += myVeh->tripDist;
	VHT += TT_veh[counter_arrived];
	counter_arrived++;	 
	free(myVeh);
}

void  qpx_NET_complete()
{
	FILE *fsum, *fopMode, *fVSP, *fTT, *fsbs;
	int i;
	fsbs = fopen(out1Path, "w");
	fsum = fopen(out2Path, "w");
	fopMode = fopen(out3Path, "w");
	fVSP = fopen(out4Path, "w");
	fTT = fopen(out5Path, "w");
	
	if (fsum == NULL || fopMode == NULL || fVSP == NULL || fTT == NULL || fsbs == NULL)
		qps_GUI_printf("Couldn't open file. (simulation ended)");

	VMT = VMT*meter2mile;
	dist_sum = dist_sum*meter2mile;
	energy_sum = energy_sum/dist_sum/3600;
	CO2_sum = CO2_sum/dist_sum/3600;
	CO_sum = CO_sum/dist_sum/3600;
	HC_sum = HC_sum/dist_sum/3600;
	NOx_sum = NOx_sum/dist_sum/3600;
	PM_sum = PM_sum/dist_sum/3600;

	fprintf(fsum, "Energy(KJ/mi),CO2(g/mi),CO(g/mi),HC(g/mi),NOx(g/mi),PM2.5(g/mi),VHT(sec/veh),VMT(mi/veh)\n");
	fprintf(fsum, "%f, %f, %f, %f, %f, %f, %f, %f\n", energy_sum, CO2_sum, CO_sum, HC_sum, NOx_sum, PM_sum, VHT/counter_arrived, VMT/counter_arrived);
	
	for (i = 0; i <= 40; i++)
		fprintf(fopMode, "%d, %d\n", i, modeBins[i]);

	for (i = 0; i < counter_sbs; i++)
		fprintf(fVSP, "%f\n", VSP_sbs[i]);

	for (i = 0; i < counter_arrived; i++)
		fprintf(fTT, "%f\n", TT_veh[i]);

	/*fprintf(fsbs, "timeStamp, vehID, vehType, linkID, originZone, destZone, speed(m/s), acc(m/s^2)\n");
	for (i = 0; i < counter_sbs; i++)
		fprintf(fsbs, "%d, %d, %d, %d, %d, %d, %f, %f\n", timeStamp_sbs[i], vehID_sbs[i], vehType_sbs[i], linkID_sbs[i], origin_sbs[i], dest_sbs[i], speed_sbs[i], acc_sbs[i]);
	
	fprintf(fsbs, "vehID, vehType, speed(m/s), Energy(KJ/mi),CO2(g/mi),CO(g/mi),HC(g/mi),NOx(g/mi),PM2.5(g/mi)\n");
	for (i = 0; i < counter_sbs; i++)
		if (origin_sbs[i] == 40 && dest_sbs[i] == 12) fprintf(fsbs, "%d, %d, %f\n", vehID_sbs[i], vehType_sbs[i], speed_sbs[i]);
	*/
	fclose(fsum);
	fclose(fTT);
	fclose(fVSP);
	fclose(fopMode);
	fclose(fsbs);
	qps_GUI_printf("%d vehicles released. %d vehicles arrived.", counter_released,counter_arrived);
}