/****************************************************************************
This file is part of Project Apollo - NASSP

RTCC Calculations for Skylab missions

Project Apollo is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

Project Apollo is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Project Apollo; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

See http://nassp.sourceforge.net/license/ for more details.

**************************************************************************/

#include "Orbitersdk.h"
#include "soundlib.h"
#include "apolloguidance.h"
#include "saturn.h"
#include "saturn1b.h"
#include "iu.h"
#include "LVDC.h"
#include "mcc.h"
#include "rtcc.h"
#include "../src_skylab/skylab.h"

bool RTCC::CalculationMTP_ASTP(int fcn, LPVOID& pad, char* upString, char* upDesc, char* upMessage)
{
	char uplinkdata[1024 * 3];
	char Buff[128];
	bool preliminary = false;
	bool scrubbed = false;

	switch (fcn)
	{
	case 1: //MISSION INITIALIZATION
	{
		//P80 MED: mission initialization
		mcc->mcc_calcs.PrelaunchMissionInitialization();

		//P10 MED: Predicted liftoff time

		double iptr, gmt, ss;
		int hh, mm;
		gmt = modf(oapiGetSimMJD(), &iptr) * 24.0 * 3600.0 + 4.0 * 3600.0; //TBD: Not sure this is good

		OrbMech::SStoHHMMSS(gmt, hh, mm, ss, 0.01);

		sprintf_s(Buff, "P10,CSM,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);

		//P15: CMC, clock zero
		sprintf_s(Buff, "P15,AGC,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);
	}
	break;
	case 10: //ON-ORBIT INITIALIZATION
	{
		//P10 MED: Enter actual liftoff time
		double tephem_scal;
		Saturn* cm = (Saturn*)calcParams.src;

		//Get TEPHEM
		tephem_scal = GetTEPHEMFromAGC(&cm->agc.vagc, true);
		double LaunchMJD = (tephem_scal / 8640000.) + SystemParameters.TEPHEM0;
		LaunchMJD = (LaunchMJD - SystemParameters.GMTBASE) * 24.0;

		int hh, mm;
		double ss;

		OrbMech::SStoHHMMSS(LaunchMJD * 3600.0, hh, mm, ss, 0.01);

		sprintf_s(Buff, "P10,CSM,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);

		//P12: CSM GRR and Azimuth
		Saturn1b* Sat1b = (Saturn1b*)cm;
		LVDC1B* lvdc = (LVDC1B*)Sat1b->iu->GetLVDC();
		double Azi = lvdc->Azimuth * DEG;
		double T_GRR = lvdc->T_GRR;

		sprintf_s(Buff, "P12,CSM,%d:%d:%.2lf,%.2lf;", hh, mm, ss, Azi);
		GMGMED(Buff);

		//P15: CMC, clock zero
		sprintf_s(Buff, "P15,AGC,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);

		//P12: IU GRR and Azimuth
		OrbMech::SStoHHMMSS(T_GRR, hh, mm, ss, 0.01);
		sprintf_s(Buff, "P12,IU1,%d:%d:%.2lf,%.2lf;", hh, mm, ss, Azi);
		GMGMED(Buff);

		//Get actual liftoff REFSMMAT from telemetry
		BZSTLM.CMC_REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		BZSTLM.CMCRefsPresent = true;
		EMSGSUPP(1, 1);
		//Make telemetry matrix current
		GMGMED("G00,CSM,TLM,CSM,CUR;");
	}
	break;
	case 11: //ACM preliminary update
	case 12: //ACM Final update
	{
		AP7MNV* form = (AP7MNV*)pad;
		AP7ManPADOpt manopt;
		VECTOR3 dV_LVLH, dV_imp;
		double P30TIG, TIG_imp, GET_TH;
		PMMMPTInput in;
		GMPOpt gmpopt;

		EphemerisData svCSM, svLEM;

		char buffer1[1000];
		char buffer2[1000];

		if (fcn == 11)
		{
			preliminary = true;
		}
		else
		{
			preliminary = false;
		}

		svCSM = StateVectorCalcEphem(calcParams.src);
		svLEM = StateVectorCalcEphem(calcParams.tgt);
		GET_TH = OrbMech::HHMMSSToSS(3, 0, 0);

		gmpopt.ManeuverCode = RTCC_GMP_CRA;
		gmpopt.sv_in.sv = svCSM;
		gmpopt.TIG_GET = GET_TH;

		GeneralManeuverProcessor(&gmpopt, dV_imp, TIG_imp);

		in.CONFIG = 1; //CSM
		in.CSMWeight = calcParams.src->GetMass();
		in.sv_before = PZGPMELM.SV_before;
		in.V_aft = PZGPMELM.V_after;
		in.DETU = 20.0; //Ullage
		in.UT = false; //2 jets
		in.IgnitionTimeOption = false;
		in.IterationFlag = false;
		in.LMWeight = 0.0;
		in.Thruster = RTCC_ENGINETYPE_CSMSPS;
		in.VC = RTCC_MANVEHICLE_CSM;
		in.VehicleArea = PZMPTCSM.ConfigurationArea;
		in.HeadsUpIndicator = true;

		double GMT_TIG;
		PoweredFlightProcessor(in, GMT_TIG, dV_LVLH);
		P30TIG = GETfromGMT(GMT_TIG);

		manopt.TIG = P30TIG;
		manopt.dV_LVLH = dV_LVLH;
		manopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		manopt.HeadsUp = false;
		manopt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		manopt.navcheckGET = 0.0;
		manopt.sxtstardtime = 0.0;
		manopt.UllageDT = 20.0;
		manopt.UllageThrusterOpt = false;
		manopt.sv0 = svCSM;
		manopt.WeightsTable = GetWeightsTable(calcParams.src, true, true);

		AP7ManeuverPAD(manopt, *form);
		sprintf(form->purpose, "ACM");

		if (!preliminary)
		{

			AGCStateVectorUpdate(buffer1, 1, RTCC_MPT_CSM, svCSM);
			AGCStateVectorUpdate(buffer2, 1, RTCC_MPT_LM, svLEM);

			sprintf(uplinkdata, "%s%s", buffer1, buffer2);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM & Soyuz state vectors");
			}
		}
	}
	break;
	}
	return scrubbed;
}