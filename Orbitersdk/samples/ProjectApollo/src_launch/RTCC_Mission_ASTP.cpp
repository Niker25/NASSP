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

		//P10 MED: Soyuz liftoff time
		GMGMED("P10,LEM,12:20:00.0;");

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

		//P10 Enter CSM & Soyuz Liftoff time
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
		in.CSMWeight = calcParams.src->GetMass() + 1740.888; //CSM + DM Weight
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

		if (preliminary)
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
		else
		{
			CMCExternalDeltaVUpdate(buffer1, P30TIG, dV_LVLH);
			sprintf(uplinkdata, "%s", buffer1);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "ACM Target load");
			}
		}
	}
	break;
	case 13: //NC1 preliminary update
	case 14: //NC1 final update
	{
		SLMNV* form = (SLMNV*)pad;
		AP7ManPADOpt manopt;
		VECTOR3 dV_LVLH_LEM, dV_imp;
		double LEMCircTIG, TIG_imp, GET_TH;
		PMMMPTInput in;
		GMPOpt gmpopt;
		EphemerisData svCSM, svLEM, sv_bco, sv_aco, sv_nc1, sv_nh, sv_ncc, sv_nsr, sv_tpi;
		SV sv_circ, sv_soyuz;
		PLAWDTOutput WeightsTableLEM, WeightsTable2LEM, WeightsTableCSM, WeightsTable2CSM;
		DKIOpt dki;

		char buffer1[1000];
		char buffer2[1000];

		if (fcn == 13)
		{
			preliminary = true;
		}
		else
		{
			preliminary = false;
		}

		if (PZMPTCSM.ManeuverNum > 0 || PZMPTLEM.ManeuverNum > 0)
		{
			//Delete maneuvers from MPT
			GMGMED("M62,CSM,1,D;");
			GMGMED("M62,LEM,1,D;");
		}

		//GPM Caclulation
		svLEM = StateVectorCalcEphem(calcParams.tgt);
		GET_TH = OrbMech::HHMMSSToSS(16, 0, 0);

		gmpopt.ManeuverCode = RTCC_GMP_CRA;
		gmpopt.sv_in.sv = svLEM;
		gmpopt.TIG_GET = GET_TH;

		GeneralManeuverProcessor(&gmpopt, dV_imp, TIG_imp);

		//PFP
		in.CONFIG = 12; //LM
		in.CSMWeight = 0.0;
		in.DETU = 0;
		in.DPSScaleFactor = 1;
		in.DT_10PCT = -1.0;
		in.HeadsUpIndicator = false;
		in.IgnitionTimeOption = false;
		in.IterationFlag = false; //Could be true
		in.LMWeight = calcParams.tgt->GetMass();
		in.sv_before = PZGPMELM.SV_before;
		in.Thruster = RTCC_ENGINETYPE_LMDPS;
		in.UT = false; //2 Jets
		in.VC = RTCC_MANVEHICLE_LM;
		in.VehicleArea = PZMPTLEM.ConfigurationArea;
		in.VehicleWeight = calcParams.tgt->GetMass();
		in.V_aft = PZGPMELM.V_after;

		double GMT_TIG;
		PoweredFlightProcessor(in, GMT_TIG, dV_LVLH_LEM);
		LEMCircTIG = GETfromGMT(GMT_TIG);

		sv_soyuz = StateVectorCalc(calcParams.tgt);
		sv_circ = ExecuteManeuver(sv_soyuz, LEMCircTIG, dV_LVLH_LEM, calcParams.tgt->GetMass(), RTCC_ENGINETYPE_LMDPS);

		////CSM Config and mass update
		//med_m55.Table = RTCC_MPT_CSM;
		//MPTMassUpdate(calcParams.src, med_m50, med_m55, med_m49);
		//PMMWTC(55);
		//med_m50.Table = RTCC_MPT_CSM;
		//med_m50.WeightGET = GETfromGMT(RTCCPresentTimeGMT());
		//PMMWTC(50);

		////CSM Trajectory Update
		//StateVectorTableEntry sv0;
		//sv0.Vector = StateVectorCalcEphem(calcParams.src);
		//sv0.LandingSiteIndicator = false;
		//sv0.VectorCode = "APIC001";

		//PMSVCT(4, RTCC_MPT_CSM, sv0);

		////LEM Config and mass update
		//med_m55.Table = RTCC_MPT_LM;
		//MPTMassUpdate(calcParams.tgt, med_m50, med_m55, med_m49);
		//PMMWTC(55);
		//med_m50.Table = RTCC_MPT_LM;
		//med_m50.WeightGET = GETfromGMT(RTCCPresentTimeGMT());
		//PMMWTC(50);

		////LEM Trajectory Initialization
		//StateVectorTableEntry sv1;
		//sv1.Vector = StateVectorCalcEphem(calcParams.tgt);
		//sv1.LandingSiteIndicator = false;
		//sv1.VectorCode = "APIL001";

		//PMSVCT(4, RTCC_MPT_LM, sv1);

		////Add Soyuz Circ maneuver to MPT
		//char buf[256];
		//sprintf(buf, "M40,P4,%f,%f,%f;",
		//	dV_LVLH_LEM.x * 3.281,
		//	dV_LVLH_LEM.y * 3.281,
		//	dV_LVLH_LEM.z * 3.281);
		//GMGMED(buf);

		//med_m66.Table = RTCC_MPT_LM;
		//med_m66.ReplaceCode = 0; //Don't replace
		//med_m66.DPSThrustFactor = 1;
		//med_m66.TenPercentDT = -1;
		//med_m66.GETBI = LEMCircTIG;
		//med_m66.Thruster = RTCC_ENGINETYPE_LMDPS;
		//med_m66.BurnParamNo = 4;
		//med_m66.ConfigChangeInd = RTCC_CONFIGCHANGE_NONE;

		////Dummy data
		//std::vector<std::string> str;
		//PMMMED("66", str);


		//GMGMED("U02,LEM,GET,17:00:00,,ECI"); //Checkout Soyuz cutoff vector

		////Get Soyuz vector from checkout monitor
		//double x = EZCHECKDIS.Pos.x;
		//double y = EZCHECKDIS.Pos.y;
		//double z = EZCHECKDIS.Pos.z;

		//double xDOT = EZCHECKDIS.Vel.x;
		//double yDOT = EZCHECKDIS.Vel.y;
		//double zDOT = EZCHECKDIS.Vel.z;

		//double GET = EZCHECKDIS.GET;

		//double ss;
		//int hh, mm;
		//OrbMech::SStoHHMMSS(GET, hh, mm, ss, 0.01);

		//sprintf(buf,
		//	"P14,LEM,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%02d:%02d:%05.2f,L,ECI;",
		//	x, y, z,
		//	xDOT, yDOT, zDOT,
		//	hh, mm, ss);
		//GMGMED(buf);

		//NC1 Calculation
		double NC1_THR = 5*3600 + 41*60 + 28;
		svCSM = StateVectorCalcEphem(calcParams.src);
		dki.sv_CSM = svCSM;
		dki.sv_LM = ConvertSVtoEphemData(sv_circ);
		dki.MV = 1; //CSM maneuvers

		dki.I4 = true;
		dki.IPUTNA = 1; //Maneuver line at chaser apogee
		dki.PUTTNA = GMTfromGET(NC1_THR); //THR Time at PET 05:00:00
		dki.PUTNA = 1.0;
		dki.K46 = 4; //TPI at X minutes into day
		dki.TIMLIT = -19.0;
		dki.DHNCC = 20.0 * 1852.0;
		dki.DHSR = 10.0 * 1852.0;
		dki.Elev = 27.0 * RAD;
		dki.dt_NCC_NSR = 37.0 * 60.0;
		dki.NC1 = 1.0;
		dki.NH = 25;
		dki.NCC = 25.5;
		dki.MI = 27.0;
		dki.WT = 130.0 * RAD;
		dki.NPC = -1.0;

		DockingInitiationProcessor(dki);

		calcParams.Phasing = PZREDT.GET[0]; //NC1 time
		calcParams.Insertion = PZREDT.GET[1]; //NC2 time
		calcParams.CSI = PZREDT.GET[2]; //NCC time
		calcParams.CDH = PZREDT.GET[3]; //NSR time
		calcParams.TPI = PZRPDT.data[0].GETTPI; //TPI time

		mcc->mcc_calcs.StoreStateVector(PZDKIELM.Block[0].SV_before[4]); //Save TPI vector for RNVZ REFSMMAT

		manopt.TIG = calcParams.Phasing;
		manopt.dV_LVLH = PZREDT.DVVector[0] * 0.3048;
		manopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		manopt.HeadsUp = false;
		manopt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		manopt.navcheckGET = 0.0;
		manopt.sxtstardtime = 0.0;
		manopt.UllageDT = 20.0;
		manopt.UllageThrusterOpt = false;
		manopt.sv0 = dki.sv_CSM;
		manopt.WeightsTable = GetWeightsTable(calcParams.src, true, true);

		SLManeuverPAD(manopt, *form);

		form->type = 1;
		form->prelim = preliminary;

		if (preliminary)
		{
			char buffer1[1000];

			AGCStateVectorUpdate(buffer1, 1, RTCC_MPT_CSM, dki.sv_CSM);

			sprintf(uplinkdata, "%s", buffer1);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM State Vector");
			}
		}
		else
		{
			CMCExternalDeltaVUpdate(buffer1, calcParams.Phasing, PZREDT.DVVector[0] * 0.3048);
			sprintf(uplinkdata, "%s", buffer1);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "NC1 Target load");
			}
		}

			////Execute all RNVZ maneuvers until TPI
			//WeightsTableCSM = GetWeightsTable(calcParams.src, true, true);
			//ExecuteManeuver(dki.sv_CSM, WeightsTableCSM, calcParams.Phasing, PZREDT.DVVector[0] * 0.3048, RTCC_ENGINETYPE_CSMSPS, sv_nc1, WeightsTable2CSM);
			//ExecuteManeuver(sv_nc1, WeightsTable2CSM, calcParams.Insertion, PZREDT.DVVector[1] * 0.3048, RTCC_ENGINETYPE_CSMSPS, sv_nh, WeightsTable2CSM);
			//ExecuteManeuver(sv_nh, WeightsTable2CSM, calcParams.CSI, PZREDT.DVVector[3] * 0.3048, RTCC_ENGINETYPE_CSMSPS, sv_ncc, WeightsTable2CSM);
			//ExecuteManeuver(sv_ncc, WeightsTable2CSM, calcParams.CDH, PZREDT.DVVector[4] * 0.3048, RTCC_ENGINETYPE_CSMSPS, sv_nsr, WeightsTable2CSM);
			//ExecuteManeuver(sv_nsr, WeightsTable2CSM, calcParams.TPI, PZREDT.DVVector[5] * 0.3048, RTCC_ENGINETYPE_CSMSPS, sv_tpi, WeightsTable2CSM); //We obtain the CSM sv after TPI burn
	}
	break;
	case 15:
	{
		AP10DAPDATA* form = (AP10DAPDATA*)pad;

		CSMDAPUpdate(calcParams.src, *form, false, false);
	}
	break;
	case 16:
	{
		//Calculate RNVZ REFSMMAT
		REFSMMATOpt refs;
		MATRIX3 REFSMMAT, A;
		char buffer1[1000];

		refs.REFSMMATopt = 2;
		refs.REFSMMATTime = calcParams.TPI;
		refs.vessel = calcParams.src;
		refs.vesseltype = 1;
		refs.useSV = true;
		mcc->mcc_calcs.RestoreStateVector(refs.RV_MCC);

		REFSMMAT = REFSMMATCalc(&refs);
		EMGSTSTM(RTCC_MPT_CSM, REFSMMAT, RTCC_REFSMMAT_TYPE_CUR, refs.REFSMMATTime);

		AGCDesiredREFSMMATUpdate(buffer1, EZJGMTX1.data[0].REFSMMAT);

		sprintf(uplinkdata, "%s", buffer1);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "Rendezvous REFSMMAT");
		}
	}
	break;
	case 17: //Liftoff Time update -7h30min & ATS S.V.
	{
		//P10 MED: Enter updated liftoff time
		double tephem_scal;
		char buffer1[1000];
		char buffer2[1000];
		Saturn* cm = (Saturn*)calcParams.src;
		OBJHANDLE hATS = oapiGetVesselByName("ATS-6");
		VESSEL* ats = NULL;
		EphemerisData svATS;
		
		if (hATS) {
			ats = oapiGetVesselInterface(hATS);
		}

		//Get TEPHEM
		tephem_scal = GetTEPHEMFromAGC(&cm->agc.vagc, true);
		double LaunchMJD = (tephem_scal / 8640000.) + SystemParameters.TEPHEM0;
		LaunchMJD = (LaunchMJD - SystemParameters.GMTBASE) * 24.0;

		//Save original CSM Liftoff time
		double LaunchMJD_original = LaunchMJD;

		//Subtract 7h30
		double LaunchMJD_updated = LaunchMJD_original - 7.5;

		//dT between original and updated Liftoff time
		double LaunchMJD_diff = LaunchMJD_updated - LaunchMJD_original;

		int hh, mm;
		double ss;

		OrbMech::SStoHHMMSS(LaunchMJD_updated * 3600.0, hh, mm, ss, 0.01);

		sprintf_s(Buff, "P10,CSM,%d:%d:%.2lf;", hh, mm, ss); //P10 Update CSM Liftoff time
		GMGMED(Buff);
		sprintf_s(Buff, "P15,AGC,%d:%d:%.2lf;", hh, mm, ss); //P15: CMC, clock zero
		GMGMED(Buff);

		IncrementAGCLiftoffTime(buffer1, RTCC_MPT_CSM, LaunchMJD_diff * 3600);

		if (ats) {
			svATS = StateVectorCalcEphem(ats);
			AGCStateVectorUpdate(buffer2, 1, RTCC_MPT_LM, svATS);
		}

		sprintf(uplinkdata, "%s%s", buffer1, buffer2);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "Lift-off Time, ATS State vector");
		}
	}
	break;
	}
	return scrubbed;
}