/****************************************************************************
This file is part of Project Apollo - NASSP

MCC sequencing for Skylab missions

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

#include "mcc.h"
#include "rtcc.h"
#include "MCC_Mission_ASTP.h"

void MCC::MissionSequence_ASTP()
{
	switch (MissionState)
	{
	case MST_ASTP_PRELAUNCH: //Scenario start to Insertion
		UpdateMacro(UTP_NONE, PT_NONE, mcc_calcs.GETEval(-16.0 * 60.0), 1, MST_1B_PRELAUNCH);
		break;
	case MST_ASTP_INSERTION: //Insertion to ACM Prelim update
		UpdateMacro(UTP_NONE, PT_NONE, mcc_calcs.GETEval(3 * 3600.0), 10, MST_ASTP_ACM_PRELIM);
		break;
	case MST_ASTP_ACM_PRELIM: //ACM Prelim to ACM Final update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP7MNV, mcc_calcs.GETEval(3 * 3600.0 + 24 * 60), 11, MST_ASTP_ACM_FINAL);
		break;
	case MST_ASTP_ACM_FINAL: //ACM Final to NC1 Preliminary update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP7MNV, mcc_calcs.GETEval(4 * 3600.0 + 24 * 60), 12, MST_ASTP_NC1_PRELIM);
		break;
	case MST_ASTP_NC1_PRELIM: //NC1 Prelim to NC1 Final update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_SLMNV, mcc_calcs.GETEval(4 * 3600.0 + 45 * 60), 13, MST_ASTP_NC1_FINAL);
		break;
	case MST_ASTP_NC1_FINAL: //NC1 Final
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_SLMNV, mcc_calcs.GETEval(5 * 3600.0 + 48 * 60), 14, MST_ASTP_CSM_WT);
		break;
	case MST_ASTP_CSM_WT: //CSM Alone weight update
		UpdateMacro(UTP_PADONLY, PT_AP10DAPDATA, mcc_calcs.GETEval(6 * 3600.0 + 18 * 60), 15, MST_ASTP_RNDZ_REFSMMAT);
		break;
	case MST_ASTP_RNDZ_REFSMMAT: //Rendezvous REFSMMAT
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, mcc_calcs.GETEval(7 * 3600.0 + 45 * 60), 16, MST_ASTP_LIFTOFF_UPDATE);
		break;
	case MST_ASTP_LIFTOFF_UPDATE: //Liftoff Time update -7h 30min & ATS S.V.
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, mcc_calcs.GETEval(25 * 3600.0 + 41 * 60), 17, MST_ASTP_CSM_ATS_SV);
		break;
	case MST_ASTP_CSM_ATS_SV: //CSM & ATS SV Update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, mcc_calcs.GETEval(29 * 3600.0 + 45 * 60), 18, MST_ASTP_BLOCK_33_48);
		break;
	case MST_ASTP_BLOCK_33_48: //Block Data revs 33 & 48
		UpdateMacro(UTP_PADONLY, PT_AP7BLK, mcc_calcs.GETEval(29 * 3600.0 + 50 * 60), 19, MST_ASTP_PCM_PRELIM);
		break;
	}
}