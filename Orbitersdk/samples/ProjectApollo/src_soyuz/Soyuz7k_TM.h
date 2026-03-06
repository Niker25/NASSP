/**************************************************************************
  This file is part of Project Apollo - NASSP


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

  See https://github.com/orbiternassp/NASSP/blob/Orbiter2016/NASSP-LICENSE.txt
  for more details.

**************************************************************************/

#if !defined(_PA_SOYUZ_H)
#define _PA_SOYUZ_H

#include "Orbitersdk.h"
#include "PanelSDK/PanelSDK.h"
#include "powersource.h"
#include "soundlib.h"
#include "SoyuzConnector.h"
#include "nasspdefs.h"


class Soyuz7k_TM : public ProjectApolloConnectorVessel {
public:
	Soyuz7k_TM(OBJHANDLE hVessel, int flightmodel);
	virtual ~Soyuz7k_TM();
	void clbkPostCreation();
	void clbkSetClassCaps(FILEHANDLE cfg);
	void clbkPreStep(double simt, double simdt, double mjd);
	void clbkSaveState(FILEHANDLE scn);
	void clbkLoadStateEx(FILEHANDLE scn, void* status);
	int clbkConsumeBufferedKey(DWORD key, bool down, char* kstate);
	void clbkFocusChanged(bool getfocus, OBJHANDLE hNewVessel, OBJHANDLE hOldVessel);
	void clbkGetRadiationForce(const VECTOR3& mflux, VECTOR3& F, VECTOR3& pos);
	void CalcApses();
	void CalcCircular();
	void CalcIBurn();
	void ArmAutoBurn();
	void APTimeStep(double simt);
	bool clbkDrawHUD(int mode, const HUDPAINTSPEC* hps, oapi::Sketchpad* skp);
	virtual void ExecuteCirc();
	THRUSTER_HANDLE th_main;

private:
	ATTACHMENTHANDLE hAttach;
	DOCKHANDLE hDock;
	MESHHANDLE hT_AF_PAO, hTM_BO, hOK_SA, hok_vc, hOK_cabling;
	BEACONLIGHTSPEC blinklights[2], attlights[4];
	VECTOR3 blinklights_pos[2], attlights_pos[4];
	bool blinklightsActive, attlightsActive;
	double visibilitySize;
	PROPELLANT_HANDLE hpPAO;
	PROPELLANT_HANDLE hpSA;

	bool IsArmed = false;
	bool IsEngaged = false;
	bool IsCircular = false;
	bool ManualAbort = false;

	double dv = 0.0;
	double IBurn = 0.0;
	double IBurn2 = 0.0;

	double EReference = 0.0;
	double ECutoff = 0.0;

	double mu, a, e;
	double IPeri, IApo;
	double Rperi, Rapo;

	int cam_status, gc_cam_status, hud_dsp;
};


#endif