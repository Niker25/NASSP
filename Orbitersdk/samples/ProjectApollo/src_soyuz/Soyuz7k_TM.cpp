// ==============================================================
//
// Module for Soyuz-7k vessel class
//
// ==============================================================

#define ORBITER_MODULE

// To force Orbitersdk.h to use <fstream> in any compiler version
#pragma include_alias( <fstream.h>, <fstream> )
#include "Orbitersdk.h"

// Vessel Parameters
class Soyuz7k_TM : public VESSEL4 {
public:
	Soyuz7k_TM(OBJHANDLE hVessel, int flightmodel);
	~Soyuz7k_TM();
	void clbkPostCreation();
	void clbkSetClassCaps(FILEHANDLE cfg);
	void clbkPreStep(double simt, double simdt, double mjd);
	void clbkSaveState(FILEHANDLE scn);
	void clbkLoadStateEx(FILEHANDLE scn, void* status);
	int clbkConsumeBufferedKey(DWORD key, bool down, char* kstate);
	void clbkFocusChanged(bool getfocus, OBJHANDLE hNewVessel, OBJHANDLE hOldVessel);
	void clbkGetRadiationForce(const VECTOR3& mflux, VECTOR3& F, VECTOR3& pos);

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
};

// Vessel functions
Soyuz7k_TM::Soyuz7k_TM(OBJHANDLE hVessel, int flightmodel)
	: VESSEL4(hVessel, flightmodel)
{
}

Soyuz7k_TM::~Soyuz7k_TM()
{
}

void Soyuz7k_TM::clbkPostCreation()
{
	visibilitySize = 31.1; //Tuned so Soyuz disappears in the CSM optics at 400nm range

	if (oapiGetFocusObject() == GetHandle()) { SetSize(10); }
	else { SetSize(visibilitySize); }
}

void Soyuz7k_TM::clbkPreStep(double simt, double simdt, double mjd)
{
	//sprintf(oapiDebugString(), "size %0.1f", GetSize());
}

void Soyuz7k_TM::clbkSetClassCaps(FILEHANDLE cfg)
{
	blinklightsActive = false;
	attlightsActive = false;

	//General Settings
	SetEmptyMass(7055.0);
	SetSize(10);
	SetPMI(_V(4.34, 6.19, 2.58));
	SetCrossSections(_V(5.94, 16.73, 4.68));
	SetRotDrag(_V(0.3, 0.3, 1.5));

	//Meshes
	hT_AF_PAO = oapiLoadMesh("Soyuz/T_AF_PAO");
	hTM_BO = oapiLoadMesh("Soyuz/TM_BO");
	hOK_SA = oapiLoadMesh("Soyuz/OK_SA");
	hOK_cabling = oapiLoadMesh("Soyuz/OK_cabling");

	VECTOR3 vc_off = _V(0, -1.5, 0);

	SetMeshVisibilityMode(AddMesh(hT_AF_PAO), MESHVIS_EXTERNAL);
	SetMeshVisibilityMode(AddMesh(hTM_BO), MESHVIS_EXTERNAL);
	SetMeshVisibilityMode(AddMesh(hOK_SA), MESHVIS_EXTERNAL);
	SetMeshVisibilityMode(AddMesh(hOK_cabling), MESHVIS_EXTERNAL);

	//Docks
	hDock = CreateDock(_V(0.1, -0.05, 3.72), _V(0, 0, 1), _V(-0.669131, -0.743145, 0));

	//Attachments
	hAttach = CreateAttachment(true, _V(0, 0, -3.5), _V(0, 0, -1), _V(1, 0, 0), "Soyuz");

	//Propellant
	hpPAO = CreatePropellantResource(500, 500);
	hpSA = CreatePropellantResource(40, 40);

	//Exhaust textures
	SURFHANDLE atrcs = oapiRegisterExhaustTexture("dragon1/Dragon_atrcs");

	//Thrusters definition
	THRUSTER_HANDLE th_main = CreateThruster(_V(0, 0, -3.35), _V(0, 0, 1), 4090.0, hpPAO, 2763.6);

	THRUSTER_HANDLE th_fwd[4];
	th_fwd[0] = CreateThruster(_V(0, -0.375, -3.45), _V(0, 0, 1), 62.5, hpPAO, 1e7);
	th_fwd[1] = CreateThruster(_V(0, 0.375, -3.45), _V(0, 0, 1), 62.5, hpPAO, 1e7);
	th_fwd[2] = CreateThruster(_V(0.375, 0, -3.45), _V(0, 0, 1), 62.5, hpPAO, 1e7);
	th_fwd[3] = CreateThruster(_V(-0.375, 0, -3.45), _V(0, 0, 1), 62.5, hpPAO, 1e7);

	THRUSTER_HANDLE th_back[2];
	th_back[0] = CreateThruster(_V(-1.15, 0, -0.85), _V(0.410365, 0, -0.911921), 125.0, hpPAO, 1e7);
	th_back[1] = CreateThruster(_V(1.15, 0, -0.85), _V(-0.410365, 0, -0.911921), 125.0, hpPAO, 1e7);

	THRUSTER_HANDLE th_translate_left = CreateThruster(_V(0, 0, 0), _V(-1, 0, 0), 1129.12, hpPAO, 1e7);

	THRUSTER_HANDLE th_translate_right = CreateThruster(_V(0, 0, 0), _V(1, 0, 0), 1129.12, hpPAO, 1e7);

	THRUSTER_HANDLE th_translate_up = CreateThruster(_V(0, 0, 0), _V(0, 1, 0), 1129.12, hpPAO,1e7);

	THRUSTER_HANDLE th_translate_down = CreateThruster(_V(0, 0, 0), _V(0, -1, 0), 1129.12, hpPAO, 1e7);

	THRUSTER_HANDLE th_pitch_up[2];
	th_pitch_up[0] = CreateThruster(_V(0, 0, 10), _V(0, 1, 0), 85.5279, hpPAO, 1e7);
	th_pitch_up[1] = CreateThruster(_V(0, 0, -10), _V(0, -1, 0), 85.5279, hpPAO, 1e7);

	THRUSTER_HANDLE th_pitch_dn[2];
	th_pitch_dn[0] = CreateThruster(_V(0, 0, 10), _V(0, -1, 0), 85.5279, hpPAO, 1e7);
	th_pitch_dn[1] = CreateThruster(_V(0, 0, -10), _V(0, 1, 0), 85.5279, hpPAO, 1e7);

	THRUSTER_HANDLE th_yaw_l[2];
	THRUSTER_HANDLE th_yaw_r[2];
	th_yaw_l[0] = CreateThruster(_V(0, 0, 10), _V(-1, 0, 0), 121.986, hpPAO, 1e7);
	th_yaw_l[1] = CreateThruster(_V(0, 0, -10), _V(1, 0, 0), 121.986, hpPAO, 1e7);
	th_yaw_r[0] = CreateThruster(_V(0, 0, 10), _V(1, 0, 0), 121.986, hpPAO, 1e7);
	th_yaw_r[1] = CreateThruster(_V(0, 0, -10), _V(-1, 0, 0), 121.986, hpPAO, 1e7);

	THRUSTER_HANDLE th_bank_l[2];
	THRUSTER_HANDLE th_bank_r[2];
	th_bank_l[0] = CreateThruster(_V(10, 0, 0), _V(0, 1, 0), 50.84, hpPAO, 1e7);
	th_bank_l[1] = CreateThruster(_V(-10, 0, 0), _V(0, -1, 0), 50.84, hpPAO, 1e7);
	th_bank_r[0] = CreateThruster(_V(10, 0, 0), _V(0, -1, 0), 50.84, hpPAO, 1e7);
	th_bank_r[1] = CreateThruster(_V(-10, 0, 0), _V(0, 1, 0), 50.84, hpPAO, 1e7);

	//Thruster groups
	CreateThrusterGroup(&th_main, 1, THGROUP_MAIN);

	CreateThrusterGroup(th_fwd, 4, THGROUP_ATT_FORWARD);
	CreateThrusterGroup(th_back, 2, THGROUP_ATT_BACK);

	CreateThrusterGroup(&th_translate_left, 1, THGROUP_ATT_LEFT);
	CreateThrusterGroup(&th_translate_right, 1, THGROUP_ATT_RIGHT);

	CreateThrusterGroup(&th_translate_up, 1, THGROUP_ATT_UP);
	CreateThrusterGroup(&th_translate_down, 1, THGROUP_ATT_DOWN);

	CreateThrusterGroup(th_pitch_up, 2, THGROUP_ATT_PITCHUP);
	CreateThrusterGroup(th_pitch_dn, 2, THGROUP_ATT_PITCHDOWN);

	CreateThrusterGroup(th_yaw_l, 2, THGROUP_ATT_YAWLEFT);
	CreateThrusterGroup(th_yaw_r, 2, THGROUP_ATT_YAWRIGHT);

	CreateThrusterGroup(th_bank_l, 2, THGROUP_ATT_BANKLEFT);
	CreateThrusterGroup(th_bank_r, 2, THGROUP_ATT_BANKRIGHT);

	//Exhaust definitions
	AddExhaust(th_main, 5.00, 0.50, atrcs);

	for (int i = 0; i < 4; i++) {
		AddExhaust(th_fwd[i], 5.00, 0.25, atrcs);
	}
	AddExhaust(th_back[0], 5.00, 0.25, _V(-1.15, 0, -0.85), _V(-0.41, 0, 0.912), atrcs);
	AddExhaust(th_back[1], 5.00, 0.25, _V(1.15, 0, -0.85), _V(0.41, 0, 0.912), atrcs);

	AddExhaust(th_translate_left, 5.00, 0.25, _V(0.875, 0.82, -1.11), _V(0.762, 0.648, 0), atrcs);
	AddExhaust(th_translate_left, 5.00, 0.25, _V(0.875, -0.82, -1.11), _V(0.762, -0.648, 0), atrcs);

	AddExhaust(th_translate_right, 5.00, 0.25, _V(-0.875, 0.82, -1.11), _V(-0.762, 0.648, 0), atrcs);
	AddExhaust(th_translate_right, 5.00, 0.25, _V(-0.875, -0.82, -1.11), _V(-0.762,- 0.648, 0), atrcs);

	AddExhaust(th_translate_up, 5.00, 0.25, _V(0.0, -1.2, -1.15), _V(0, -1, 0), atrcs);
	AddExhaust(th_translate_down, 5.00, 0.25, _V(0.0, 1.2, -1.15), _V(0, 1, 0), atrcs);

	AddExhaust(th_pitch_up[0], 5.00, 0.25, _V(0, 1.12, -2.93), _V(0, 1, 0), atrcs);
	AddExhaust(th_pitch_dn[0], 5.00, 0.25, _V(0, -1.12, -2.93), _V(0, -1, 0), atrcs);

	AddExhaust(th_yaw_l[0], 5.00, 0.25, _V(-1.15, 0, -0.85), _V(-0.41, 0, 0.912), atrcs);
	AddExhaust(th_yaw_r[0], 5.00, 0.25, _V(1.15, 0, -0.85), _V(0.41, 0, 0.912), atrcs);

	AddExhaust(th_bank_l[0], 5.00, 0.25, _V(0.875, 0.82, -1.11), _V(0.762, 0.648, 0), atrcs);
	AddExhaust(th_bank_l[1], 5.00, 0.25, _V(-0.875, -0.82, -1.11), _V(-0.762, -0.648, 0), atrcs);

	AddExhaust(th_bank_r[0], 5.00, 0.25, _V(0.875, -0.82, -1.11), _V(0.762, -0.648, 0), atrcs);
	AddExhaust(th_bank_r[1], 5.00, 0.25, _V(-0.875, 0.82, -1.11), _V(-0.762, 0.648, 0), atrcs);

	//Beacons def
	blinklights_pos[0] = _V(0, 1.20061, -1.72033);
	blinklights_pos[1] = _V(0, -1.12247, -1.80755);

	attlights_pos[0] = _V(-5.04453, -0.01211, -1.22717);
	attlights_pos[1] = _V(-5.00324, -0.011685, -3.06356);

	attlights_pos[2] = _V(5.04453, -0.01211, -1.22717);
	attlights_pos[3] = _V(5.00324, -0.011685, -3.06356);

	static VECTOR3 whiteCol = _V(1, 1, 1);
	static VECTOR3 attCol[3] = {
		{ 0.5, 1.0, 0.5 }, //Green
		{ 1.0, 0.5, 0.5 }, //Red
		{ 1, 1, 1 }, //White

	};

	for (int i = 0; i < 2; i++) {
		blinklights[i].shape = BEACONSHAPE_STAR; 
		blinklights[i].pos = &blinklights_pos[i];
		blinklights[i].col = &whiteCol;
		blinklights[i].size = 0.25;
		blinklights[i].falloff = 0.5;
		blinklights[i].period = 1.0;
		blinklights[i].duration = 0.1;
		blinklights[i].tofs = 0;
		blinklights[i].active = false;

		AddBeacon(blinklights + i);
	}

	for (int i = 0; i < 4; i++) {
		attlights[i].shape = BEACONSHAPE_STAR;
		attlights[i].pos = &attlights_pos[i];
		attlights[i].col = (i == 0) ? attCol : (i == 2) ? attCol + 1 : attCol + 2;
		attlights[i].size = 0.1;
		attlights[i].falloff = 0.5;
		attlights[i].period = 0;
		attlights[i].duration = 1.0;
		attlights[i].tofs = 0;
		attlights[i].active = false;

		AddBeacon(attlights + i);
	}
}

int Soyuz7k_TM::clbkConsumeBufferedKey(DWORD key, bool down, char* kstate)
{
	if (!down) return 0; // only process keydown events

	if (KEYMOD_SHIFT(kstate))
	{
		switch (key)
		{
		}
	}
	else if (KEYMOD_ALT(kstate))
	{
		switch (key)
		{
		case OAPI_KEY_L:
			if (attlightsActive == false)
			{
				for (int i = 0; i < 4; i++)
				{
					attlights[i].active = true;
					attlightsActive = true;
				}
			}
			else if (attlightsActive == true)
			{
				for (int i = 0; i < 4; i++)
				{
					attlights[i].active = false;
					attlightsActive = false;
				}
			}
			return 1;
		}
	}
	else if (KEYMOD_CONTROL(kstate))
	{
		switch (key)
		{
		case OAPI_KEY_L:
			if (blinklightsActive == false)
			{
				for (int i = 0; i < 2; i++)
				{
					blinklights[i].active = true;
					blinklightsActive = true;
				}
			}
			else if (blinklightsActive == true)
			{
				for (int i = 0; i < 2; i++)
				{
					blinklights[i].active = false;
					blinklightsActive = false;
				}
			}
			return 1;
		}
	}
	else { // unmodified keys
		switch (key)
		{
		}
	}
	return 0;
}

// ==============================================================
// API callback interface
// ==============================================================
// --------------------------------------------------------------
// Module initialisation
// --------------------------------------------------------------
DLLCLBK void InitModule(HINSTANCE hModule)
{
}

DLLCLBK void ExitModule(HINSTANCE hModule)
{
}

// --------------------------------------------------------------
// Vessel initialisation
// --------------------------------------------------------------
DLLCLBK VESSEL* ovcInit(OBJHANDLE hvessel, int flightmodel)
{
	return new Soyuz7k_TM(hvessel, flightmodel);

}

// --------------------------------------------------------------
// Vessel cleanup
// --------------------------------------------------------------
DLLCLBK void ovcExit(VESSEL* vessel)
{
	if (vessel) delete (Soyuz7k_TM*)vessel;

}

void Soyuz7k_TM::clbkSaveState(FILEHANDLE scn)
{
	char cbuf[256];
	VESSEL4::clbkSaveState(scn);
	sprintf(cbuf, "%d", blinklightsActive);
	oapiWriteScenario_string(scn, "BLINK", cbuf);

	sprintf(cbuf, "%d", attlightsActive);
	oapiWriteScenario_string(scn, "ATT", cbuf);
}

void Soyuz7k_TM::clbkLoadStateEx(FILEHANDLE scn, void* vs)
{
	char* line;
	while (oapiReadScenario_nextline(scn, line)) 
	{
		if (!_strnicmp(line, "BLINK", 5))
		{
			sscanf(line + 5, "%d", &blinklightsActive);
		}
		else if (!_strnicmp(line, "ATT", 3))
		{
			sscanf(line + 3, "%d", &attlightsActive);
		}
		else
		{
			ParseScenarioLineEx(line, vs);
		}
	}

	if (oapiGetFocusObject() == GetHandle()) { SetSize(10); }
	else { SetSize(visibilitySize); }

	if (blinklightsActive == true)
	{
		for (int i = 0; i < 2; i++)
		{
			blinklights[i].active = true;
		}
	}

	if (attlightsActive == true)
	{
		for (int i = 0; i < 4; i++)
		{
			attlights[i].active = true;
		}
	}
}

void Soyuz7k_TM::clbkFocusChanged(bool getfocus, OBJHANDLE hNewVessel, OBJHANDLE hOldVessel)
{
	OBJHANDLE hSoyuz = GetHandle();
	if (hNewVessel == hSoyuz) { //Soyuz gains focus

		bool fixCamera = false;
		if (oapiCameraInternal() == false) {
			fixCamera = true;
			oapiCameraAttach(hSoyuz, 0);
		}

		SetSize(10);

		if (fixCamera == true) {
			oapiCameraAttach(hSoyuz, 1);
		}
	}
	else if (hOldVessel == hSoyuz) { //Soyuz loses focus
		SetSize(visibilitySize);
	}
}

void Soyuz7k_TM::clbkGetRadiationForce(const VECTOR3& mflux, VECTOR3& F, VECTOR3& pos)
{
	double size = 10;
	double cs = size * size;  // simplified cross section
	double albedo = 1.5;    // simplistic albedo (mixture of absorption, reflection)

	F = mflux * (cs * albedo);
	pos = _V(0, 0, 0);        // don't induce torque
}