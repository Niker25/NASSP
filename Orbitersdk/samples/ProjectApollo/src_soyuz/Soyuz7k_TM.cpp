// ==============================================================
//
// Module for Soyuz-7k vessel class
//
// ==============================================================

#define ORBITER_MODULE

// To force Orbitersdk.h to use <fstream> in any compiler version
#pragma include_alias( <fstream.h>, <fstream> )
#include "Orbitersdk.h"
#include "Soyuz7k_TM.h"

// Vessel functions
Soyuz7k_TM::Soyuz7k_TM(OBJHANDLE hObj, int fmodel) : ProjectApolloConnectorVessel(hObj, fmodel)
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
	th_main = CreateThruster(_V(0, 0, -3.35), _V(0, 0, 1), 4090.0, hpPAO, 2763.6);

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
//DLLCLBK void InitModule(HINSTANCE hModule)
//{
//}
//
//DLLCLBK void ExitModule(HINSTANCE hModule)
//{
//}

// --------------------------------------------------------------
// Vessel initialisation
// --------------------------------------------------------------
DLLCLBK VESSEL* ovcInit(OBJHANDLE hvessel, int flightmodel)
{
	VESSEL* soyuz;
	soyuz = new Soyuz7k_TM(hvessel, flightmodel);

	return soyuz;
}

// --------------------------------------------------------------
// Vessel cleanup
// --------------------------------------------------------------
DLLCLBK void ovcExit(VESSEL* vessel)
{
	if (vessel) {
		delete (Soyuz7k_TM*)vessel;
	}
}

void Soyuz7k_TM::clbkSaveState(FILEHANDLE scn)
{
	char cbuf[256];
	VESSEL4::clbkSaveState(scn);
	sprintf(cbuf, "%d", blinklightsActive);
	oapiWriteScenario_string(scn, "BLINK", cbuf);

	sprintf(cbuf, "%d", attlightsActive);
	oapiWriteScenario_string(scn, "ATT", cbuf);

	sprintf(cbuf, "%d", IsEngaged);
	oapiWriteScenario_string(scn, "ISENGAGED", cbuf);

	sprintf(cbuf, "%d", IsCircular);
	oapiWriteScenario_string(scn, "ISCIRCULAR", cbuf);

	sprintf(cbuf, "%d", ManualAbort);
	oapiWriteScenario_string(scn, "MANUALABORT", cbuf);

	sprintf(cbuf, "%d", IsPrograde);
	oapiWriteScenario_string(scn, "ISPROGRADE", cbuf);

	sprintf(cbuf, "%lf", dv);
	oapiWriteScenario_string(scn, "DV", cbuf);

	sprintf(cbuf, "%lf", IBurn);
	oapiWriteScenario_string(scn, "IBURN", cbuf);

	sprintf(cbuf, "%lf", IBurn2);
	oapiWriteScenario_string(scn, "IBURN2", cbuf);

	sprintf(cbuf, "%lf", ECutoff);
	oapiWriteScenario_string(scn, "ECUTOFF", cbuf);

	sprintf(cbuf, "%lf", IPeri);
	oapiWriteScenario_string(scn, "IPERI", cbuf);

	sprintf(cbuf, "%lf", IApo);
	oapiWriteScenario_string(scn, "IAPO", cbuf);

	sprintf(cbuf, "%lf", Rperi);
	oapiWriteScenario_string(scn, "RPERI", cbuf);

	sprintf(cbuf, "%lf", Rapo);
	oapiWriteScenario_string(scn, "RAPO", cbuf);

	sprintf(cbuf, "%lf", TIG);
	oapiWriteScenario_string(scn, "TIG", cbuf);

	sprintf(cbuf, "%d", IsArmed);
	oapiWriteScenario_string(scn, "ISARMED", cbuf);
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
		else if (!_strnicmp(line, "MANUALABORT", 11))
		{
			sscanf(line + 11, "%d", &ManualAbort);
		}
		else if (!_strnicmp(line, "ISCIRCULAR", 10))
		{
			sscanf(line + 10, "%d", &IsCircular);
		}
		else if (!_strnicmp(line, "ISPROGRADE", 10))
		{
			sscanf(line + 10, "%d", &IsPrograde);
		}
		else if (!_strnicmp(line, "ISENGAGED", 9))
		{
			sscanf(line + 9, "%d", &IsEngaged);
		}
		else if (!_strnicmp(line, "ECUTOFF", 7))
		{
			sscanf(line + 7, "%lf", &ECutoff);
		}
		else if (!_strnicmp(line, "IBURN2", 6))
		{
			sscanf(line + 6, "%lf", &IBurn2);
		}
		else if (!_strnicmp(line, "IBURN", 5))
		{
			sscanf(line + 5, "%lf", &IBurn);
		}
		else if (!_strnicmp(line, "IPERI", 5))
		{
			sscanf(line + 5, "%lf", &IPeri);
		}
		else if (!_strnicmp(line, "IAPO", 4))
		{
			sscanf(line + 4, "%lf", &IApo);
		}
		else if (!_strnicmp(line, "RPERI", 5))
		{
			sscanf(line + 5, "%lf", &Rperi);
		}
		else if (!_strnicmp(line, "RAPO", 4))
		{
			sscanf(line + 4, "%lf", &Rapo);
		}
		else if (!_strnicmp(line, "TIG", 3))
		{
			sscanf(line + 3, "%lf", &TIG);
		}
		else if (!_strnicmp(line, "DV", 2))
		{
			sscanf(line + 2, "%lf", &dv);
		}
		else if (!_strnicmp(line, "ISARMED", 7))
		{
			sscanf(line + 7, "%d", &IsArmed);
			char buf[128];
			sprintf(buf, "DEBUG: Loaded ISARMED = %d", IsArmed);
			oapiWriteLog(buf);
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

	char buf[128];
	sprintf(buf, "DEBUG: Loaded ISARMED after finish= %d", IsArmed);
	oapiWriteLog(buf);
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

void Soyuz7k_TM::clbkPreStep(double simt, double simdt, double mjd)
{
	APTimeStep(simt);
	ManeuverPrograde();
}

void Soyuz7k_TM::CalcApses()
{
	ELEMENTS el;
	double MJDRef;

	OBJHANDLE ref = GetElements(el, MJDRef);

	e = el.e;
	a = el.a;
	mu = oapiGetMass(ref) * GGRAV;

	double n = sqrt((e < 1 ? 1 : -1) * mu / (a * a * a));
	double M = el.L - el.omegab;

	double MJD = oapiTime2MJD(oapiGetSimTime());
	M += n * (MJD - MJDRef) * 86400;

	while (M < 0) M += 2 * PI;
	while (M > 2 * PI) M -= 2 * PI;

	IPeri = -M / n;
	double Period = (2 * PI) / n;

	Rperi = a * (1 - e);
	Rapo = a * (1 + e);

	IApo = IPeri + Period / 2;

	while (IPeri < 0) IPeri += Period;
	while (IApo < 0) IApo += Period;
}

void Soyuz7k_TM::CalcCircular()
{
	double Rapse = Rapo;
	double Vcirc = sqrt(mu / Rapse);
	double Vapse = sqrt(2 * mu / Rapse - mu / a);

	dv = fabs(Vcirc - Vapse);
}

double RocketEqnT(double dv, double m, double F, double isp)
{
	return (dv * m / (2.0 * F)) * (1 + exp(-dv / isp));
}

void Soyuz7k_TM::CalcIBurn()
{
	double F = GetThrusterMax0(th_main);
	double isp = GetThrusterIsp0(th_main);
	double m = GetMass();

	IBurn = RocketEqnT(dv, m, F, isp);
	IBurn2 = RocketEqnT(dv / 2.0, m, F, isp);
}

void Soyuz7k_TM::ArmAutoBurn()
{
	double mjd = oapiGetSimMJD();
	TIG = mjd + IApo / 86400.0 - IBurn2 / 86400.0;
	ECutoff = TIG + IBurn / 86400.0;
	IsArmed = true;
}

void Soyuz7k_TM::APTimeStep(double simt)
{
	// Auto Burn Ignition
	double mjd = oapiGetSimMJD();

	if (IsArmed && !IsEngaged && mjd >= TIG)
	{
		SetThrusterGroupLevel(THGROUP_MAIN, 1.0);
		IsEngaged = true;
		IsCircular = false;

		ECutoff = mjd + IBurn / 86400.0;
	}

	// End of Auto Burn
	if (IsEngaged && mjd >= ECutoff)
	{
		SetThrusterGroupLevel(THGROUP_MAIN, 0.0);
		IsEngaged = false;
		IsArmed = false;
		IsCircular = true;
		IsPrograde = false;

		ActivateNavmode(NAVMODE_KILLROT);
	}

	// Manual disarm
	if (!IsArmed && IsEngaged)
	{
		SetThrusterGroupLevel(THGROUP_MAIN, 0.0);
		IsEngaged = false;
		IsPrograde = false;
		ActivateNavmode(NAVMODE_KILLROT);
	}
}

void Soyuz7k_TM::ExecuteCirc()
{
	CalcApses();
	CalcCircular();
	CalcIBurn();

	if (!IsArmed) {
		ArmAutoBurn();
	}

	IsPrograde = true;

	char buffer[256];
	sprintf(buffer, "TIG = %f", TIG);
	oapiWriteLog(buffer);
}

void Soyuz7k_TM::ManeuverPrograde()
{
	if (IsPrograde) {
		ActivateNavmode(NAVMODE_PROGRADE);
		IsPrograde = true;
	}
	else {
		DeactivateNavmode(NAVMODE_PROGRADE);
		IsPrograde = false;
	}
}

bool Soyuz7k_TM::clbkDrawHUD(int mode, const HUDPAINTSPEC* hps, oapi::Sketchpad* skp)
{
	int s = hps->H;
	double d = (s * 0.00130208);

	VESSEL4::clbkDrawHUD(mode, hps, skp);

	int sw = ((hps->W));
	int lw = (int)(16 * sw / 1024);
	int lwoffset = sw - (18 * lw);
	int hlw = (int)(lw / 2);


	int roxl = 0;
	int royl = 0;

	double ds = s;
	double dsw = sw;
	double sc_ratio = ds / dsw;


	if (sc_ratio < 0.7284)
	{
		roxl = (lw * 10);
		royl = (int)(-88 * d);
	}

	int wd = (int)(136 * d);
	int wc = (int)(152 * d);
	int w0 = (int)(168 * d);
	int w1 = (int)(184 * d);
	int w2 = (int)(200 * d);
	int w3 = (int)(216 * d);
	int w4 = (int)(232 * d);
	int w5 = (int)(248 * d);
	int w6 = (int)(264 * d);
	int w7 = (int)(280 * d);
	int w8 = (int)(296 * d);
	int w9 = (int)(312 * d);
	int w10 = (int)(328 * d);
	int w11 = (int)(344 * d);
	int w12 = (int)(360 * d);
	int w13 = (int)(376 * d);
	int w14 = (int)(392 * d);
	int w15 = (int)(408 * d);
	int w16 = (int)(424 * d);

	if (oapiCockpitMode() != COCKPIT_VIRTUAL)
	{
		char abuf[256];

		double mjd = oapiGetSimMJD();
		double dv_remaining = dv;

		sprintf(abuf, "AUTO CIRCULARIZATION");
		skp->Text((10 + roxl), (wc + royl), abuf, strlen(abuf));

		// Remaining DV
		if (IsEngaged) {
			double burnTimeLeft = (ECutoff - mjd) * 86400.0;
			if (burnTimeLeft < 0.0) burnTimeLeft = 0.0;

			dv_remaining = dv * (burnTimeLeft / IBurn);

			sprintf(abuf, "dV: %.2f m/s", dv_remaining);
		}
		else {
			sprintf(abuf, "dV: %.2f m/s", dv);
		}
		skp->Text((10 + roxl), (w0 + royl), abuf, strlen(abuf));

		// Burn time
		if (IsEngaged) {
			double burnTimeLeft = (ECutoff - mjd) * 86400.0;
			if (burnTimeLeft < 0.0) burnTimeLeft = 0.0;

			sprintf(abuf, "Burn time left: %.2f s", burnTimeLeft);
		}
		else {
			sprintf(abuf, "Burn time: %.2f s", IBurn);
		}
		skp->Text((10 + roxl), (w1 + royl), abuf, strlen(abuf));

		// Time to ignition
		double timeToIgnition = (TIG - oapiGetSimMJD()) * 86400.0;
		if (timeToIgnition < 0.0)
			timeToIgnition = 0.0;
		sprintf(abuf, "Time to Burn: %.2f s", timeToIgnition);
		skp->Text((10 + roxl), (w2 + royl), abuf, strlen(abuf));

		//Engine armed
		sprintf(abuf, "Armed: %s", IsArmed ? "YES" : "NO");
		skp->Text((10 + roxl), (w3 + royl), abuf, strlen(abuf));

		//Engine Burning
		sprintf(abuf, "Engaged: %s", IsEngaged ? "YES" : "NO");
		skp->Text((10 + roxl), (w4 + royl), abuf, strlen(abuf));
	}
	return true;
}