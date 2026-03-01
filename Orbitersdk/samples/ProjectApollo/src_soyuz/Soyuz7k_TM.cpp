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
	SURFHANDLE atrcs;
	void clbkSetClassCaps(FILEHANDLE cfg);

private:
	ATTACHMENTHANDLE hAttach;
	DOCKHANDLE hDock;
	MESHHANDLE hT_AF_PAO, hTM_BO, hOK_SA, hok_vc, hOK_cabling;
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

void Soyuz7k_TM::clbkSetClassCaps(FILEHANDLE cfg)
{
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
	atrcs = oapiRegisterExhaustTexture("dragon1/atrcs");

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
	AddExhaust(th_back[0], 5.00, 0.25, atrcs);
	AddExhaust(th_back[1], 5.00, 0.25, atrcs);

	AddExhaust(th_translate_left, 5.00, 0.25, atrcs);
	AddExhaust(th_translate_right, 5.00, 0.25, atrcs);

	AddExhaust(th_translate_up, 5.00, 0.25, atrcs);
	AddExhaust(th_translate_down, 5.00, 0.25, atrcs);

	AddExhaust(th_pitch_up[0], 5.00, 0.25, atrcs);
	AddExhaust(th_pitch_up[1], 5.00, 0.25, atrcs);

	AddExhaust(th_pitch_dn[0], 5.00, 0.25, atrcs);
	AddExhaust(th_pitch_dn[1], 5.00, 0.25, atrcs);

	AddExhaust(th_yaw_l[0], 5.00, 0.25, atrcs);
	AddExhaust(th_yaw_l[1], 5.00, 0.25, atrcs);

	AddExhaust(th_yaw_r[0], 5.00, 0.25, atrcs);
	AddExhaust(th_yaw_r[1], 5.00, 0.25, atrcs);

	AddExhaust(th_bank_l[0], 5.00, 0.25, atrcs);
	AddExhaust(th_bank_l[1], 5.00, 0.25, atrcs);

	AddExhaust(th_bank_r[0], 5.00, 0.25, atrcs);
	AddExhaust(th_bank_r[1], 5.00, 0.25, atrcs);
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