#ifndef SPHSYSTEMSIMULATOR_h
#define SPHSYSTEMSIMULATOR_h
#include "Simulator.h"
#include <vector>
#include "spheresystem.h"

#define NAIVEACC 0
#define GRIDACC 1

class SphereSystemSimulator:public Simulator{
	struct Sphere
	{
		Vec3 Position;
		Vec3 Velocity;

	};
public:
	// Construtors
	SphereSystemSimulator();
	// Functions
	const char * getTestCasesStr();
	float length(Vec3 vector);
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	int PositionToCell(Vec3 Position);

	Vec3 dampingForce(Vec3 springForce, Vec3 velocity);
	
protected:
	// Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	float m_fMass;
	float m_fRadius;
	float m_fForceScaling;
	float m_fDamping;
	int   m_iNumSpheres;
	float m_fLambda;
	vector<Sphere*> m_spheres;  //dynamisches Array
	int   m_iKernel; // index of the m_Kernels[5], more detials in SphereSystemSimulator.cpp
	static std::function<float(float)> m_Kernels[5];
	
	int   m_iAccelerator; // switch between NAIVEACC and GRIDACC, (optionally, KDTREEACC, 2)
	
	vector<vector<int>> m_grid;
	vector<vector<int>> m_adjacentCells;

	SphereSystem* m_pSphereSystem; // add your own sphere system member!
	SphereSystem* m_SphereSystem2;
	// for Demo 3 only:
	// you will need multiple SphereSystem objects to do comparisons in Demo 3
	// m_iAccelerator should be ignored.
	// SphereSystem * m_pSphereSystemGrid; 

};

#endif