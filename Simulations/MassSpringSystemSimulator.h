#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include <vector>

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change



class MassSpringSystemSimulator:public Simulator{

	struct MassPoint
	{
		Vec3 Position;
		Vec3 Velocity;
		bool isFixed;
		Vec3 Force;
	};

	struct Spring
	{
		int masspoint1;
		int masspoint2;
		float initialLength;
	};
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	void setBounciness(float bouncyness);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	
	Vec3 springForce(Vec3 position1, Vec3 position2, float initialLength);
	Vec3 dampingForce(Vec3 springForce, Vec3 velocity);
	void simulateTimestepEuler(float timeElapsed);
	void simulateTimestepMidpoint(float timeElapsed);
	void simulateTimestepLeapfrog(float timeElapsed);

	//Specific Case Functions
	void drawSimpleSetup();
	void drawComplexSetup();
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	float m_fGravity;
	float m_fBouncyness;

	bool m_bRealTimeSimulation;
	float m_fSimulationSpeedFactor;
	float m_fElapsedRealTime;
	clock_t lastTime;

	// Calculate elapsed time
	void updateElapsedTime();

	//MassPoint and Spring Management
	vector<MassPoint*> m_spheres;
	vector<Spring*> m_springs;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif