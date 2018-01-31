#pragma once
#ifndef OpenProject_h
#define OpenProject_h
#include "Simulator.h"
#include <vector>

// Do Not Change
#define EULER 0
#define MIDPOINT 1
// Do Not Change



class OpenProject :public Simulator {

	struct MassPoint
	{
		int rigidBody;
		Vec3 Position;
	};

	struct RigidBody
	{
		float Mass;
		Vec3 Size;
		Mat4 InvInertiaRaw;
		Mat4 InvInertiaNow;

		Vec3 Position;
		Vec3 VelocityLin;
		Vec3 VelocityAng;
		Vec3 Momentum;
		Quat Orientation;

		Vec3 Force;
		Vec3 Torque;

		bool isFixed;
	};

	struct Spring
	{
		int masspoint1;
		int masspoint2;
		float initialLength;
	};
public:
	// Construtors
	OpenProject();

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
	int addMassPoint(int rigisBodyID, Vec3 position);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	void addRigidBody(Vec3 position, Vec3 size, int mass, bool isFixed = false);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void applyForceOnBodyLocal(int i, Vec3 loc, Vec3 force);

	Vec3 springForce(Vec3 position1, Vec3 position2, float initialLength);
	Vec3 dampingForce(Vec3 springForce, Vec3 velocity);

	Vec3 worldPos(int rigidBody, Vec3 Position);
	Vec3 worldVel(int rigidBody, Vec3 Position);
	

	void simulateTimestepEuler(float timeElapsed);
	void simulateTimestepMidpoint(float timeElapsed);

	//Specific Case Functions
	void drawProject();

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
	vector<MassPoint*> m_masspoints;
	vector<RigidBody*> m_rigidBodies;
	vector<Spring*> m_springs;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif