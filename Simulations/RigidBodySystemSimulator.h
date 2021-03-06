#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include <vector>

//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator:public Simulator{

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
	};

public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

	void normalize(Vec3& in)
	{
		float norm = sqrt(pow(in.x, 2) + pow(in.y, 2) + pow(in.z, 2));

		in = Vec3(in.x / norm, in.y / norm, in.z / norm);
	}

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;

	vector<RigidBody*> m_rigidBodies;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	float m_fTimeFactor;
	int m_iSelectedRigidbody;
	};
#endif