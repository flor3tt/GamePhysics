#include "MassSpringSystemSimulator.h"
#include <math.h>

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
}

const char * MassSpringSystemSimulator::getTestCasesStr()
{
	return "1_Step, Simple_Euler, Simple_Midpoint, Complex_Setup";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:
		//TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
		//TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
		break;
	case 2:break;
	default:break;
	}
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0: drawSimpleSetup(); break;
	case 1: drawSimpleSetup(); break;
	case 2: drawSimpleSetup(); break;
	case 3: drawComplexSetup(); break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "One Step Simulation !\n";
		break;
	case 1:
		cout << "Simple Euler Simulation !\n";
		break;
	case 2:
		cout << "Simple Midpoint Simulation !\n";
		break;
	case 3:
		cout << "Complex Scene Setup Simulation !\n";
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		//float inputScale = 0.001f;
		float inputScale = 1.0f;
		inputWorld = inputWorld * inputScale;
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
		m_externalForce = inputWorld;
	}
	else {
		m_externalForce = Vec3(0, 0, 0);
		//m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	//Different Simulation dependent on Integrator
	switch (m_iIntegrator)
	{
	case 0:
		//Simulate with Euler Integration
		for each(MassPoint mp in masspoints)
		{
			//Reset Force
			mp.Force = 0;

			mp.Position += mp.Velocity * timeStep;
		}

		for each(Spring sp in springs)
		{
			Vec3 distance = masspoints[sp.masspoint1].Position - masspoints[sp.masspoint2].Position;
			
			float length = sqrt(pow(distance.x, 2) + pow(distance.y, 2) + pow(distance.z, 2));
			Vec3 force = -1 * m_fStiffness * (length - sp.initialLength) * distance / length;

			masspoints[sp.masspoint1].Force += force;
			masspoints[sp.masspoint2].Force += force;
		}

		externalForcesCalculations(timeStep);
		applyExternalForce(m_externalForce);

		for each(MassPoint mp in masspoints)
		{
			mp.Velocity += timeStep * (mp.Force / m_fMass);
		}

		break;
	case 1:
		//Simulate with Leap-Frog Integration
		break;
	case 2:
		//Simulate with Midpoint Integration
		break;
	default:
		cout << "Invalid Integrator selected!\n";
		break;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	MassPoint newPoint;

	newPoint.Position = position;
	newPoint.Velocity = Velocity;
	newPoint.isFixed = isFixed;

	masspoints.push_back(newPoint);

	return masspoints.size - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring newSpring;

	newSpring.masspoint1 = masspoint1;
	newSpring.masspoint2 = masspoint2;
	newSpring.initialLength = initialLength;

	springs.push_back(newSpring);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return masspoints.size;
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springs.size;
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return masspoints[index].Position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return masspoints[index].Velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	for each(MassPoint mp in masspoints)
	{
		mp.Force += force;
	}
}

void MassSpringSystemSimulator::drawSimpleSetup()
{
	//DUC->DrawTriangleUsingShaders();
}

void MassSpringSystemSimulator::drawComplexSetup()
{
	//DUC->DrawTriangleUsingShaders();
}
