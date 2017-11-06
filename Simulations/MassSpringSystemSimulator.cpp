#include "MassSpringSystemSimulator.h"
#include <math.h>

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	m_iIntegrator = 0;
	m_fDamping = 0;
	m_fGravity = 9.81f;
	m_fStiffness = 40;
	m_fMass = 10;
}

const char * MassSpringSystemSimulator::getTestCasesStr()
{
	return "1_Step, Simple_Setup, Complex_Setup";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;

	//Define Enum for Integrator Selector
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler, Leap-Frog, Midpoint");

	switch (m_iTestCase)
	{
	case 0:break;
	case 1:		
		//TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
		//TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
		break;
	case 2:
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "step=0.5 min=0.5");
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "step=0.5 min=0.5");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "step=0.5 min=0.5");
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "step=0.01");
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	m_masspoints.clear();
	m_springs.clear();
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0: break;
	case 1: drawSimpleSetup(); break;
	case 2: drawSimpleSetup(); break;
	case 3: drawComplexSetup(); break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	reset();

	switch (m_iTestCase)
	{
	case 0:
		cout << "One Step Simulation !\n";

		//Add Simple Setup
		m_fGravity = 0;
		m_fMass = 10;
		m_fStiffness = 40;
		m_fDamping = 0;

		//EULER
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1);
		m_iIntegrator = 0;

		simulateTimestep(0.1f);

		//Print Results
		cout << m_masspoints[0]->Force << endl;
		cout << "After an Euler Step:" << endl;
		for(int i = 0; i < getNumberOfMassPoints(); ++i)
		{
			cout << "points " << i << " vel " << m_masspoints[i]->Velocity.toString() << ", pos " << m_masspoints[i]->Position.toString() << endl;
		}

		reset();

		//MIDPOINT
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1);
		m_iIntegrator = 1;

		simulateTimestep(0.1f);

		//Print Results
		cout << m_masspoints[0]->Force << endl;
		cout << "After a Midpoint Step:" << endl;
		for (int i = 0; i < getNumberOfMassPoints(); ++i)
		{
			cout << "points " << i << " vel " << m_masspoints[i]->Velocity.toString() << ", pos " << m_masspoints[i]->Position.toString() << endl;
		}

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
	case EULER:
		//Simulate with Euler Integration
		for each(MassPoint* mp in m_masspoints)
		{
			//Reset Force
			mp->Force = 0;
		}

		//Calculate Spring Forces
		for each(Spring* sp in m_springs)
		{
			Vec3 distance = m_masspoints[sp->masspoint1]->Position - m_masspoints[sp->masspoint2]->Position;
			
			float length = sqrt(pow(distance.x, 2) + pow(distance.y, 2) + pow(distance.z, 2));
			Vec3 force = -1 * m_fStiffness * (length - sp->initialLength) * distance / length;
			force -= m_fDamping * (m_masspoints[sp->masspoint1]->Velocity + m_masspoints[sp->masspoint2]->Velocity);

			m_masspoints[sp->masspoint1]->Force += force;
			m_masspoints[sp->masspoint2]->Force -= force;
		}

		//Caculate new Positions
		for each(MassPoint* mp in m_masspoints)
		{
			//Integrate Position
			mp->Position += mp->Velocity * timeStep;

			//Add Gravity
			mp->Force += m_fMass * m_fGravity;
		}

		externalForcesCalculations(timeStep);
		applyExternalForce(m_externalForce);

		//Caculate new Velocities
		for each(MassPoint* mp in m_masspoints)
		{
			mp->Velocity += timeStep * (mp->Force / m_fMass);
		}

		break;
	case LEAPFROG:
		//Simulate with Leap-Frog Integration
		break;
	case MIDPOINT:
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
	MassPoint* newPoint = new MassPoint;

	newPoint->Position = position;
	newPoint->Velocity = Velocity;
	newPoint->isFixed = isFixed;

	m_masspoints.push_back(newPoint);

	return m_masspoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring* newSpring = new Spring;

	newSpring->masspoint1 = masspoint1;
	newSpring->masspoint2 = masspoint2;
	newSpring->initialLength = initialLength;

	m_springs.push_back(newSpring);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_masspoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_masspoints[index]->Position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_masspoints[index]->Velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	for each(MassPoint* mp in m_masspoints)
	{
		mp->Force += force;
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
