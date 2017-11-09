#include "MassSpringSystemSimulator.h"
#include <math.h>

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 1;
	m_iIntegrator = 0;
	m_fDamping = 0;
	m_fGravity = 0;
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
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "step=0.01 min=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "step=0.5 min=0.5");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "step=0.01 min=0");
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
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
	case 0: drawSimpleSetup(); break;
	case 1: drawSimpleSetup(); break;
	case 2: drawComplexSetup(); break;
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
		m_iIntegrator = EULER;

		simulateTimestepEuler(0.1f);

		//Print Results
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
		m_iIntegrator = MIDPOINT;

		simulateTimestepMidpoint(0.1f);

		//Print Results
		cout << "After a Midpoint Step:" << endl;
		for (int i = 0; i < getNumberOfMassPoints(); ++i)
		{
			cout << "points " << i << " vel " << m_masspoints[i]->Velocity.toString() << ", pos " << m_masspoints[i]->Position.toString() << endl;
		}

		break;
	case 1:
		cout << "Simple Scene Setup Simulation !\n";

		m_fGravity = 0;
		m_fMass = 10;
		m_fStiffness = 40;
		m_fDamping = 0;
		m_iIntegrator = EULER;

		
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1);

		break;
	case 2:
		cout << "Complex Scene Setup Simulation !\n";

		m_fGravity = -9.81;
		m_fMass = 0.01;
		m_fStiffness = 25;
		m_fDamping = 0.01;
		m_iIntegrator = MIDPOINT;

		/**
		addMassPoint(Vec3(0, 0.5, 0), Vec3(0, 0, 0), true);
		addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
		addSpring(0, 1, 0.3);
		*/

		
		//Hängen von der Decke
		addMassPoint(Vec3(0, 0.5, 0), Vec3(0, 0, 0), true);//0
		addMassPoint(Vec3(0, 0.3, 0), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0.1, 0.2, 0.1), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0.1, 0.2, -0.1), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(-0.1, 0.2, 0.1), Vec3(0, 0, 0), false);
		addSpring(0, 1, 0.2);
		addSpring(1, 2, 0.15);
		addSpring(1, 3, 0.15);
		addSpring(1, 4, 0.15);
		addSpring(2, 3, 0.15);
		addSpring(2, 4, 0.15);
		addSpring(3, 4, 0.15);

		//Teil frei im Raum
		addMassPoint(Vec3(-0.1, -0.5, -0.1), Vec3(0, 0, 0), false);//5
		addMassPoint(Vec3(-0.1, -0.5, 0.1), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0.1, -0.5, 0.1), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0.1, -0.5, -0.1), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0, -0.4, 0), Vec3(0, 0, 0), false);//9
		addMassPoint(Vec3(-0.1, -0.3, -0.1), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(-0.1, -0.3, 0.1), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0.1, -0.3, 0.1), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0.1, -0.3, -0.1), Vec3(0, 0, 0), false);//13

		addSpring(5, 6, 0.2);
		addSpring(5, 8, 0.2);
		addSpring(5, 9, 0.15);
		addSpring(5, 10, 0.2);
		addSpring(6, 7, 0.2);
		addSpring(6, 9, 0.15);
		addSpring(6, 11, 0.2);
		addSpring(7, 8, 0.2);
		addSpring(7, 9, 0.15);
		addSpring(7, 12, 0.2);
		addSpring(8, 9, 0.15);
		addSpring(8, 13, 0.2);
		addSpring(9, 10, 0.15);
		addSpring(9, 11, 0.15);
		addSpring(9, 12, 0.15);
		addSpring(9, 13, 0.15);
		addSpring(10, 11, 0.2);
		addSpring(10, 13, 0.2);
		addSpring(11, 12, 0.2);
		addSpring(12, 13, 0.2);
		

		break;
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
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
		m_externalForce = inputWorld;
	}
	else {
		m_externalForce = Vec3(0, 0, 0);
		//m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}

	//Add Gravity
	m_externalForce.y += m_fMass * m_fGravity;

}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	if (m_iTestCase == 0)
		return;
	//Different Simulation dependent on Integrator
	switch (m_iIntegrator)
	{
	case EULER:
		//Simulate with Euler Integration
		simulateTimestepEuler(timeStep);
		break;
	case LEAPFROG:
		//Simulate with Leap-Frog Integration
		simulateTimestepLeapfrog(timeStep);
		break;
	case MIDPOINT:
		//Simulate with Midpoint Integration
		simulateTimestepMidpoint(timeStep);
		break;
	default:
		cout << "Invalid Integrator selected!\n";
		break;
	}

	if (m_iTestCase == 2)
	{
		//Collision calculation
		//i.e. make sure, that all MassPoints stay inside the cube
		for each(MassPoint* mp in m_masspoints)
		{
			mp->Position.makeCeil(-0.5);
			mp->Position.makeFloor(0.5);
		}
	}


}

Vec3 MassSpringSystemSimulator::springForce(Vec3 position1, Vec3 position2, float initialLength)
{
	float length = sqrt(position1.squaredDistanceTo(position2));
	return -m_fStiffness * (length - initialLength) * (position1 - position2) / length;
}

Vec3 MassSpringSystemSimulator::dampingForce(Vec3 springForce, Vec3 velocity)
{
	if (springForce.value[springForce.getAbsolutes().maxComponentId()] == 0)
		return Vec3(0, 0, 0);

	Vec3 force_unit = springForce / sqrt(pow(springForce.x, 2) + pow(springForce.y, 2) + pow(springForce.z, 2));

	return m_fDamping * force_unit * (velocity.x * force_unit.x + velocity.y * force_unit.y + velocity.z * force_unit.z);
}

void MassSpringSystemSimulator::simulateTimestepEuler(float timeStep)
{
	for each(MassPoint* mp in m_masspoints)
	{
		//Reset Force
		mp->Force = 0;
	}

	//Applay external forces, i.e. User Input + Gravity
	externalForcesCalculations(timeStep);
	applyExternalForce(m_externalForce);

	//Calculate Spring Forces
	for each(Spring* sp in m_springs)
	{
		Vec3 distance = m_masspoints[sp->masspoint1]->Position - m_masspoints[sp->masspoint2]->Position;
		
		float length = sqrt(pow(distance.x, 2) + pow(distance.y, 2) + pow(distance.z, 2));
		if (length == 0)
		{
			m_masspoints[sp->masspoint2]->Position += Vec3(0, -0.001, 0);

			distance = m_masspoints[sp->masspoint1]->Position - m_masspoints[sp->masspoint2]->Position;
			length = sqrt(pow(distance.x, 2) + pow(distance.y, 2) + pow(distance.z, 2));
		}
		Vec3 force = -1 * m_fStiffness * (length - sp->initialLength) * distance / length;

		m_masspoints[sp->masspoint1]->Force += force - dampingForce(force, m_masspoints[sp->masspoint1]->Velocity);
		m_masspoints[sp->masspoint2]->Force -= force - dampingForce(-1 * force, m_masspoints[sp->masspoint2]->Velocity);
	}

	//Caculate new Positions
	for each(MassPoint* mp in m_masspoints)
	{
		if (mp->isFixed)
			continue;

		//Integrate Position
		mp->Position += timeStep * mp->Velocity;
	}

	//Caculate new Velocities
	for each(MassPoint* mp in m_masspoints)
	{
		mp->Velocity += timeStep * (mp->Force / m_fMass);
	}
}

void MassSpringSystemSimulator::simulateTimestepMidpoint(float timeStep)
{
	externalForcesCalculations(timeStep);

	// Midpoint position integration based on last position and velocity
	vector<Vec3> pos_tmp;
	for each(MassPoint* massPoint in m_masspoints) {
		pos_tmp.push_back(massPoint->Position + (timeStep / 2) * massPoint->Velocity);
	}

	// Calculate midpoint spring forces using the updated pos_tmp
	vector<Vec3> f_tmp;
	f_tmp.resize(m_masspoints.size());
	for each(Spring* spring in m_springs) {
		MassPoint* massPoint1 = m_masspoints[spring->masspoint1];
		MassPoint* massPoint2 = m_masspoints[spring->masspoint2];
		Vec3 f_tmp_spring = springForce(pos_tmp[spring->masspoint1], pos_tmp[spring->masspoint2], spring->initialLength);
		
		f_tmp[spring->masspoint1] += f_tmp_spring -dampingForce(f_tmp_spring, massPoint1->Velocity);
		f_tmp[spring->masspoint2] += -f_tmp_spring -dampingForce(-1 * f_tmp_spring, massPoint2->Velocity);
	}
	
	// Integrate velocity using midpoint spring forces and these new values to integrate the position
	vector<Vec3> v_tmp;
	unsigned int i;
	for (i = 0; i < m_masspoints.size(); i++) {
		MassPoint* massPoint = m_masspoints[i];
		v_tmp.push_back(massPoint->Velocity + (timeStep / 2) * (f_tmp[i] / m_fMass));
		if (massPoint->isFixed)
			continue;
		massPoint->Position += timeStep * v_tmp[i];
	}

	// Again, calculate spring forces
	vector<Vec3> f_estimate;
	f_estimate.resize(m_masspoints.size());
	for each(Spring* spring in m_springs) {
		Vec3 f_tmp_spring = springForce(pos_tmp[spring->masspoint1], pos_tmp[spring->masspoint2], spring->initialLength);
		f_estimate[spring->masspoint1] += f_tmp_spring - dampingForce(f_tmp_spring, v_tmp[spring->masspoint1]);
		f_estimate[spring->masspoint2] += -f_tmp_spring -dampingForce(-1 * f_tmp_spring, v_tmp[spring->masspoint2]);
	}

	// Integrate Velocity
	for (i = 0; i < m_masspoints.size(); i++) {
		if (m_masspoints[i]->isFixed)
			continue;
		m_masspoints[i]->Velocity += timeStep * ((f_estimate[i] + m_externalForce) / m_fMass);
	}

}



void MassSpringSystemSimulator::simulateTimestepLeapfrog(float timeStep)
{

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
	Vec3 mpScale = Vec3(0.01f, 0.01f, 0.01f);
	Vec3 springColor = Vec3(0, 1, 0);

	//Setup Lighting
	DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0.97, 0.86, 1));

	//Draw MassPoints
	for each(MassPoint* mp in m_masspoints)
	{
		DUC->drawSphere(mp->Position, mpScale);
	}

	//Draw Springs
	DUC->beginLine();
	for each(Spring* sp in m_springs)
	{
		DUC->drawLine(m_masspoints[sp->masspoint1]->Position, springColor, m_masspoints[sp->masspoint2]->Position, springColor);
	}
	DUC->endLine();
}

void MassSpringSystemSimulator::drawComplexSetup()
{
	Vec3 mpScale = Vec3(0.01);

	//Setup Lighting
	DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0.97, 0.86, 1));

	//Draw MassPoints
	for each(MassPoint* mp in m_masspoints)
	{
		DUC->drawSphere(mp->Position, mpScale);
	}

	//Draw Springs
	DUC->beginLine();
	for each(Spring* sp in m_springs)
	{
		//Make Spring Color dependent on current spring Length
		Vec3 distance = m_masspoints[sp->masspoint1]->Position - m_masspoints[sp->masspoint2]->Position;
		float length = sqrt(pow(distance.x, 2) + pow(distance.y, 2) + pow(distance.z, 2));

		float greenAmount;
		if (length <= sp->initialLength)
		{
			greenAmount = length / sp->initialLength;
		}
		else
		{
			greenAmount = sp->initialLength / length;
		}
		Vec3 springColor = Vec3(1 - greenAmount, greenAmount, 0);

		DUC->drawLine(m_masspoints[sp->masspoint1]->Position, springColor, m_masspoints[sp->masspoint2]->Position, springColor);
	}
	DUC->endLine();
}
