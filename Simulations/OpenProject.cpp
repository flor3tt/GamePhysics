#include "OpenProject.h"
#include <math.h>

OpenProject::OpenProject()
{
	m_iTestCase = 0;
	m_iIntegrator = 0;
	m_fDamping = 0;
	m_fGravity = 0;
	m_fStiffness = 40;

	m_bRealTimeSimulation = false;
	m_fSimulationSpeedFactor = 1;
	lastTime = clock();
}

const char * OpenProject::getTestCasesStr()
{
	return "OpenProject";
}

void OpenProject::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;

	//Define Enum for Integrator Selector
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler, Midpoint");
	TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");

	TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "step=0.5 min=0.5");
	TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "step=0.01 min=0");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Real Time", TW_TYPE_BOOLCPP, &m_bRealTimeSimulation, "");
	TwAddVarRW(DUC->g_pTweakBar, "Time Factor", TW_TYPE_FLOAT, &m_fSimulationSpeedFactor, "step=0.01 min=0.01");

}

void OpenProject::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	m_masspoints.clear();
	m_springs.clear();
	m_rigidBodies.clear();
}

void OpenProject::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	drawProject();
}

void OpenProject::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	reset();

	switch (m_iTestCase)
	{
	case 0:
		cout << "Open Project !\n";

		m_fDamping = 0.01;
		m_fGravity = -9.81;
		m_fStiffness = 40;

		m_iIntegrator = MIDPOINT;

		addRigidBody(Vec3(0.5, 0.5, 0), Vec3(0.001, 0.001, 0.001), 0, true);
		addRigidBody(Vec3(-0.5, 0.5, 0), Vec3(0.001, 0.001, 0.001), 0, true);
		addRigidBody(Vec3(0, 0, 0), Vec3(0.2, 0.1, 0.2), 2);
		
		addMassPoint(0, Vec3(0, 0, 0));
		addMassPoint(1, Vec3(0, 0, 0));
		addMassPoint(2, Vec3(0.1, 0.05, 0.1));
		addMassPoint(2, Vec3(-0.1, 0.05, -0.1));
		addSpring(0, 2, 0.2);
		addSpring(1, 3, 0.2);



		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void OpenProject::externalForcesCalculations(float timeElapsed)
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
		float inputScale = 0.1f;
		inputWorld = inputWorld * inputScale;
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
		m_externalForce = inputWorld;
	}
	else {
		m_externalForce = Vec3(0, 0, 0);
		//m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}

	//Add Gravity
	//m_externalForce.y += m_fMass * m_fGravity;

}

void OpenProject::simulateTimestep(float timeStep)
{
	updateElapsedTime();
	if (m_bRealTimeSimulation) {
		timeStep = m_fElapsedRealTime;
	}
	
	if(m_iIntegrator == EULER)
		simulateTimestepEuler(timeStep);
	if (m_iIntegrator == MIDPOINT)
		simulateTimestepMidpoint(timeStep);
	if (m_iTestCase == 2)
	{
		//Collision calculation
		//i.e. make sure, that all MassPoints stay inside the cube
		//for each(MassPoint* mp in m_masspoints)
		//{
		//	//mp->Position.makeCeil(-0.5);
		//	//mp->Position.makeFloor(0.5);
		//	for (int i = 0; i < 3; ++i)
		//	{
		//		float j = mp->Position.value[i];

		//		if (j <= -0.5)
		//		{
		//			mp->Velocity.value[i] = m_fBouncyness * mp->Velocity.getAbsolutes().value[i];
		//			mp->Position.value[i] = -0.5;
		//		}
		//		else if (j >= 0.5)
		//		{
		//			mp->Velocity.value[i] = -1 * m_fBouncyness * mp->Velocity.getAbsolutes().value[i];
		//			mp->Position.value[i] = 0.5;
		//		}
		//	}

		//}
	}


}

Vec3 OpenProject::springForce(Vec3 position1, Vec3 position2, float initialLength)
{
	float length = sqrt(position1.squaredDistanceTo(position2));
	return -m_fStiffness * (length - initialLength) * (position1 - position2) / length;
}

Vec3 OpenProject::dampingForce(Vec3 springForce, Vec3 velocity)
{
	if (springForce.value[springForce.getAbsolutes().maxComponentId()] == 0)
		return Vec3(0, 0, 0);

	Vec3 force_unit = springForce / sqrt(pow(springForce.x, 2) + pow(springForce.y, 2) + pow(springForce.z, 2));

	return m_fDamping * force_unit * (velocity.x * force_unit.x + velocity.y * force_unit.y + velocity.z * force_unit.z);
}

Vec3 OpenProject::worldPos(int rigidBody, Vec3 Position)
{
	return m_rigidBodies[rigidBody]->Position + m_rigidBodies[rigidBody]->Orientation.getRotMat().transformVector(Position);
	
}

Vec3 OpenProject::worldVel(int rigidBody, Vec3 Position)
{
	return m_rigidBodies[rigidBody]->VelocityLin + cross(m_rigidBodies[rigidBody]->VelocityAng, Position);
}

void OpenProject::simulateTimestepMidpoint(float timeStep)
{

	externalForcesCalculations(timeStep);

	// Midpoint position integration based on last position and velocity
	vector<Vec3> pos_tmp;
	for each(MassPoint* massPoint in m_masspoints) {
		pos_tmp.push_back(worldPos(massPoint->rigidBody, massPoint->Position) + (timeStep / 2) * worldVel(massPoint->rigidBody, massPoint->Position));
	}

	// Calculate midpoint spring forces using the updated pos_tmp
	vector<Vec3> f_tmp;
	f_tmp.resize(m_masspoints.size());
	for each(Spring* spring in m_springs) {
		MassPoint* massPoint1 = m_masspoints[spring->masspoint1];
		MassPoint* massPoint2 = m_masspoints[spring->masspoint2];
		Vec3 f_tmp_spring = springForce(pos_tmp[spring->masspoint1], pos_tmp[spring->masspoint2], spring->initialLength);

		f_tmp[spring->masspoint1] += f_tmp_spring - dampingForce(f_tmp_spring, worldVel(massPoint1->rigidBody, massPoint1->Position));
		f_tmp[spring->masspoint2] += -f_tmp_spring - dampingForce(-1 * f_tmp_spring, worldVel(massPoint2->rigidBody, massPoint2->Position));
	}

	// Integrate velocity using midpoint spring forces
	vector<Vec3> v_tmp;
	unsigned int i;
	for (i = 0; i < m_masspoints.size(); i++)
	{
		MassPoint* massPoint = m_masspoints[i];
		v_tmp.push_back(worldVel(massPoint->rigidBody, massPoint->Position) + (timeStep / 2) * (f_tmp[i] / m_rigidBodies[massPoint->rigidBody]->Mass));

	}

	// Again, calculate spring forces
	vector<Vec3> f_estimate;
	f_estimate.resize(m_masspoints.size());
	for each(Spring* spring in m_springs) {
		Vec3 f_tmp_spring = springForce(pos_tmp[spring->masspoint1], pos_tmp[spring->masspoint2], spring->initialLength);
		f_estimate[spring->masspoint1] += f_tmp_spring - dampingForce(f_tmp_spring, v_tmp[spring->masspoint1]);
		f_estimate[spring->masspoint2] += -f_tmp_spring - dampingForce(-1 * f_tmp_spring, v_tmp[spring->masspoint2]);
	}

	// Apply actual forces on rigidbodies
	for (i = 0; i < m_masspoints.size(); i++)
	{
		if(!m_rigidBodies[m_masspoints[i]->rigidBody]->isFixed)
			applyForceOnBodyLocal(m_masspoints[i]->rigidBody, m_masspoints[i]->Position, f_estimate[i]);

	}

	//Update Rigisbodies with full Euler Step
	for each(RigidBody* rb in m_rigidBodies)
	{
		rb->Force += m_externalForce;
		rb->Force.y += m_fGravity * rb->Mass;

		//Euler Step
		if (!rb->isFixed)
		{
			rb->Position += timeStep * rb->VelocityLin;
			rb->VelocityLin += timeStep * (rb->Force / rb->Mass);
		}

		//Update Orientation
		Quat newRot = rb->Orientation + (timeStep / 2) * Quat(rb->VelocityAng.x, rb->VelocityAng.y, rb->VelocityAng.z, 0) * rb->Orientation;
		double norm = newRot.norm();

		rb->Orientation = newRot;
		rb->Orientation /= norm;

		//cout << rb->Torque << endl;

		//Update angular velocity and stuff
		rb->Momentum += timeStep * rb->Torque;

		Mat4 rotMat = rb->Orientation.getRotMat();
		Mat4 rotMatTrans = rotMat;
		rotMatTrans.transpose();
		rb->InvInertiaNow = rotMat * rb->InvInertiaRaw * rotMatTrans;

		if (!rb->isFixed)
		{
			rb->VelocityAng = rb->InvInertiaNow.transformVector(rb->Momentum);
		}

		//Clear Force and Torque
		rb->Force = Vec3(0, 0, 0);
		rb->Torque = Vec3(0, 0, 0);

	}

}

void OpenProject::simulateTimestepEuler(float timeStep)
{
	for each(MassPoint* mp in m_masspoints)
	{
		//Reset Force
		//mp->Force = 0;
	}

	//Applay external forces, i.e. User Input + Gravity
	externalForcesCalculations(timeStep);
	//applyExternalForce(m_externalForce);

	//Calculate Spring Forces
	for each(Spring* sp in m_springs)
	{
		Vec3 pos1 = worldPos(m_masspoints[sp->masspoint1]->rigidBody, m_masspoints[sp->masspoint1]->Position);
		Vec3 pos2 = worldPos(m_masspoints[sp->masspoint2]->rigidBody, m_masspoints[sp->masspoint2]->Position);

		Vec3 distance = pos1 - pos2;

		float length = sqrt(pow(distance.x, 2) + pow(distance.y, 2) + pow(distance.z, 2));
		if (length == 0)
		{
			pos2 += Vec3(0, -0.001, 0);

			distance = pos1 - pos2;
			length = sqrt(pow(distance.x, 2) + pow(distance.y, 2) + pow(distance.z, 2));
		}
		Vec3 force = -1 * m_fStiffness * (length - sp->initialLength) * distance / length;

		Vec3 force1 = force - dampingForce(force, worldVel(m_masspoints[sp->masspoint1]->rigidBody, m_masspoints[sp->masspoint1]->Position));
		Vec3 force2 = force - force - dampingForce(-1 * force, worldVel(m_masspoints[sp->masspoint2]->rigidBody, m_masspoints[sp->masspoint2]->Position));
		force2 *= -1;

		applyForceOnBody(m_masspoints[sp->masspoint1]->rigidBody, m_masspoints[sp->masspoint1]->Position, force1);
		applyForceOnBody(m_masspoints[sp->masspoint2]->rigidBody, m_masspoints[sp->masspoint2]->Position, force2);
	}

	for each(RigidBody* rb in m_rigidBodies)
	{
		rb->Force += m_externalForce;
		rb->Force.y += m_fGravity * rb->Mass;

		//Euler Step
		if (!rb->isFixed)
		{
			rb->Position += timeStep * rb->VelocityLin;
			rb->VelocityLin += timeStep * ((m_externalForce + rb->Force) / rb->Mass);
		}

		//Update Orientation
		Quat newRot = rb->Orientation + (timeStep / 2) * Quat(rb->VelocityAng.x, rb->VelocityAng.y, rb->VelocityAng.z, 0) * rb->Orientation;
		double norm = newRot.norm();

		rb->Orientation = newRot;
		rb->Orientation /= norm;

		//Update angular velocity and stuff
		rb->Momentum += timeStep * rb->Torque;

		Mat4 rotMat = rb->Orientation.getRotMat();
		Mat4 rotMatTrans = rotMat;
		rotMatTrans.transpose();
		rb->InvInertiaNow = rotMat * rb->InvInertiaRaw * rotMatTrans;

		if (!rb->isFixed)
		{
			rb->VelocityAng = rb->InvInertiaNow.transformVector(rb->Momentum);
		}

		//Clear Force and Torque
		rb->Force = Vec3(0, 0, 0);
		rb->Torque = Vec3(0, 0, 0);

	}


	////Caculate new Positions
	//for each(MassPoint* mp in m_masspoints)
	//{
	//	if (mp->isFixed)
	//		continue;

	//	//Integrate Position
	//	mp->Position += timeStep * mp->Velocity;
	//}

	////Caculate new Velocities
	//for each(MassPoint* mp in m_masspoints)
	//{
	//	mp->Velocity += timeStep * (mp->Force / m_fMass);
	//}
}

void OpenProject::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void OpenProject::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void OpenProject::setMass(float mass)
{
	m_fMass = mass;
}

void OpenProject::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void OpenProject::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

void OpenProject::setBounciness(float bouncyness)
{
	m_fBouncyness = bouncyness;
}

int OpenProject::addMassPoint(int rigidBodyID, Vec3 position)
{
	MassPoint* newPoint = new MassPoint;

	newPoint->Position = position;
	newPoint->rigidBody = rigidBodyID;

	m_masspoints.push_back(newPoint);

	return m_masspoints.size() - 1;
}

void OpenProject::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring* newSpring = new Spring;

	newSpring->masspoint1 = masspoint1;
	newSpring->masspoint2 = masspoint2;
	newSpring->initialLength = initialLength;

	m_springs.push_back(newSpring);
}

void OpenProject::addRigidBody(Vec3 position, Vec3 size, int mass, bool isFixed)
{
	RigidBody* rb = new RigidBody;

	rb->isFixed = isFixed;
	rb->Mass = mass;
	rb->Position = position;
	rb->Size = size;
	rb->Momentum = Vec3(0, 0, 0);

	Mat4 rotMat;
	rotMat.initRotationX(0);
	rb->Orientation = Quat(rotMat);

	//Calculate initial Inertia Matrix for a cuboid
	Mat4 inertia;
	double matValues[16];
	for (int i = 0; i < 15; ++i)
	{
		matValues[i] = 0;
	}
	matValues[15] = 1;
	matValues[0] = ((double)mass / 12) * (pow(size.y, 2) + pow(size.z, 2));
	matValues[5] = ((double)mass / 12) * (pow(size.x, 2) + pow(size.z, 2));
	matValues[10] = ((double)mass / 12) * (pow(size.x, 2) + pow(size.y, 2));

	inertia.initFromArray(matValues);
	rb->InvInertiaRaw = inertia.inverse();

	m_rigidBodies.push_back(rb);
}

void OpenProject::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_rigidBodies[i]->Force += force;

	Vec3 relPos = loc - m_rigidBodies[i]->Position;
	m_rigidBodies[i]->Torque += cross(relPos, force);
}

void OpenProject::applyForceOnBodyLocal(int i, Vec3 loc, Vec3 force)
{
	m_rigidBodies[i]->Force += force;

	m_rigidBodies[i]->Torque += cross(loc, force);
}

void OpenProject::drawProject()
{
	Vec3 mpScale = Vec3(0.01);

	//Setup Lighting
	DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0.97, 0.86, 1));

	//Draw MassPoints
	/*for each(MassPoint* mp in m_masspoints)
	{
		DUC->drawSphere(mp->Position, mpScale);
	}*/

	//Draw Springs
	DUC->beginLine();
	for each(Spring* sp in m_springs)
	{
		//Make Spring Color dependent on current spring Length
		Vec3 pos1 = worldPos(m_masspoints[sp->masspoint1]->rigidBody, m_masspoints[sp->masspoint1]->Position);
		Vec3 pos2 = worldPos(m_masspoints[sp->masspoint2]->rigidBody, m_masspoints[sp->masspoint2]->Position);

		Vec3 distance = pos1 - pos2;
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

		DUC->drawLine(pos1, springColor, pos2, springColor);
	}
	DUC->endLine();


	for each(RigidBody* rb in m_rigidBodies)
	{
		Mat4 scaleMat;
		Mat4 rotMat;
		Mat4 translatMat;

		scaleMat.initScaling(rb->Size.x, rb->Size.y, rb->Size.z);
		rotMat = rb->Orientation.getRotMat();
		translatMat.initTranslation(rb->Position.x, rb->Position.y, rb->Position.z);

		DUC->drawRigidBody(scaleMat * rotMat * translatMat);
	}
}

void OpenProject::updateElapsedTime()
{
	clock_t now = clock();
	m_fElapsedRealTime = float(now - lastTime) / CLOCKS_PER_SEC * m_fSimulationSpeedFactor;
	lastTime = now;
}
