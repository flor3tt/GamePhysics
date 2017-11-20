#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
}

const char * RigidBodySystemSimulator::getTestCasesStr()
{
	return "1_Step, Single, Simple_Collision, Complex";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	m_rigidBodies.clear();
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	if (m_iTestCase == 0)
		return;

	//Setup Lighting
	DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0.97, 0.86, 1));

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

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	reset();

	Mat4 rotMat;
	//Vec3 pointPos;
	switch (m_iTestCase)
	{
	case 0:
		cout << "One Step Calculation!" << endl;

		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setVelocityOf(0, Vec3(0, 0, 0));
		rotMat.initRotationZ(90);
		setOrientationOf(0, Quat(rotMat));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));

		simulateTimestep(2);

		cout << "Linear Velocity: " << m_rigidBodies[0]->VelocityLin << endl;
		cout << "Angular Velocity: " << m_rigidBodies[0]->VelocityAng << endl;
		//pointPos = m_rigidBodies[0]->Position + m_rigidBodies[0]->Orientation.getRotMat().transformVector(Vec3(0.3, 0.5, 0.25));
		cout << "Point Velocity: " << m_rigidBodies[0]->VelocityLin + cross(m_rigidBodies[0]->VelocityAng, Vec3(-0.3, -0.5, -0.25)) << endl;
		
		break;
	case 1:
		cout << "One Body Simulation!" << endl;

		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setVelocityOf(0, Vec3(0, 0, 0));
		rotMat.initRotationZ(90);
		setOrientationOf(0, Quat(rotMat));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
		break;
	case 2:
		break;
	case 3:
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	//TO-DO: Sinnvolle external force calculation
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
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
		m_externalForce = inputWorld;
	}
	else {
		m_externalForce = Vec3(0, 0, 0);
		//m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}

	m_externalForce = Vec3(0, 0, 0);
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	for each(RigidBody* rb in m_rigidBodies)
	{
		//Euler Step
		rb->Position += timeStep * rb->VelocityLin;
		rb->VelocityLin += timeStep * ((m_externalForce + rb->Force) / rb->Mass);

		//Update Orientation
		Quat newRot = rb->Orientation + (timeStep / 2) * Quat(0, rb->VelocityAng.x, rb->VelocityAng.y, rb->VelocityAng.z) * rb->Orientation;	
		double norm = newRot.norm();

		rb->Orientation = newRot;
		rb->Orientation /= norm;

		//Update angular velocity and stuff
		rb->Momentum += timeStep * rb->Torque;

		Mat4 rotMat = rb->Orientation.getRotMat();
		Mat4 rotMatTrans = rotMat;
		rotMatTrans.transpose();
		Mat4 invInertiaNow = rotMatTrans * rb->InvInertiaRaw * rotMat;
		
		rb->VelocityAng = invInertiaNow.transformVector(rb->Momentum);

		//Clear Force and Torque
		rb->Force = Vec3(0, 0, 0);
		rb->Torque = Vec3(0, 0, 0);
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return m_rigidBodies[i]->Position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_rigidBodies[i]->VelocityLin;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_rigidBodies[i]->VelocityAng;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_rigidBodies[i]->Force += force;

	Vec3 relPos = loc - m_rigidBodies[i]->Position;
	m_rigidBodies[i]->Torque += cross(relPos, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBody* rb = new RigidBody;

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

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_rigidBodies[i]->Orientation = orientation;

}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_rigidBodies[i]->VelocityLin = velocity;
}
