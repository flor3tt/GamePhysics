#include "SphereSystemSimulator.h"
#include <math.h>

std::function<float(float)> SphereSystemSimulator::m_Kernels[5] = {
	[](float x) {return 1.0f; },              // Constant, m_iKernel = 0
	[](float x) {return 1.0f - x; },          // Linear, m_iKernel = 1, as given in the exercise Sheet, x = d/2r
	[](float x) {return (1.0f - x)*(1.0f - x); }, // Quadratic, m_iKernel = 2
	[](float x) {return 1.0f / (x)-1.0f; },     // Weak Electric Charge, m_iKernel = 3
	[](float x) {return 1.0f / (x*x) - 1.0f; },   // Electric Charge, m_iKernel = 4
};


SphereSystemSimulator::SphereSystemSimulator()
{
	m_iTestCase = 1;
	m_iAccelerator = 0;
	m_fDamping = 0;
	m_fMass = 10;

}

const char * SphereSystemSimulator::getTestCasesStr()
{
	return "naive, accelerated, Accuracy comparison, Speed Comparison";
}

void SphereSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;

	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "step=0.01 min=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Radius", TW_TYPE_FLOAT, &m_fRadius, "step=0.01 min=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT16, &m_iNumSpheres, "");

	TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "step=0.01 min=0");

}

void SphereSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void SphereSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	/**

	Vec3 mpScale = Vec3(0.01f, 0.01f, 0.01f);
	Vec3 springColor = Vec3(0, 1, 0);

	//Setup Lighting
	DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0.97, 0.86, 1));

	//Draw MassPoints
	for each(MassPoint* mp in m_masspoints)
	{
		DUC->drawSphere(mp->Position, mpScale);
	}
	
	*/
}

void SphereSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	reset();

	switch (m_iTestCase)
	{
	case 0:
		cout << "Simulation with naive collision detection!\n";
		
		break;
	case 1:
		cout << "Simulation with accelerated collision detection!\n";
		
		break;
	case 2:
		cout << "Accuracy comparison of both collision detections!\n";
		
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void SphereSystemSimulator::externalForcesCalculations(float timeElapsed)
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
	m_externalForce.y += m_fMass * -9.81;

}

void SphereSystemSimulator::simulateTimestep(float timeStep)
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

		f_tmp[spring->masspoint1] += f_tmp_spring - dampingForce(f_tmp_spring, massPoint1->Velocity);
		f_tmp[spring->masspoint2] += -f_tmp_spring - dampingForce(-1 * f_tmp_spring, massPoint2->Velocity);
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
		f_estimate[spring->masspoint2] += -f_tmp_spring - dampingForce(-1 * f_tmp_spring, v_tmp[spring->masspoint2]);
	}

	// Integrate Velocity
	for (i = 0; i < m_masspoints.size(); i++) {
		if (m_masspoints[i]->isFixed)
			continue;
		m_masspoints[i]->Velocity += timeStep * ((f_estimate[i] + m_externalForce) / m_fMass);
	}


	if (m_iTestCase == 2)
	{
		//Collision calculation
		//i.e. make sure, that all MassPoints stay inside the cube
		for each(MassPoint* mp in m_masspoints)
		{
			//mp->Position.makeCeil(-0.5);
			//mp->Position.makeFloor(0.5);
			for (int i = 0; i < 3; ++i)
			{
				float j = mp->Position.value[i];

				if (j <= -0.5)
				{
					mp->Velocity.value[i] = m_fBouncyness * mp->Velocity.getAbsolutes().value[i];
					mp->Position.value[i] = -0.5;
				}
				else if (j >= 0.5)
				{
					mp->Velocity.value[i] = -1 * m_fBouncyness * mp->Velocity.getAbsolutes().value[i];
					mp->Position.value[i] = 0.5;
				}
			}

		}
	}


}

Vec3 SphereSystemSimulator::dampingForce(Vec3 springForce, Vec3 velocity)
{
	if (springForce.value[springForce.getAbsolutes().maxComponentId()] == 0)
		return Vec3(0, 0, 0);

	Vec3 force_unit = springForce / sqrt(pow(springForce.x, 2) + pow(springForce.y, 2) + pow(springForce.z, 2));

	return m_fDamping * force_unit * (velocity.x * force_unit.x + velocity.y * force_unit.y + velocity.z * force_unit.z);
}

void SphereSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void SphereSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
