#include "SphereSystemSimulator.h"
#include <math.h>
#include "MassSpringSystemSimulator.h"
              //Rückgabe //Eingabe 
std::function<float(float)> SphereSystemSimulator::m_Kernels[5] = {
	[](float x) {return 1.0f; },              // Constant, m_iKernel = 0
	[](float x) {return 1.0f - x; },          // Linear, m_iKernel = 1, as given in the exercise Sheet, x = d/2r
	[](float x) {return (1.0f - x)*(1.0f - x); }, // Quadratic, m_iKernel = 2
	[](float x) {return 1.0f / (x)-1.0f; },     // Weak Electric Charge, m_iKernel = 3
	[](float x) {return 1.0f / (x*x) - 1.0f; },   // Electric Charge, m_iKernel = 4
};


SphereSystemSimulator::SphereSystemSimulator()
{
	m_iTestCase = 0;
	m_iAccelerator = 0;
	m_fDamping = 0.7;
	m_fMass = 0.1;
	m_fRadius = 0.05;
	m_iNumSpheres = 100;
	m_fLambda = 10;
	m_iKernel = 2;
}

const char * SphereSystemSimulator::getTestCasesStr()
{
	return "naive, accelerated, Accuracy comparison, Speed Comparison";
}

void SphereSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;

	if (m_iTestCase == 3)
	{
		//Define Enum for Accelerator Selector
		TwType TW_TYPE_ACCELERATOR = TwDefineEnumFromString("Accelerator", "Naive, Grid");

		TwAddVarRW(DUC->g_pTweakBar, "Accelerator", TW_TYPE_ACCELERATOR, &m_iAccelerator, "");
	}

	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "step=0.01 min=0.01");  //User input
	TwAddVarRW(DUC->g_pTweakBar, "Radius", TW_TYPE_FLOAT, &m_fRadius, "step=0.01 min=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Lamda", TW_TYPE_FLOAT, &m_fLambda, "step=0.01 min=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT16, &m_iNumSpheres, "");

	TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "step=0.01 min=0");

}

float SphereSystemSimulator::length(Vec3 distance) {
	return sqrt(pow(distance.x, 2) + pow(distance.y, 2) + pow(distance.z, 2));
}
void SphereSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	m_spheres.clear();
	m_spheres2.clear();
	m_occupiedCells.clear();
	m_adjacentCells.clear();

	m_grid.clear();
}

void SphereSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{

	if (m_iTestCase == 0 || m_iTestCase == 1)
	{
		Vec3 sphereScale = Vec3(m_fRadius);
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0, 1, 0));
		for each(Sphere* sphere in m_spheres) {
			DUC->drawSphere(sphere->Position, sphereScale);
		}
	}
	else if (m_iTestCase == 2)
	{
		Vec3 sphereScale = Vec3(m_fRadius);
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0, 1, 0));
		for each(Sphere* sphere in m_spheres)
		{
			DUC->drawSphere(sphere->Position, sphereScale);
		}

		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(1, 0, 0));
		for each(Sphere* sphere in m_spheres2)
		{
			DUC->drawSphere(sphere->Position, sphereScale);
		}		
	}
}

void SphereSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	reset();
	
	for (int i = 0; i < m_iNumSpheres; i++)
	{

		Sphere* sphere = new Sphere();
		sphere->Velocity = (0, 0, 0);
		sphere->Position.x = -0.44 + (i % 9) * 0.105 + (i / 81) * 0.01;
		sphere->Position.z = -0.44 + (i / 9) % 9 * 0.105 + (i / 81) * 0.01; //für alle vollen 10 Schritte
		sphere->Position.y = 0.44 - (i / 81) * 0.11;

		m_spheres.push_back(sphere);


		Sphere* sphere2 = new Sphere();
		sphere2->Velocity = (0, 0, 0);
		sphere2->Position.x = -0.44 + (i % 9) * 0.105 + (i / 81) * 0.01;
		sphere2->Position.z = -0.44 + (i / 9) % 9 * 0.105 + (i / 81) * 0.01; //für alle vollen 10 Schritte
		sphere2->Position.y = 0.44 - (i / 81) * 0.11;

		m_spheres2.push_back(sphere2);
	}


	int numCellsPerRow;

	switch (m_iTestCase)
	{
	case 0:
		cout << "Simulation with naive collision detection!\n";
		
		m_iAccelerator = NAIVEACC;
		
		

		break;
	case 1:
		cout << "Simulation with accelerated collision detection!\n";
		
		m_iAccelerator = GRIDACC;

		numCellsPerRow = 1 / (2 * m_fRadius);

		m_grid.resize(pow(numCellsPerRow, 3));
		
		for each(vector<int> cell in m_grid)
		{
			cell.clear();
		}

		m_adjacentCells.resize(m_grid.size());

		//Calculate adjacent cells
		for (int i = 0; i < m_grid.size() - 1; ++i)
		{
			int iX = i % numCellsPerRow;
			int iZ = (i / numCellsPerRow) % numCellsPerRow;
			int iY = i / (numCellsPerRow * numCellsPerRow);

			for (int j = i + 1; j < m_grid.size(); ++j)
			{
				int jX = j % numCellsPerRow;
				int jZ = (j / numCellsPerRow) % numCellsPerRow;
				int jY = j / (numCellsPerRow * numCellsPerRow);

				if (abs(iX - jX) <= 1 && abs(iY - jY) <= 1 && abs(iZ - jZ) <= 1)
				{
					m_adjacentCells[i].push_back(j);
					//m_adjacentCells[j].push_back(i);
				}
			}
		}

		break;
	case 2:
		cout << "Accuracy comparison of both collision detections!\n";
		
		numCellsPerRow = 1 / (2 * m_fRadius);

		m_grid.resize(pow(numCellsPerRow, 3));

		for each(vector<int> cell in m_grid)
		{
			cell.clear();
		}

		m_adjacentCells.resize(m_grid.size());

		//Calculate adjacent cells
		for (int i = 0; i < m_grid.size() - 1; ++i)
		{
			int iX = i % numCellsPerRow;
			int iZ = (i / numCellsPerRow) % numCellsPerRow;
			int iY = i / (numCellsPerRow * numCellsPerRow);

			for (int j = i + 1; j < m_grid.size(); ++j)
			{
				int jX = j % numCellsPerRow;
				int jZ = (j / numCellsPerRow) % numCellsPerRow;
				int jY = j / (numCellsPerRow * numCellsPerRow);

				if (abs(iX - jX) <= 1 && abs(iY - jY) <= 1 && abs(iZ - jZ) <= 1)
				{
					m_adjacentCells[i].push_back(j);
					//m_adjacentCells[j].push_back(i);
				}
			}
		}
		
		break;

	case 3:
		cout << "Speed comparison of both collision detections!\n";

		numCellsPerRow = 1 / (2 * m_fRadius);

		m_grid.resize(pow(numCellsPerRow, 3));

		for each(vector<int> cell in m_grid)
		{
			cell.clear();
		}

		m_adjacentCells.resize(m_grid.size());

		//Calculate adjacent cells
		for (int i = 0; i < m_grid.size() - 1; ++i)
		{
			int iX = i % numCellsPerRow;
			int iZ = (i / numCellsPerRow) % numCellsPerRow;
			int iY = i / (numCellsPerRow * numCellsPerRow);

			for (int j = i + 1; j < m_grid.size(); ++j)
			{
				int jX = j % numCellsPerRow;
				int jZ = (j / numCellsPerRow) % numCellsPerRow;
				int jY = j / (numCellsPerRow * numCellsPerRow);

				if (abs(iX - jX) <= 1 && abs(iY - jY) <= 1 && abs(iZ - jZ) <= 1)
				{
					m_adjacentCells[i].push_back(j);
					//m_adjacentCells[j].push_back(i);
				}
			}
		}

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
		//m_externalForce = inputWorld;
		m_externalForce = Vec3(0, 0, 0);
	}
	else {
		m_externalForce = Vec3(0, 0, 0);
		//m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}

	//Add Gravity
	m_externalForce.y += m_fMass * -9.81;

}

int SphereSystemSimulator::PositionToCell(Vec3 Position)
{
	int numCellsPerRow = 1 / (2 * m_fRadius);

	//Map positions to [0;1]
	Position.x += 0.5;
	Position.y += 0.5;
	Position.z += 0.5;

	//Map positions to ints in [0;numCellsPerRow)
	int gridPosX = int(Position.x * numCellsPerRow);
	int gridPosY = int(Position.y * numCellsPerRow);
	int gridPosZ = int(Position.z * numCellsPerRow);

	return gridPosX + gridPosY * numCellsPerRow * numCellsPerRow + gridPosZ * numCellsPerRow;
}

bool VectorContains(vector<int> vec, int value)
{
	for each(int i in vec)
	{
		if (i == value)
			return true;
	}

	return false;
}

void SphereSystemSimulator::simulateTimestep(float timeStep)
{

	externalForcesCalculations(timeStep);

	if (m_iTestCase == 0 || m_iTestCase == 1 || m_iTestCase == 3)
	{
		if (m_iAccelerator == GRIDACC)
		{
			for (int i = 0; i < m_grid.size(); ++i)
			{
				m_grid[i].clear();
			}

			m_occupiedCells.clear();

			//Store balls in cells
			for (int i = 0; i < m_iNumSpheres; ++i)
			{
				Sphere* sphere = m_spheres[i];

				int gridPos = PositionToCell(m_spheres[i]->Position);
				m_grid[gridPos].push_back(i);

				if (m_grid[gridPos].size() == 1)
					m_occupiedCells.push_back(gridPos);
			}

			std::sort(m_occupiedCells.begin(), m_occupiedCells.end());
		}

		// Midpoint position integration based on last position and velocity
		vector<Vec3> pos_tmp;
		for each(Sphere* massPoint in m_spheres) {
			pos_tmp.push_back(massPoint->Position + (timeStep / 2) * massPoint->Velocity);
		}

		// Calculate midpoint spring forces using the updated pos_tmp
		vector<Vec3> f_tmp;
		f_tmp.resize(m_spheres.size());

		/*for each(Spring* spring in m_springs) {
		Sphere* massPoint1 = m_spheres[spring->masspoint1];
		Sphere* massPoint2 = m_spheres[spring->masspoint2];
		Vec3 f_tmp_spring = springForce(pos_tmp[spring->masspoint1], pos_tmp[spring->masspoint2], spring->initialLength);

		f_tmp[spring->masspoint1] += f_tmp_spring - dampingForce(f_tmp_spring, massPoint1->Velocity);
		f_tmp[spring->masspoint2] += -f_tmp_spring - dampingForce(-1 * f_tmp_spring, massPoint2->Velocity);
		}*/
		if (m_iAccelerator == NAIVEACC)
		{
			for (int i = 0; i < m_spheres.size() - 1; ++i)
			{
				for (int j = i + 1; j < m_spheres.size(); j++)
				{
					Vec3 distance = m_spheres[i]->Position - m_spheres[j]->Position;
					if (length(distance) < m_fRadius * 2)
					{
						Vec3 force = m_fLambda * m_Kernels[m_iKernel](length(distance)) * distance / length(distance);
						f_tmp[i] += force;
						f_tmp[j] -= force;

						f_tmp[i] -= dampingForce(force, m_spheres[i]->Velocity);
						f_tmp[j] -= dampingForce(-1 * force, m_spheres[j]->Velocity);
					}
				}
			}
		}
		else if (m_iAccelerator == GRIDACC)
		{
			vector<int> processedCells;

			for (int i = 0; i < m_occupiedCells.size(); ++i)
			{
				int iID = m_occupiedCells[i];

				if (!VectorContains(processedCells, iID))
				{
					processedCells.push_back(iID);

					if (m_grid[iID].size() > 0)
					{
						//test all spheres in current cell
						if (m_grid[iID].size() > 1)
						{
							for (int j = 0; j < m_grid[iID].size() - 1; ++j)
							{
								int jID = m_grid[iID][j];
								for (int k = j + 1; k < m_grid[iID].size(); ++k)
								{
									int kID = m_grid[iID][k];
									Vec3 distance = m_spheres[jID]->Position - m_spheres[kID]->Position;
									if (length(distance) < m_fRadius * 2)
									{
										Vec3 force = m_fLambda * m_Kernels[m_iKernel](length(distance)) * distance / length(distance);
										f_tmp[jID] += force;
										f_tmp[kID] -= force;

										f_tmp[jID] -= dampingForce(force, m_spheres[jID]->Velocity);
										f_tmp[kID] -= dampingForce(-1 * force, m_spheres[kID]->Velocity);
									}
								}
							}
						}

						//Test against all adjacent cells
						for each(int adjCell in m_adjacentCells[iID])
						{
							if (!VectorContains(processedCells, adjCell))
							{
								if (!m_grid[adjCell].empty())
								{
									for (int j = 0; j < m_grid[iID].size(); ++j)
									{
										int jID = m_grid[iID][j];
										for (int k = 0; k < m_grid[adjCell].size(); ++k)
										{
											int kID = m_grid[adjCell][k];

											Vec3 distance = m_spheres[jID]->Position - m_spheres[kID]->Position;
											if (length(distance) < m_fRadius * 2)
											{
												Vec3 force = m_fLambda * m_Kernels[m_iKernel](length(distance)) * distance / length(distance);
												f_tmp[jID] += force;
												f_tmp[kID] -= force;

												f_tmp[jID] -= dampingForce(force, m_spheres[jID]->Velocity);
												f_tmp[kID] -= dampingForce(-1 * force, m_spheres[kID]->Velocity);
											}
										}
									}
								}
							}
						}
					}
				}
			}

		}



		// Integrate velocity using midpoint spring forces and these new values to integrate the position
		vector<Vec3> v_tmp;
		unsigned int i;
		for (i = 0; i < m_spheres.size(); i++) {
			Sphere* m_sphere = m_spheres[i];
			v_tmp.push_back(m_spheres[i]->Velocity + (timeStep / 2) * (f_tmp[i] / m_fMass));

			m_spheres[i]->Position += timeStep * v_tmp[i];
		}


		// Integrate Velocity
		for (i = 0; i < m_spheres.size(); i++) {

			m_spheres[i]->Velocity += timeStep * ((f_tmp[i] + m_externalForce) / m_fMass);
		}


		//Collision calculation
		for each(Sphere* mp in m_spheres)
		{
			//mp->Position.makeCeil(-0.5);
			//mp->Position.makeFloor(0.5);
			for (int i = 0; i < 3; ++i)
			{
				float j = mp->Position.value[i];

				if (j <= -0.5 + m_fRadius)
				{
					mp->Velocity.value[i] = 0.1 * mp->Velocity.getAbsolutes().value[i];
					mp->Position.value[i] = -0.5 + m_fRadius;
				}
				else if (j >= 0.5 - m_fRadius)
				{
					mp->Velocity.value[i] = -1 * 0.1 * mp->Velocity.getAbsolutes().value[i];
					mp->Position.value[i] = 0.5 - m_fRadius;
				}
			}
		}
	}
	else if (m_iTestCase == 2)
	{
		//First calculate 1 spheresystem with naive collision detection
		if (true)
		{

			// Midpoint position integration based on last position and velocity
			vector<Vec3> pos_tmp;
			for each(Sphere* massPoint in m_spheres) {
				pos_tmp.push_back(massPoint->Position + (timeStep / 2) * massPoint->Velocity);
			}

			// Calculate midpoint spring forces using the updated pos_tmp
			vector<Vec3> f_tmp;
			f_tmp.resize(m_spheres.size());

			for (int i = 0; i < m_spheres.size() - 1; ++i)
			{
				for (int j = i + 1; j < m_spheres.size(); j++)
				{
					Vec3 distance = m_spheres[i]->Position - m_spheres[j]->Position;
					if (length(distance) < m_fRadius * 2)
					{
						Vec3 force = m_fLambda * m_Kernels[m_iKernel](length(distance)) * distance / length(distance);
						f_tmp[i] += force;
						f_tmp[j] -= force;

						f_tmp[i] -= dampingForce(force, m_spheres[i]->Velocity);
						f_tmp[j] -= dampingForce(-1 * force, m_spheres[j]->Velocity);
					}
				}
			}

			// Integrate velocity using midpoint spring forces and these new values to integrate the position
			vector<Vec3> v_tmp;
			unsigned int i;
			for (i = 0; i < m_spheres.size(); i++) {
				Sphere* m_sphere = m_spheres[i];
				v_tmp.push_back(m_spheres[i]->Velocity + (timeStep / 2) * (f_tmp[i] / m_fMass));

				m_spheres[i]->Position += timeStep * v_tmp[i];
			}


			// Integrate Velocity
			for (i = 0; i < m_spheres.size(); i++) {

				m_spheres[i]->Velocity += timeStep * ((f_tmp[i] + m_externalForce) / m_fMass);
			}


			//Collision calculation
			for each(Sphere* mp in m_spheres)
			{
				//mp->Position.makeCeil(-0.5);
				//mp->Position.makeFloor(0.5);
				for (int i = 0; i < 3; ++i)
				{
					float j = mp->Position.value[i];

					if (j <= -0.5 + m_fRadius)
					{
						mp->Velocity.value[i] = 0.1 * mp->Velocity.getAbsolutes().value[i];
						mp->Position.value[i] = -0.5 + m_fRadius;
					}
					else if (j >= 0.5 - m_fRadius)
					{
						mp->Velocity.value[i] = -1 * 0.1 * mp->Velocity.getAbsolutes().value[i];
						mp->Position.value[i] = 0.5 - m_fRadius;
					}
				}
			}
		}

		//Then calculate the other system with grid accelerated collisison detection
		if (true)
		{
			for (int i = 0; i < m_grid.size(); ++i)
			{
				m_grid[i].clear();
			}

			m_occupiedCells.clear();

			//Store balls in cells
			for (int i = 0; i < m_iNumSpheres; ++i)
			{
				Sphere* sphere = m_spheres2[i];

				int gridPos = PositionToCell(m_spheres2[i]->Position);
				m_grid[gridPos].push_back(i);

				if (m_grid[gridPos].size() == 1)
					m_occupiedCells.push_back(gridPos);
			}

			std::sort(m_occupiedCells.begin(), m_occupiedCells.end());

			// Midpoint position integration based on last position and velocity
			vector<Vec3> pos_tmp;
			for each(Sphere* massPoint in m_spheres2) {
				pos_tmp.push_back(massPoint->Position + (timeStep / 2) * massPoint->Velocity);
			}

			// Calculate midpoint spring forces using the updated pos_tmp
			vector<Vec3> f_tmp;
			f_tmp.resize(m_spheres2.size());

			vector<int> processedCells;

			for (int i = 0; i < m_occupiedCells.size(); ++i)
			{
				int iID = m_occupiedCells[i];

				if (!VectorContains(processedCells, iID))
				{
					processedCells.push_back(iID);

					if (m_grid[iID].size() > 0)
					{
						//test all spheres in current cell
						if (m_grid[iID].size() > 1)
						{
							for (int j = 0; j < m_grid[iID].size() - 1; ++j)
							{
								int jID = m_grid[iID][j];
								for (int k = j + 1; k < m_grid[iID].size(); ++k)
								{
									int kID = m_grid[iID][k];
									Vec3 distance = m_spheres2[jID]->Position - m_spheres2[kID]->Position;
									if (length(distance) < m_fRadius * 2)
									{
										Vec3 force = m_fLambda * m_Kernels[m_iKernel](length(distance)) * distance / length(distance);
										f_tmp[jID] += force;
										f_tmp[kID] -= force;

										f_tmp[jID] -= dampingForce(force, m_spheres2[jID]->Velocity);
										f_tmp[kID] -= dampingForce(-1 * force, m_spheres2[kID]->Velocity);
									}
								}
							}
						}

						//Test against all adjacent cells
						for each(int adjCell in m_adjacentCells[iID])
						{
							if (!VectorContains(processedCells, adjCell))
							{
								if (!m_grid[adjCell].empty())
								{
									for (int j = 0; j < m_grid[iID].size(); ++j)
									{
										int jID = m_grid[iID][j];
										for (int k = 0; k < m_grid[adjCell].size(); ++k)
										{
											int kID = m_grid[adjCell][k];

											Vec3 distance = m_spheres2[jID]->Position - m_spheres2[kID]->Position;
											if (length(distance) < m_fRadius * 2)
											{
												Vec3 force = m_fLambda * m_Kernels[m_iKernel](length(distance)) * distance / length(distance);
												f_tmp[jID] += force;
												f_tmp[kID] -= force;

												f_tmp[jID] -= dampingForce(force, m_spheres2[jID]->Velocity);
												f_tmp[kID] -= dampingForce(-1 * force, m_spheres2[kID]->Velocity);
											}
										}
									}
								}
							}
						}
					}
				}
			}


			// Integrate velocity using midpoint spring forces and these new values to integrate the position
			vector<Vec3> v_tmp;
			unsigned int i;
			for (i = 0; i < m_spheres2.size(); i++) {
				Sphere* m_sphere = m_spheres2[i];
				v_tmp.push_back(m_spheres2[i]->Velocity + (timeStep / 2) * (f_tmp[i] / m_fMass));

				m_spheres2[i]->Position += timeStep * v_tmp[i];
			}


			// Integrate Velocity
			for (i = 0; i < m_spheres2.size(); i++) {

				m_spheres2[i]->Velocity += timeStep * ((f_tmp[i] + m_externalForce) / m_fMass);
			}


			//Collision calculation
			for each(Sphere* mp in m_spheres2)
			{
				//mp->Position.makeCeil(-0.5);
				//mp->Position.makeFloor(0.5);
				for (int i = 0; i < 3; ++i)
				{
					float j = mp->Position.value[i];

					if (j <= -0.5 + m_fRadius)
					{
						mp->Velocity.value[i] = 0.1 * mp->Velocity.getAbsolutes().value[i];
						mp->Position.value[i] = -0.5 + m_fRadius;
					}
					else if (j >= 0.5 - m_fRadius)
					{
						mp->Velocity.value[i] = -1 * 0.1 * mp->Velocity.getAbsolutes().value[i];
						mp->Position.value[i] = 0.5 - m_fRadius;
					}
				}
			}
		}
	}
}

Vec3 SphereSystemSimulator::dampingForce(Vec3 repulsionForce, Vec3 velocity)
{
	if (repulsionForce.value[repulsionForce.getAbsolutes().maxComponentId()] == 0)
		return Vec3(0, 0, 0);

	Vec3 force_unit = repulsionForce / sqrt(pow(repulsionForce.x, 2) + pow(repulsionForce.y, 2) + pow(repulsionForce.z, 2));

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
