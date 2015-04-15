#include "BoundingBoxManagerSingleton.h"

//  BoundingBoxManagerSingleton
BoundingBoxManagerSingleton* BoundingBoxManagerSingleton::m_pInstance = nullptr;
void BoundingBoxManagerSingleton::Init(void)
{
	m_nBoxs = 0;
}
void BoundingBoxManagerSingleton::Release(void)
{
	//Clean the list of Boxs
	for(int n = 0; n < m_nBoxs; n++)
	{
		//Make sure to release the memory of the pointers
		if(m_lBox[n] != nullptr)
		{
			delete m_lBox[n];
			m_lBox[n] = nullptr;
		}
	}
	m_lBox.clear();
	m_lMatrix.clear();
	m_lColor.clear();
	m_nBoxs = 0;
}
BoundingBoxManagerSingleton* BoundingBoxManagerSingleton::GetInstance()
{
	if(m_pInstance == nullptr)
	{
		m_pInstance = new BoundingBoxManagerSingleton();
	}
	return m_pInstance;
}
void BoundingBoxManagerSingleton::ReleaseInstance()
{
	if(m_pInstance != nullptr)
	{
		delete m_pInstance;
		m_pInstance = nullptr;
	}
}
//The big 3
BoundingBoxManagerSingleton::BoundingBoxManagerSingleton(){Init();}
BoundingBoxManagerSingleton::BoundingBoxManagerSingleton(BoundingBoxManagerSingleton const& other){ }
BoundingBoxManagerSingleton& BoundingBoxManagerSingleton::operator=(BoundingBoxManagerSingleton const& other) { return *this; }
BoundingBoxManagerSingleton::~BoundingBoxManagerSingleton(){Release();};
//Accessors
int BoundingBoxManagerSingleton::GetBoxTotal(void){ return m_nBoxs; }

//--- Non Standard Singleton Methods
void BoundingBoxManagerSingleton::GenerateBoundingBox(matrix4 a_mModelToWorld, String a_sInstanceName)
{
	MeshManagerSingleton* pMeshMngr = MeshManagerSingleton::GetInstance();
	//Verify the instance is loaded
	if(pMeshMngr->IsInstanceCreated(a_sInstanceName))
	{//if it is check if the Box has already been created
		int nBox = IdentifyBox(a_sInstanceName);
		if(nBox == -1)
		{
			//Create a new bounding Box
			BoundingBoxClass* pBB = new BoundingBoxClass();
			//construct its information out of the instance name
			pBB->GenerateOrientedBoundingBox(a_sInstanceName);
			//Push the Box back into the list
			m_lBox.push_back(pBB);
			//Push a new matrix into the list
			m_lMatrix.push_back(matrix4(IDENTITY));
			//Specify the color the Box is going to have
			m_lColor.push_back(vector3(1.0f));
			//Increase the number of Boxes
			m_nBoxs++;
		}
		else //If the box has already been created you will need to check its global orientation
		{
			m_lBox[nBox]->GenerateAxisAlignedBoundingBox(a_mModelToWorld);
		}
		nBox = IdentifyBox(a_sInstanceName);
		m_lMatrix[nBox] = a_mModelToWorld;
	}
}

void BoundingBoxManagerSingleton::SetBoundingBoxSpace(matrix4 a_mModelToWorld, String a_sInstanceName)
{
	int nBox = IdentifyBox(a_sInstanceName);
	//If the Box was found
	if(nBox != -1)
	{
		//Set up the new matrix in the appropriate index
		m_lMatrix[nBox] = a_mModelToWorld;
	}
}

int BoundingBoxManagerSingleton::IdentifyBox(String a_sInstanceName)
{
	//Go one by one for all the Boxs in the list
	for(int nBox = 0; nBox < m_nBoxs; nBox++)
	{
		//If the current Box is the one we are looking for we return the index
		if(a_sInstanceName == m_lBox[nBox]->GetName())
			return nBox;
	}
	return -1;//couldn't find it return with no index
}

void BoundingBoxManagerSingleton::AddBoxToRenderList(String a_sInstanceName)
{
	//If I need to render all
	if(a_sInstanceName == "ALL")
	{
		for(int nBox = 0; nBox < m_nBoxs; nBox++)
		{
			m_lBox[nBox]->AddAABBToRenderList(m_lMatrix[nBox], m_lColor[nBox], true);
		}
	}
	else
	{
		int nBox = IdentifyBox(a_sInstanceName);
		if(nBox != -1)
		{
			m_lBox[nBox]->AddAABBToRenderList(m_lMatrix[nBox], m_lColor[nBox], true);
		}
	}
}

void BoundingBoxManagerSingleton::CalculateCollision(void)
{
	//Create a placeholder for all center points
	std::vector<vector3> lCentroid;
	//for all Boxs...
	for(int nBox = 0; nBox < m_nBoxs; nBox++)
	{
		//Make all the Boxs white
		m_lColor[nBox] = vector3(1.0f);
		//Place all the centroids of Boxs in global space
		lCentroid.push_back(static_cast<vector3>(m_lMatrix[nBox] * vector4(m_lBox[nBox]->GetCentroid(), 1.0f)));
	}

	//Now the actual check
	for(int i = 0; i < m_nBoxs - 1; i++)
	{
		for(int j = i + 1; j < m_nBoxs; j++)
		{
			//If the distance between the center of both Boxs is less than the sum of their radius there is a collision
			//For this check we will assume they will be colliding unless they are not in the same space in X, Y or Z
			//so we place them in global positions
			vector3 v1Min = m_lBox[i]->GetMinimumAABB();
			vector3 v1Max = m_lBox[i]->GetMaximumAABB();

			vector3 v2Min = m_lBox[j]->GetMinimumAABB();
			vector3 v2Max = m_lBox[j]->GetMaximumAABB();

			bool bColliding = true;
			if(v1Max.x < v2Min.x || v1Min.x > v2Max.x)
				bColliding = false;
			else if(v1Max.y < v2Min.y || v1Min.y > v2Max.y)
				bColliding = false;
			else if(v1Max.z < v2Min.z || v1Min.z > v2Max.z)
				bColliding = false;

			if(bColliding)
			{
				vector3 axis[] = {
					vector3(1, 0, 0)
					, vector3(0, 1, 0)
					, vector3(0, 0, 1)
				};
				
				auto rotate1 = glm::mat3(m_lMatrix[i]);
				auto rotate2 = glm::mat3(m_lMatrix[j]);

				vector3 axis1[3];
				vector3 axis2[3];

				for(int k = 0; k < 3; k++)
				{
					axis1[k] = glm::normalize(rotate1 * axis[k]);
					axis2[k] = glm::normalize(rotate2 * axis[k]);
				}

				std::vector<vector3> testAxes;
				
				for(int k = 0; k < 3; k++)
				{
					testAxes.push_back(axis1[k]);
					testAxes.push_back(axis2[k]);
				}

				for(int k = 0; k < 3; k++)
				{
					for(int l = 0; l < 3; l++)
					{
						if(glm::abs(axis1[k]) != glm::abs(axis2[l]))
						{
							testAxes.push_back(glm::normalize(glm::cross(axis1[k], axis2[l])));
						}
					}
				}
				
				std::vector<vector3> box1Points;
				std::vector<vector3> box2Points;

				auto center1 = rotate1 * m_lBox[i]->GetCentroid() + vector3(m_lMatrix[i][3]);
				auto center2 = rotate2 * m_lBox[j]->GetCentroid() + vector3(m_lMatrix[j][3]);
				
				auto sizes1 = m_lBox[i]->getSizes() / 2.0f;
				auto sizes2 = m_lBox[j]->getSizes() / 2.0f;

				for(int x = -1; x <= 1; x += 2) {
					for(int y = -1; y <= 1; y += 2) {
						for(int z = -1; z <= 1; z += 2) {
							vector3 mult(x, y, z);
							vector3 point1(0.0f, 0.0f, 0.0f), point2(0.0f, 0.0f, 0.0f);
							for(int k = 0; k < 3; k++)
							{
								point1 += axis1[k] * (sizes1[k] * mult[k]);
								point2 += axis2[k] * (sizes2[k] * mult[k]);
							}
							box1Points.push_back(point1);
							box2Points.push_back(point2);
						}
					}
				}

				bColliding = true;

				auto centerDiff = center2 - center1;

				for(auto testAxis : testAxes)
				{
					auto dist = abs(glm::dot(testAxis, centerDiff));
					auto halfDim1 = 0.0f;
					for(auto p : box1Points)
					{
						halfDim1 = std::max(halfDim1, abs(glm::dot(p, testAxis)));
					}
					auto halfDim2 = 0.0f;
					for(auto p : box2Points)
					{
						halfDim2 = std::max(halfDim2, abs(glm::dot(p, testAxis)));
					}
					if(dist > halfDim1 + halfDim2) {
						bColliding = false;
						// todo add axis to list of planes
					}
				}
				if(bColliding){
					m_lColor[i] = m_lColor[j] = MERED; //We make the Boxes red
				}
			}
			
		}
	}
}