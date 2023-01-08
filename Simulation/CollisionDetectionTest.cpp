
#include "CollisionDetectionTest.h"
#include "ObjParser.h"
#include "Poly.h"
#include "HullCollider.h"
#include "Collider.h"
#include "SphereCollider.h"
#include "BroadPhase.h"

CollisionDetectionTest& CollisionDetectionTest::GetInstance()
{
	static CollisionDetectionTest instance;
	return instance;
}

float myrandom()
{
	return (float)rand() / (float)RAND_MAX;
}
bool checkvaild(std::vector<glm::vec3>& history, glm::vec3 input)
{
	for (auto position : history)
	{
		if (glm::l2Norm(input, position) < 2.f)
		{
			return false;
		}
	}
	return true;
}
glm::vec3 generatePositon()
{
	static std::vector<glm::vec3> history;

Regen:
	auto positon = glm::vec3(5 + (myrandom() - .5) * 20, 20 + (myrandom() - .5) * 30, -5.0);
	if (!checkvaild(history, positon))
	{
		goto Regen;
	}
	return positon;
}

void CollisionDetectionTest::OnInit(GLFWwindow* window)
{
	Simulation::OnInit(window);

	HMesh mesh;
	ModelData model;
	Collider* collider;
	Body body;

	// slope
	Simulation::AddObjToScene("resources/floor.obj",
		glm::vec3(5, -3.0, 0),
		glm::angleAxis(60.0f, glm::vec3(0, 0, 1)),
		0.0f,
		glm::vec3(0.7, 0.7, 0.6));
	// floor
	Simulation::AddObjToScene("resources/floor.obj",
		glm::vec3(-5, -5.0, 0),
		glm::angleAxis(0.0f, glm::vec3(0, 0, 1)),
		0.0f,
		glm::vec3(0.7, 0.7, 0.1));

	srand(time(NULL));
	//goto BallSimu;
	//Create A blocker
	for (float spy = 1.0f; spy < 40.f; spy += 3.8f)
	{
		for (float spx = -3.0f; spx < 10.f; spx += 2.8f)
		{
			Simulation::AddObjToScene("resources/box.obj",
				glm::vec3(spx, spy, myrandom() * 5),
				glm::angleAxis(0.0f, glm::vec3(0, 0, 1)),
				1.0f,
				(glm::vec3(250.0 / 255, 193.7 / 255, 26.1 / 255)));
		}
	}
	goto END;

BallSimu:
	float radius = 0.5f;
	CreateSphere(radius, model);
	body.SetModelData(model);
	body.SetMass(1.f);
	body.SetColor(glm::vec3(1.0, 0.9, 0.3));
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));

	for (float spy = 1.0f; spy < 20.f; spy += 1.8f)
	{
		for (float spx = -3.0f; spx < 10.f; spx += 1.8f)
		{
			body.SetPosition(glm::vec3(spx, spy, myrandom() * 5));
			bodies.push_back(body);
			collider = new SphereCollider(radius);
			bodies.back().AddCollider(collider);
			colliders.push_back(collider);
		}
	}
END:
	BroadPhase::GetInstance().Init(colliders);
}

void CollisionDetectionTest::OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods)
{
	Simulation::OnKeyInput(window, key, code, action, mods);

	glm::vec3 p = bodies[1].GetPosition();
	glm::quat o = bodies[1].GetOrientation();
	glm::vec3 axis;
	glm::quat dq;
	float t = 0.1f;
	if (keys[GLFW_KEY_DOWN])
		bodies[1].SetPosition(p + t * glm::vec3(0, -1.0, 0));
	if (keys[GLFW_KEY_UP])
		bodies[1].SetPosition(p + t * glm::vec3(0, 1.0, 0));
	if (keys[GLFW_KEY_RIGHT])
		bodies[1].SetPosition(p + t * glm::vec3(1.0, 0, 0));
	if (keys[GLFW_KEY_LEFT])
		bodies[1].SetPosition(p + t * glm::vec3(-1.0, 0, 0));
	if (keys[GLFW_KEY_X])
	{
		axis = glm::vec3(1, 0, 0);
		dq = glm::quat(0, axis * t);
		o += dq * o * 0.5f;
		o = glm::normalize(o);
		bodies[1].SetOrientation(o);
	}
}