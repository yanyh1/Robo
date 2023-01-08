
#pragma once

#include "Collider.h"

class SphereCollider : public Collider
{
private:
	float radius;

public:
	SphereCollider(const float radius);
	glm::vec3 getCenterWorldPosition();

	float GetRadius();

	void CalculateMass();
};