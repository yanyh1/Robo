
#pragma once

#include <glm/glm.hpp>
#include "Geometry.h"
#include "ConstraintCommon.h"
#include "Body.h"

// contact data for a collider pair
class Contact
{
private:
	Body* A;				// the body pair
	Body* B;				// involved in this contact

	glm::vec3 position;		// the contact point in world space
	glm::vec3 normal;		// the directon in which the colliders must be spearated (by convention always from A to B)
	glm::vec3 tangent[2];		// mutually perendicular directions for friction
	float penetration;		// the amount of overlap b/w the colliders

	// for clamping the lagrangian
	float impulseSumN;
	float impulseSumT[2];

	glm::vec3 rA;
	glm::vec3 rB;

	Jacobian JN;
	Jacobian JT[2];

	// Bias is the "work" term in the velocity constraint equation
	// Accounts for position drift, and bounce(restitution)
	float bias;

	float kn;		// effective mass for normal impulse
	float kt[2];	// effective mass for tangent impulse

private:
	// Projection of relative velocity along the normal 
	float CalculateNormalConstraint() const;

	// Calculates the Jacobian for the normal/tangent part
	void CalculateJacobian(Jacobian& J, const glm::vec3& axis);

	void CalculateBias();

public:
	Contact(Body* A, Body* B, const glm::vec3& position, const glm::vec3& normal, const float penetration);

	glm::vec3 GetPosition() const { return position; }
	Body* GetBodyA() const { return A; }
	Body* GetBodyB() const { return B; }

	glm::vec3 GetTangent(int i) const { return tangent[i]; }

	// Solves the contact by applying impulses
	void SolveVelocities(Velocity& A, Velocity& B);

	void SolvePositions(Position& A, Position& B);

	friend struct Manifold;
};

struct Manifold
{
	std::vector<Contact> contacts;	// To Do : Duplicated data. Store pointer to contact and contact size

	void SolveVelocities();

	void SolvePositions();
};