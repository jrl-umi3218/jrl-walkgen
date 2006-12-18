/**	\file mathparse.h
 *	\brief	Declaration of the mathparse class.
 *	\authors Aurélien Pocheville
 *	\since 2004/05/12
 *	\history
 *		+ 2004/05/12 First Version.
 */ 

/*
 *	You can't copy/modify/disitribute this file without express acknowledge from the author(s).
 *	Please contact: slurdge@slurdge.org
 */

//Safety define
#pragma once
#if !defined(INCLUDE_MATHPARSE_H_2004_05_12)
#define INCLUDE_MATHPARSE_H_2004_05_12

#include "vector3.h"
#include "quaternion.h"
#include "matrix3.h"
#include "matrix4.h"
#include "transformation.h"
#include "asset/AssetLoader.h"
#include <sstream>

template <typename T>
bool	FillDataIn(const AssetNode& assNode, vector3<T>& v)
{
	if (assNode.hasAttribute("value"))
	{
		if (assNode.getAttribute("value") == "null")
		{
			v.x = v.y = v.z = 0;
		}
		else
		{
			std::stringstream s(assNode.getAttribute("value"));
			s >> v.x >> v.y >> v.z;
		}
		return true;
	}
	else if (assNode.hasText())
	{
		std::stringstream s(assNode.getText());
		s >> v.x >> v.y >> v.z;
		return true;
	}
	return false;
}

template <typename T>
bool	FillDataIn(const AssetNode& assNode, quaternion<T>& q)
{
	if (assNode.hasAttribute("value"))
	{
		if (assNode.getAttribute("value") == "identity")
		{
			q.x = q.y = q.z = 0.0;
			q.s = 1.0;
		}
		else if (assNode.getAttribute("value") == "null")
		{
			q.x = q.y = q.z = q.s = 0.0;
		}
		else
		{
			std::stringstream s(assNode.getAttribute("value"));
			s >> q.x >> q.y >> q.z >> q.s;
		}
		return true;
	}
	else if (assNode.hasText())
	{
		std::stringstream s(assNode.getText());
		s >> q.x >> q.y >> q.z >> q.s;
		return true;
	}
	return false;
}

template <typename T>
bool	FillDataIn(const AssetNode& assNode, matrix3<T>& m)
{
	if (assNode.hasText())
	{
		std::stringstream s(assNode.getText());
		s >> m.m[0] >> m.m[1] >> m.m[2] >> 
			 m.m[3] >> m.m[4] >> m.m[5] >> 
			 m.m[6] >> m.m[7] >> m.m[8];
		return true;
	}
	return false;
}

template <typename T>
bool	FillDataIn(const AssetNode& assNode, matrix4<T>& m)
{
	if (assNode.hasText())
	{
		std::stringstream s(assNode.getText());
		s >> m.m[0] >> m.m[1] >> m.m[2] >> m.m[3] >> 
			 m.m[4] >> m.m[5] >> m.m[6] >> m.m[7] >> 
			 m.m[8] >> m.m[9] >> m.m[10] >> m.m[11] >> 
			 m.m[12] >> m.m[13] >> m.m[14] >> m.m[15];
		return true;
	}
	return false;
}

template <typename T>
bool	FillDataIn(const AssetNode& assNode, transformation<T>& t)
{
	bool	bSucceed = true;
	if (!assNode.getAssetByType("quaternion",t.q))
		bSucceed = false;
	if (!assNode.getAssetByType("vector3",t.t))
		bSucceed = false;
	return bSucceed;
}

#endif // INCLUDE_MATHPARSE_H_2004_05_12

//End of file : mathparse.h

