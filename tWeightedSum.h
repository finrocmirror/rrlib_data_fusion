//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    tWeightedSum.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-25
 *
 * \brief   Contains tWeightedSum
 *
 * \b tWeightedSum
 *
 * A few words for tWeightedSum
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__data_fusion__tWeightedSum_h__
#define __rrlib__data_fusion__tWeightedSum_h__

#include "rrlib/data_fusion/tDataFusion.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_MATH_PRESENT_
#include "rrlib/math/tAngle.h"
#include "rrlib/math/tPose2D.h"
#include "rrlib/math/tPose3D.h"
#endif

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace data_fusion
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Short description of tWeightedSumBase
/*! A more detailed description of tWeightedSumBase, which
 *  Tobias Foehst hasn't done yet !!
 */
template <
typename TSample,
         template <typename> class TChannel = channel::LastValue
         >
class tWeightedSum : public tDataFusion<TSample, TChannel>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const char *GetLogDescription() const
  {
    return "tWeightedSum";
  }

  virtual const bool HasValidState() const
  {
    return true;
  }

  virtual const TSample CalculateFusedValue(const std::vector<TChannel<TSample>> &channels)
  {
    char buffer[sizeof(TSample)];
    memset(buffer, 0, sizeof(buffer));
    TSample *accumulated = new(buffer) TSample;
    double max_weight = 0;
    for (typename std::vector<TChannel<TSample>>::const_iterator it = channels.begin(); it != channels.end(); ++it)
    {
      max_weight = std::max(max_weight, it->GetKey());
    }

    if (max_weight != 0.0)
    {

      for (typename std::vector<TChannel<TSample>>::const_iterator it = channels.begin(); it != channels.end(); ++it)
      {
        double weight = it->GetKey() / max_weight;
        *accumulated += it->GetSample() * weight;
      }
    }

    return *accumulated;
  }

  virtual void ResetStateImplementation()
  {}

  virtual void EnterNextTimestepImplementation()
  {}

};

#ifdef _LIB_RRLIB_MATH_PRESENT_

template <typename TElement, typename TUnitPolicy, typename TSignedPolicy, template <typename> class TChannel>
class tWeightedSum<math::tAngle<TElement, TUnitPolicy, TSignedPolicy>, TChannel> : public tDataFusion<math::tAngle<TElement, TUnitPolicy, TSignedPolicy>, TChannel>
{

  typedef math::tAngle<TElement, TUnitPolicy, TSignedPolicy> tAngle;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const char *GetLogDescription() const
  {
    return "tWeightedSum<math::tAngle<...>>";
  }

  virtual const bool HasValidState() const
  {
    return true;
  }

  virtual const tAngle CalculateFusedValue(const std::vector<TChannel<tAngle>> &channels)
  {
    double accumulated_value = 0;
    double max_weight = 0;
    for (typename std::vector<TChannel<tAngle>>::const_iterator it = channels.begin(); it != channels.end(); ++it)
    {
      max_weight = std::max(max_weight, it->GetKey());
    }

    if (max_weight != 0.0)
    {
      for (typename std::vector<TChannel<tAngle>>::const_iterator it = channels.begin(); it != channels.end(); ++it)
      {
        double weight = it->GetKey() / max_weight;
        accumulated_value += static_cast<double>(it->GetSample()) * weight;
      }
    }

    return tAngle(accumulated_value);
  }

  virtual void ResetStateImplementation()
  {}

  virtual void EnterNextTimestepImplementation()
  {}

};

template <template <typename> class TChannel>
class tWeightedSum<math::tPose2D, TChannel> : public tDataFusion<math::tPose2D, TChannel>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const char *GetLogDescription() const
  {
    return "tWeightedSum<math::tPose2D>";
  }

  virtual const bool HasValidState() const
  {
    return true;
  }

  virtual const math::tPose2D CalculateFusedValue(const std::vector<TChannel<math::tPose2D>> &channels)
  {
    math::tVec2d accumulated_position;
    math::tAngle<double, math::angle::Radian, math::angle::NoWrap> accumulated_yaw;
    double max_weight = 0;
    for (typename std::vector<TChannel<math::tPose2D>>::const_iterator it = channels.begin(); it != channels.end(); ++it)
    {
      max_weight = std::max(max_weight, it->GetKey());
    }

    if (max_weight != 0.0)
    {
      for (typename std::vector<TChannel<math::tPose2D>>::const_iterator it = channels.begin(); it != channels.end(); ++it)
      {
        double weight = it->GetKey() / max_weight;
        accumulated_position += it->GetSample().Position() * weight;
        accumulated_yaw += it->GetSample().Yaw() * weight;
      }
    }

    return math::tPose2D(accumulated_position, math::tAngleRad(accumulated_yaw));
  }

  virtual void ResetStateImplementation()
  {}

  virtual void EnterNextTimestepImplementation()
  {}

};

template <template <typename> class TChannel>
class tWeightedSum<math::tPose3D, TChannel> : public tDataFusion<math::tPose3D, TChannel>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const char *GetLogDescription() const
  {
    return "tWeightedSum<math::tPose3D>";
  }

  virtual const bool HasValidState() const
  {
    return true;
  }

  virtual const math::tPose3D CalculateFusedValue(const std::vector<TChannel<math::tPose3D>> &channels)
  {
    math::tVec3d accumulated_position;
    double accumulated_roll = 0;
    double accumulated_pitch = 0;
    double accumulated_yaw = 0;
    double max_weight = 0;
    for (typename std::vector<TChannel<math::tPose3D>>::const_iterator it = channels.begin(); it != channels.end(); ++it)
    {
      max_weight = std::max(max_weight, it->GetKey());
    }

    if (max_weight != 0.0)
    {
      for (typename std::vector<TChannel<math::tPose3D>>::const_iterator it = channels.begin(); it != channels.end(); ++it)
      {
        double weight = it->GetKey() / max_weight;
        accumulated_position += it->GetSample().Position() * weight;
        accumulated_roll += it->GetSample().Roll() * weight;
        accumulated_pitch += it->GetSample().Pitch() * weight;
        accumulated_yaw += it->GetSample().Yaw() * weight;
      }
    }

    return math::tPose3D(accumulated_position, math::tAngleRad(accumulated_roll), math::tAngleRad(accumulated_pitch), math::tAngleRad(accumulated_yaw));
  }

  virtual void ResetStateImplementation()
  {}

  virtual void EnterNextTimestepImplementation()
  {}

};

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
