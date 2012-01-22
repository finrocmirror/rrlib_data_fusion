//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    tWeightedAverage.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-25
 *
 * \brief   Contains tWeightedAverage
 *
 * \b tWeightedAverage
 *
 * A few words for tWeightedAverage
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__data_fusion__tWeightedAverage_h__
#define __rrlib__data_fusion__tWeightedAverage_h__

#include "rrlib/data_fusion/tDataFusion.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_MATH_PRESENT_
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
//! Short description of tWeightedAverageBase
/*! A more detailed description of tWeightedAverageBase, which
 *  Tobias Foehst hasn't done yet !!
 */
template <
typename TSample,
         template <typename> class TChannel = channel::LastValue
         >
class tWeightedAverage : public tDataFusion<TSample, TChannel>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const char *GetLogDescription() const
  {
    return "tWeightedAverage";
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
    double accumulated_weights = 0;
    for (typename std::vector<TChannel<TSample>>::const_iterator it = channels.begin(); it != channels.end(); ++it)
    {
      *accumulated += it->GetSample() * it->GetKey();
      accumulated_weights += it->GetKey();
    }
    return *accumulated * (1.0 / accumulated_weights);
  }

  virtual void ResetStateImplementation()
  {}

  virtual void EnterNextTimestepImplementation()
  {}

};

#ifdef _LIB_RRLIB_MATH_PRESENT_

template <template <typename> class TChannel>
class tWeightedAverage<math::tPose2D, TChannel> : public tDataFusion<math::tPose2D, TChannel>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const char *GetLogDescription() const
  {
    return "tWeightedAverage<math::tPose2D>";
  }

  virtual const bool HasValidState() const
  {
    return true;
  }

  virtual const math::tPose2D CalculateFusedValue(const std::vector<TChannel<math::tPose2D>> &channels)
  {
    math::tVec2d accumulated_position;
    double accumulated_yaw = 0;
    double accumulated_weights = 0;
    for (typename std::vector<TChannel<math::tPose2D>>::const_iterator it = channels.begin(); it != channels.end(); ++it)
    {
      accumulated_position += it->GetSample().Position() * it->GetKey();
      accumulated_yaw += it->GetSample().Yaw() * it->GetKey();
      accumulated_weights += it->GetKey();
    }
    double factor = 1.0 / accumulated_weights;
    return math::tPose2D(accumulated_position * factor, accumulated_yaw * factor);
  }

  virtual void ResetStateImplementation()
  {}

  virtual void EnterNextTimestepImplementation()
  {}

};

template <template <typename> class TChannel>
class tWeightedAverage<math::tPose3D, TChannel> : public tDataFusion<math::tPose3D, TChannel>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const char *GetLogDescription() const
  {
    return "tWeightedAverage<math::tPose3D>";
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
    double accumulated_weights = 0;
    for (typename std::vector<TChannel<math::tPose3D>>::const_iterator it = channels.begin(); it != channels.end(); ++it)
    {
      accumulated_position += it->GetSample().Position() * it->GetKey();
      accumulated_roll += it->GetSample().Roll() * it->GetKey();
      accumulated_pitch += it->GetSample().Pitch() * it->GetKey();
      accumulated_yaw += it->GetSample().Yaw() * it->GetKey();
      accumulated_weights += it->GetKey();
    }
    double factor = 1.0 / accumulated_weights;
    return math::tPose3D(accumulated_position * factor, accumulated_roll * factor, accumulated_pitch * factor, accumulated_yaw * factor);
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
