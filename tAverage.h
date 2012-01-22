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
/*!\file    tAverage.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-25
 *
 * \brief   Contains tAverage
 *
 * \b tAverage
 *
 * A few words for tAverage
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__data_fusion__tAverage_h__
#define __rrlib__data_fusion__tAverage_h__

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
//! Short description of tAverageBase
/*! A more detailed description of tAverageBase, which
 *  Tobias Foehst hasn't done yet !!
 */
template <
typename TSample,
         template <typename> class TChannel = channel::LastValue
         >
class tAverage : public tDataFusion<TSample, TChannel>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const char *GetLogDescription() const
  {
    return "tAverage";
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
    for (typename std::vector<TChannel<TSample>>::const_iterator it = channels.begin(); it != channels.end(); ++it)
    {
      *accumulated += it->GetSample();
    }
    return *accumulated * (1.0 / channels.size());
  }

  virtual void ResetStateImplementation()
  {}

  virtual void EnterNextTimestepImplementation()
  {}

};

#ifdef _LIB_RRLIB_MATH_PRESENT_

template <template <typename> class TChannel>
class tAverage<math::tPose2D, TChannel> : public tDataFusion<math::tPose2D, TChannel>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const char *GetLogDescription() const
  {
    return "tAverage<tPose2D>";
  }

  virtual const bool HasValidState() const
  {
    return true;
  }

  virtual const math::tPose2D CalculateFusedValue(const std::vector<TChannel<math::tPose2D>> &channels)
  {
    math::tVec2d accumulated_position;
    double accumulated_yaw = 0;
    for (typename std::vector<TChannel<math::tPose2D>>::const_iterator it = channels.begin(); it != channels.end(); ++it)
    {
      accumulated_position += it->GetSample().Position();
      accumulated_yaw += it->GetSample().Yaw();
    }
    double factor = 1.0 / channels.size();
    return math::tPose2D(accumulated_position * factor, accumulated_yaw * factor);
  }

  virtual void ResetStateImplementation()
  {}

  virtual void EnterNextTimestepImplementation()
  {}

};

template <template <typename> class TChannel>
class tAverage<math::tPose3D, TChannel> : public tDataFusion<math::tPose3D, TChannel>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const char *GetLogDescription() const
  {
    return "tAverage<tPose3D>";
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
    for (typename std::vector<TChannel<math::tPose3D>>::const_iterator it = channels.begin(); it != channels.end(); ++it)
    {
      accumulated_position += it->GetSample().Position();
      accumulated_roll += it->GetSample().Roll();
      accumulated_pitch += it->GetSample().Pitch();
      accumulated_yaw += it->GetSample().Yaw();
    }
    double factor = 1.0 / channels.size();
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
