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
template <typename TSample>
class tAverageBase : public tDataFusion<TSample>
{

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  virtual ~tAverageBase() = 0;

  inline TSample &AccumulatedSamples()
  {
    return this->accumulated_samples;
  }
  inline const TSample &AccumulatedSamples() const
  {
    return this->accumulated_samples;
  }

  virtual const void ClearSamplesImplementation()
  {
    this->accumulated_samples = TSample();
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  TSample accumulated_samples;

  virtual const char *GetLogDescription() const
  {
    return "tAverage";
  }

  virtual const void ResetStateImplementation()
  {}

};

template <typename TSample>
tAverageBase<TSample>::~tAverageBase()
{}

//! Short description of tAverage
/*! A more detailed description of tAverage, which
 *  Tobias Foehst hasn't done yet !!
 */
template <typename TSample>
class tAverage : public tAverageBase<TSample>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const bool AddSampleImplementation(const TSample &sample, double key)
  {
    this->AccumulatedSamples() += sample;
    return true;
  }

  virtual const TSample GetFusedValueImplementation() const
  {
    return this->AccumulatedSamples() *(1.0 / this->NumberOfSamples());
  }
};

#ifdef _LIB_RRLIB_MATH_PRESENT_

template <>
class tAverage<rrlib::math::tPose2D> : public tAverageBase<rrlib::math::tPose2D>
{

  typedef tAverageBase<rrlib::math::tPose2D> tBase;

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:
  tAverage()
      : raw_yaw_value(0)
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  double raw_yaw_value;

  virtual const void ClearSamplesImplementation()
  {
    tBase::ClearSamplesImplementation();
    this->raw_yaw_value = 0;
  }

  virtual const bool AddSampleImplementation(const tBase::tSample &sample, double key)
  {
    this->AccumulatedSamples() += sample;
    this->raw_yaw_value += sample.Yaw();
    return true;
  }

  virtual const tBase::tSample GetFusedValueImplementation() const
  {
    double factor = 1.0 / this->NumberOfSamples();
    return tBase::tSample(this->AccumulatedSamples().GetPosition() * factor, this->raw_yaw_value * factor);
  }

};

template <>
class tAverage<rrlib::math::tPose3D> : public tAverageBase<rrlib::math::tPose3D>
{

  typedef tAverageBase<rrlib::math::tPose3D> tBase;

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:
  tAverage()
      : raw_roll_value(0),
      raw_pitch_value(0),
      raw_yaw_value(0)
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  double raw_roll_value;
  double raw_pitch_value;
  double raw_yaw_value;

  virtual const void ClearSamplesImplementation()
  {
    tBase::ClearSamplesImplementation();
    this->raw_roll_value = 0;
    this->raw_pitch_value = 0;
    this->raw_yaw_value = 0;
  }

  virtual const bool AddSampleImplementation(const tBase::tSample &sample, double key)
  {
    this->AccumulatedSamples() += sample;
    this->raw_roll_value += sample.Roll();
    this->raw_pitch_value += sample.Pitch();
    this->raw_yaw_value += sample.Yaw();
    return true;
  }

  virtual const tBase::tSample GetFusedValueImplementation() const
  {
    double factor = 1.0 / this->NumberOfSamples();
    return tBase::tSample(this->AccumulatedSamples().GetPosition() * factor, this->raw_roll_value * factor, this->raw_pitch_value * factor, this->raw_yaw_value * factor);
  }

};

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
