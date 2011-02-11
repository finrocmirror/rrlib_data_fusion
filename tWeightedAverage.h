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
template <typename TSample>
class tWeightedAverageBase : public tDataFusion<TSample>
{

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:

  tWeightedAverageBase()
      : accumulated_weights(0)
  {}

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  virtual ~tWeightedAverageBase() = 0;

  inline TSample &AccumulatedSamples()
  {
    return this->accumulated_samples;
  }
  inline const TSample &AccumulatedSamples() const
  {
    return this->accumulated_samples;
  }

  inline double &AccumulatedWeights()
  {
    return this->accumulated_weights;
  }
  inline const double &AccumulatedWeights() const
  {
    return this->accumulated_weights;
  }

  virtual const void ClearSamplesImplementation()
  {
    this->accumulated_samples = TSample();
    this->accumulated_weights = 0;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  TSample accumulated_samples;
  double accumulated_weights;

  virtual const char *GetLogDescription() const
  {
    return "tWeightedAverage";
  }

  virtual const void ResetStateImplementation()
  {}

};

template <typename TSample>
tWeightedAverageBase<TSample>::~tWeightedAverageBase()
{}

//! Short description of tWeightedAverage
/*! A more detailed description of tWeightedAverage, which
 *  Tobias Foehst hasn't done yet !!
 */
template <typename TSample>
class tWeightedAverage : public tWeightedAverageBase<TSample>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const bool AddSampleImplementation(const TSample &sample, double key)
  {
    if (key == 0)
    {
      return false;
    }

    this->AccumulatedSamples() += sample * key;
    this->AccumulatedWeights() += key;
    return true;
  }

  virtual const TSample GetFusedValueImplementation() const
  {
    return this->AccumulatedSamples() *(1.0 / this->AccumulatedWeights());
  }

};

#ifdef _LIB_RRLIB_MATH_PRESENT_

template <>
class tWeightedAverage<rrlib::math::tPose2D> : public tWeightedAverageBase<rrlib::math::tPose2D>
{

  typedef tWeightedAverageBase<rrlib::math::tPose2D> tBase;

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:
  tWeightedAverage()
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
    this->AccumulatedSamples().SetPosition(this->AccumulatedSamples().Position() + sample.Position() * key);
    this->raw_yaw_value += sample.Yaw() * key;
    this->AccumulatedWeights() += key;
    return true;
  }

  virtual const tBase::tSample GetFusedValueImplementation() const
  {
    double factor = 1.0 / this->AccumulatedWeights();
    return tBase::tSample(this->AccumulatedSamples().Position() * factor, this->raw_yaw_value * factor);
  }

};

template <>
class tWeightedAverage<rrlib::math::tPose3D> : public tWeightedAverageBase<rrlib::math::tPose3D>
{

  typedef tWeightedAverageBase<rrlib::math::tPose3D> tBase;

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:
  tWeightedAverage()
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
    this->raw_yaw_value = 0;
    this->raw_pitch_value = 0;
    this->raw_yaw_value = 0;
  }

  virtual const bool AddSampleImplementation(const tBase::tSample &sample, double key)
  {
    this->AccumulatedSamples().SetPosition(this->AccumulatedSamples().Position() + sample.Position() * key);
    this->raw_roll_value += sample.Roll() * key;
    this->raw_pitch_value += sample.Pitch() * key;
    this->raw_yaw_value += sample.Yaw() * key;
    this->AccumulatedWeights() += key;
    return true;
  }

  virtual const tBase::tSample GetFusedValueImplementation() const
  {
    double factor = 1.0 / this->AccumulatedWeights();
    return tBase::tSample(this->AccumulatedSamples().Position() * factor, this->raw_roll_value * factor, this->raw_pitch_value * factor, this->raw_yaw_value * factor);
  }

};

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
