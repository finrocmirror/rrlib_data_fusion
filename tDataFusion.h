//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
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
/*!\file    tDataFusion.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-25
 *
 * \brief   Contains tDataFusion
 *
 * \b tDataFusion
 *
 * A few words for tDataFusion
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__data_fusion__tDataFusion_h__
#define __rrlib__data_fusion__tDataFusion_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <stdexcept>
#include <vector>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/data_fusion/policies/channel/LastValue.h"

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
//! Short description of tDataFusion
/*! A more detailed description of tDataFusion, which
 *  Tobias Foehst hasn't done yet !!
 */
template <
typename TSample,
         template <typename> class TChannel = channel::LastValue
         >
class tDataFusion
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef TSample tSample;

  virtual ~tDataFusion() = 0;

  inline size_t NumberOfChannels() const
  {
    return this->channels.size();
  }

  void SetNumberOfChannels(size_t number_of_channels);

  void UpdateChannel(size_t channel, const tSample &sample, double key = 1);

  template <typename TSampleIterator>
  void UpdateAllChannels(TSampleIterator begin_samples, TSampleIterator end_samples);

  template <typename TSampleIterator, typename TKeyIterator>
  void UpdateAllChannels(TSampleIterator begin_samples, TSampleIterator end_samples, TKeyIterator begin_keys, TKeyIterator end_keys);

  inline const tSample &FusedValue()
  {
    if (!this->IsValid())
    {
      throw std::runtime_error("Fused value not available with invalid state!");
    }
    if (this->data_changed)
    {
      this->fused_value = this->CalculateFusedValue(this->channels);
      this->data_changed = false;
    }
    return this->fused_value;
  }

  const bool IsValid() const;

  void ClearChannels();

  void ResetState();

  void EnterNextTimestep();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  std::vector<TChannel<TSample>> channels;
  TSample fused_value;
  bool data_changed;

  virtual const char *GetLogDescription() const
  {
    return "tDataFusion";
  }

  virtual const bool HasValidState() const = 0;
  virtual const TSample CalculateFusedValue(const std::vector<TChannel<TSample>> &channels) = 0;
  virtual void ResetStateImplementation() = 0;
  virtual void EnterNextTimestepImplementation() = 0;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/data_fusion/tDataFusion.hpp"

#endif
