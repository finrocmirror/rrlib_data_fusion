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
/*!\file    tDataFusion.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-25
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/data_fusion/definitions.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

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
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tDataFusion destructor
//----------------------------------------------------------------------
template <
typename TSample,
template <typename> class TChannel
>
tDataFusion<TSample, TChannel>::~tDataFusion()
{}

//----------------------------------------------------------------------
// tDataFusion SetNumberOfChannels
//----------------------------------------------------------------------
template <typename TSample, template <typename> class TChannel>
void tDataFusion<TSample, TChannel>::SetNumberOfChannels(size_t number_of_channels)
{
  this->channels.resize(number_of_channels);
  this->data_changed = true;
}

//----------------------------------------------------------------------
// tDataFusion UpdateChannel
//----------------------------------------------------------------------
template <typename TSample, template <typename> class TChannel>
void tDataFusion<TSample, TChannel>::UpdateChannel(unsigned int channel, const tSample &sample, double key)
{
  if (channel >= this->channels.size())
  {
    std::stringstream stream;
    stream << "Channel " << channel << " does not exist in fusion object with " << this->channels.size() << " channel" << (this->channels.size() == 1 ? "" : "s") << "!";
    throw std::runtime_error(stream.str());
  }
  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_2, "Updating channel ", channel, " with sample ", sample, " and key ", key);
  this->channels[channel].AddSample(sample, key);
  this->data_changed = true;
}

//----------------------------------------------------------------------
// tDataFusion UpdateAllChannels
//----------------------------------------------------------------------
template <typename TSample, template <typename> class TChannel>
template <typename TSampleIterator>
void tDataFusion<TSample, TChannel>::UpdateAllChannels(TSampleIterator begin_samples, TSampleIterator end_samples)
{
  size_t channel = 0;
  TSampleIterator sample = begin_samples;
  while (sample != end_samples)
  {
    this->UpdateChannel(channel++, *(sample++));
  }
}

template <typename TSample, template <typename> class TChannel>
template <typename TSampleIterator, typename TKeyIterator>
void tDataFusion<TSample, TChannel>::UpdateAllChannels(TSampleIterator begin_samples, TSampleIterator end_samples, TKeyIterator begin_keys, TKeyIterator end_keys)
{
  size_t channel = 0;
  TSampleIterator sample = begin_samples;
  TKeyIterator key = begin_keys;
  while (sample != end_samples && key != end_keys)
  {
    this->UpdateChannel(channel++, *(sample++), *(key++));
  }

  if (sample != end_samples || key != end_keys)
  {
    throw std::runtime_error("Number of samples did not match number of keys!");
  }
}

//----------------------------------------------------------------------
// tDataFusion IsValid
//----------------------------------------------------------------------
template <typename TSample, template <typename> class TChannel>
const bool tDataFusion<TSample, TChannel>::IsValid() const
{
  if (this->channels.empty())
  {
    throw std::logic_error("Number of channels must be greater than zero!");
  }
  for (typename std::vector<TChannel<TSample>>::const_iterator it = this->channels.begin(); it != this->channels.end(); ++it)
  {
    if (!it->IsValid())
    {
      return false;
    }
  }
  return this->HasValidState();
}

//----------------------------------------------------------------------
// tDataFusion ClearChannels
//----------------------------------------------------------------------
template <typename TSample, template <typename> class TChannel>
void tDataFusion<TSample, TChannel>::ClearChannels()
{
  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_1, "Clearing channels.");
  for (typename std::vector<TChannel<TSample>>::iterator it = this->channels.begin(); it != this->channels.end(); ++it)
  {
    it->Clear();
  }
}

//----------------------------------------------------------------------
// tDataFusion ResetState
//----------------------------------------------------------------------
template <typename TSample, template <typename> class TChannel>
void tDataFusion<TSample, TChannel>::ResetState()
{
  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_1, "Resetting state.");
  this->ClearChannels();
  this->ResetStateImplementation();
}

//----------------------------------------------------------------------
// tDataFusion EnterNextTimestep()
//----------------------------------------------------------------------
template <typename TSample, template <typename> class TChannel>
void tDataFusion<TSample, TChannel>::EnterNextTimestep()
{
  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_1, "Clearing channels.");
  for (typename std::vector<TChannel<TSample>>::iterator it = this->channels.begin(); it != this->channels.end(); ++it)
  {
    it->PrepareForNextTimestep();
  }
  this->EnterNextTimestepImplementation();
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
