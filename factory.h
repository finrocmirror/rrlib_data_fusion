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
/*!\file    factory.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-28
 *
 * \brief
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__data_fusion__factory_h__
#define __rrlib__data_fusion__factory_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/design_patterns/singleton.h"
#include "rrlib/design_patterns/factory.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/data_fusion/tMaximumKey.h"
#include "rrlib/data_fusion/tAverage.h"
#include "rrlib/data_fusion/tWeightedAverage.h"
#include "rrlib/data_fusion/tWeightedSum.h"
#include "rrlib/data_fusion/tMedianVoter.h"
#include "rrlib/data_fusion/tMedianKeyVoter.h"

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

// FIXME: we need template typedefs. Then this can be replaced for simplicity
template <typename TSample>
class tDataFusionFactory : public design_patterns::tSingletonHolder<design_patterns::tFactory<tDataFusion<TSample>, std::string>>
{
  tDataFusionFactory();
};

// FIXME: and we need lambda to directly insert operator new instead of using this function (which renders the factory absurd)
template <typename TSample>
tDataFusion<TSample> *Create(int type)
{
  switch (type)
  {
  case 0:
    return new tMaximumKey<TSample>;
  case 1:
    return new tAverage<TSample>;
  case 2:
    return new tWeightedAverage<TSample>;
  case 3:
    return new tWeightedSum<TSample>;
  case 4:
    return new tMedianVoter<TSample>;
  case 5:
    return new tMedianKeyVoter<TSample>;
  default:
    return NULL;
  }
}

template <typename TSample>
inline void InitializeFactory()
{
  design_patterns::tFunctor<tDataFusion<TSample> *, int> creator(&Create<TSample>);
  tDataFusionFactory<TSample>::Instance().Register("Maximum Key", design_patterns::tFunctor<tDataFusion<TSample> *>(design_patterns::BindFirstParameter(creator, 0)));
  tDataFusionFactory<TSample>::Instance().Register("Average", design_patterns::tFunctor<tDataFusion<TSample> *>(design_patterns::BindFirstParameter(creator, 1)));
  tDataFusionFactory<TSample>::Instance().Register("Weighted Average", design_patterns::tFunctor<tDataFusion<TSample> *>(design_patterns::BindFirstParameter(creator, 2)));
  tDataFusionFactory<TSample>::Instance().Register("Weighted Sum", design_patterns::tFunctor<tDataFusion<TSample> *>(design_patterns::BindFirstParameter(creator, 3)));
  tDataFusionFactory<TSample>::Instance().Register("Median Voter", design_patterns::tFunctor<tDataFusion<TSample> *>(design_patterns::BindFirstParameter(creator, 4)));
  tDataFusionFactory<TSample>::Instance().Register("Median Key Voter", design_patterns::tFunctor<tDataFusion<TSample> *>(design_patterns::BindFirstParameter(creator, 5)));
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
