/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Andreas Orthey */

#include <ompl/multilevel/datastructures/projections/XRN_XRM.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

using namespace ompl::multilevel;

Projection_XRN_XRM::Projection_XRN_XRM(ompl::base::StateSpacePtr BundleSpace, ompl::base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}
void Projection_XRN_XRM::projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const
{
    const auto *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const auto *xFiber_RJ = xFiber->as<base::RealVectorStateSpace::StateType>();

    unsigned int Noffset = getBaseDimension() - dimensionBaseFirstSubspace;
    for (unsigned int k = 0; k < getFiberDimension(); k++)
    {
        xFiber_RJ->values[k] = xBundle_RN->values[k + Noffset];
    }
}

ompl::base::StateSpacePtr Projection_XRN_XRM::computeFiberSpace()
{
    base::CompoundStateSpace *Bundle_compound = getBundle()->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();

    base::CompoundStateSpace *Base_compound = getBase()->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

    dimensionBaseFirstSubspace = Base_decomposed.at(0)->getDimension();

    unsigned int Nbundle = Bundle_decomposed.at(1)->getDimension();
    unsigned int Nbase = Base_decomposed.at(1)->getDimension();
    unsigned int Nfiber = Nbundle - Nbase;

    base::StateSpacePtr Fiber_RN(new base::RealVectorStateSpace(Nfiber));

    const auto *Bundle_RN = Bundle_decomposed.at(1)->as<base::RealVectorStateSpace>();
    base::RealVectorBounds Bundle_bounds = Bundle_RN->getBounds();
    std::vector<double> low;
    low.resize(Nfiber);
    std::vector<double> high;
    high.resize(Nfiber);
    base::RealVectorBounds Fiber_bounds(Nfiber);
    for (unsigned int k = 0; k < Nfiber; k++)
    {
        Fiber_bounds.setLow(k, Bundle_bounds.low.at(k + Nbase));
        Fiber_bounds.setHigh(k, Bundle_bounds.high.at(k + Nbase));
    }
    std::static_pointer_cast<base::RealVectorStateSpace>(Fiber_RN)->setBounds(Fiber_bounds);

    return Fiber_RN;
}
