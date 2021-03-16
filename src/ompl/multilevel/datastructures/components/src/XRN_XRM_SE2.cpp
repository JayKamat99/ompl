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

#include <ompl/multilevel/datastructures/components/XRN_XRM_SE2.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

using namespace ompl::multilevel;

ProjectionComponentWithFiber_SE2RN_SE2RM::ProjectionComponentWithFiber_SE2RN_SE2RM(ompl::base::StateSpacePtr BundleSpace,
                                                                   ompl::base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}

void ProjectionComponentWithFiber_SE2RN_SE2RM::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    const base::SE2StateSpace::StateType *xBundle_SE2 =
        xBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
    const base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    base::SE2StateSpace::StateType *xBase_SE2 = xBase->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xBase_RN =
        xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xBase_SE2->setX(xBundle_SE2->getX());
    xBase_SE2->setY(xBundle_SE2->getY());
    xBase_SE2->setYaw(xBundle_SE2->getYaw());

    for (unsigned int k = 0; k < getBaseDimension() - 3; k++)
    {
        xBase_RN->values[k] = xBundle_RN->values[k];
    }
}

void ProjectionComponentWithFiber_SE2RN_SE2RM::liftState(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                                                 ompl::base::State *xBundle) const
{
    base::SE2StateSpace::StateType *xBundle_SE2 =
        xBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const base::SE2StateSpace::StateType *xBase_SE2 =
        xBase->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
    const base::RealVectorStateSpace::StateType *xBase_RM =
        xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const base::RealVectorStateSpace::StateType *xFiber_RJ = xFiber->as<base::RealVectorStateSpace::StateType>();

    xBundle_SE2->setX(xBase_SE2->getX());
    xBundle_SE2->setY(xBase_SE2->getY());
    xBundle_SE2->setYaw(xBase_SE2->getYaw());

    //[X Y YAW] [1...M-1][M...N-1]
    // SE2               RN
    unsigned int M = getDimension() - getFiberDimension() - 3;
    unsigned int N = getFiberDimension();

    for (unsigned int k = 0; k < M; k++)
    {
        xBundle_RN->values[k] = xBase_RM->values[k];
    }
    for (unsigned int k = M; k < M + N; k++)
    {
        xBundle_RN->values[k] = xFiber_RJ->values[k - M];
    }
}
