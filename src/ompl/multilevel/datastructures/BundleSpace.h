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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_

#include <ompl/base/Planner.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/multilevel/datastructures/ProjectionComponentFactory.h>

namespace ompl
{
    /** \brief This namespace contains datastructures and planners to
         exploit multilevel abstractions, which you have to specify using
         a sequence of SpaceInformationPtr (each with their own StateValidityChecker). */
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(BundleSpaceComponent);
        OMPL_CLASS_FORWARD(BundleSpaceMetric);
        OMPL_CLASS_FORWARD(Projection);
        OMPL_CLASS_FORWARD(BundleSpacePropagator);

        /// \brief A single Bundle-space
        class BundleSpace : public ompl::base::Planner
        {
        private:
            using BaseT = ompl::base::Planner;
            using BaseT::si_;  // make it private.
            using BaseT::getSpaceInformation;

            // Note: use getBundle(), or getBase() 
            // to access the SpaceInformationPtr

            /// \brief solve is disabled (use BundleSequence::solve)
            ompl::base::PlannerStatus solve(
                const ompl::base::PlannerTerminationCondition &ptc) override final;

        public:
            /**  \brief Bundle Space contains three OMPL spaces, 
             * which we call Bundle, Base and Fiber.

                 - Bundle is (locally) a product space of Base and Fiber
                 - Base is a pointer to the next lower-dimensional Bundle-space (if any)
                 - Fiber is the quotient space Bundle / Base

                 We assume that Bundle and Base have been given 
                 (as ompl::base::SpaceInformationPtr),
                 and we automatically compute the fiber */

            BundleSpace(const ompl::base::SpaceInformationPtr &si, 
                BundleSpace *baseSpace_ = nullptr);

            virtual ~BundleSpace();

            /// \brief Get SpaceInformationPtr for Bundle
            const ompl::base::SpaceInformationPtr &getBundle() const;
            /// \brief Get SpaceInformationPtr for Base
            const ompl::base::SpaceInformationPtr &getBase() const;
            /// \brief Get ProjectionPtr from Bundle to Base
            ProjectionPtr getProjection() const;

            bool makeProjection();

            virtual void setProblemDefinition(
                const ompl::base::ProblemDefinitionPtr &pdef) override;

            virtual void grow() = 0;
            virtual bool getSolution(ompl::base::PathPtr &solution) = 0;
            virtual void setMetric(const std::string &sMetric) = 0;
            virtual void setPropagator(const std::string &sPropagator) = 0;

            virtual void sampleFromDatastructure(ompl::base::State *xBase) = 0;
            virtual void sampleBundle(ompl::base::State *xRandom);
            bool sampleBundleValid(ompl::base::State *xRandom);

            virtual bool hasSolution();
            virtual bool isInfeasible();
            virtual bool hasConverged();

            virtual void clear() override;
            virtual void setup() override;

            virtual double getImportance() const = 0;

            /// \brief Allocate State, set entries to Identity/Zero
            ompl::base::State *allocIdentityStateBundle() const;
            ompl::base::State *allocIdentityStateBase() const;
            ompl::base::State *allocIdentityState(ompl::base::StateSpacePtr) const;
            void allocIdentityState(ompl::base::State *, ompl::base::StateSpacePtr) const;

            /// \brief Print Information pertaining to why a state failed being
            /// valid
            // void debugInvalidState(const ompl::base::State *);

            /// \brief reset counter for number of levels
            static void resetCounter();

            /// \brief Dimension of Base Space
            unsigned int getBaseDimension() const;
            /// \brief Dimension of Bundle Space
            unsigned int getBundleDimension() const;

            const ompl::base::StateSamplerPtr &getBundleSamplerPtr() const;
            const ompl::base::StateSamplerPtr &getBaseSamplerPtr() const;

            /// \brief Return k-1 th bundle space (locally the base space)
            BundleSpace *getBaseSpace() const;

            /// \brief Pointer to k-1 th bundle space (locally the base space)
            void setBaseSpace(BundleSpace *baseBundleSpace);

            /// \brief Return k+1 th bundle space (locally the total space)
            BundleSpace *getParentSpace() const;

            /// \brief Pointer to k+1 th bundle space (locally the total space)
            void setParentSpace(BundleSpace *parentSpace);

            /// \brief Return if has base space pointer
            bool hasBaseSpace() const;

            /// \brief Return if has parent space pointer
            bool hasParentSpace() const;

            /// Level in hierarchy of Bundle-spaces
            unsigned int getLevel() const;

            /// Change level in hierarchy
            void setLevel(unsigned int);

            /// \brief Bundle Space Projection Operator onto first component
            /// ProjectBase: Bundle \rightarrow Base
            void projectBase(const ompl::base::State *xBundle, 
                ompl::base::State *xBase) const;

            /// \brief Lift a state from Base to Bundle
            void liftState(const ompl::base::State *xBase, 
                ompl::base::State *xBundle) const;

            ompl::base::OptimizationObjectivePtr getOptimizationObjectivePtr() const;

            /// \brief Write class to stream (use as std::cout << *this << std::endl)
            ///  Actual implementation is in void print(std::ostream& out),
            ///  which can be inherited.
            friend std::ostream &operator<<(std::ostream &, const BundleSpace &);

            bool isDynamic() const;

            base::GoalSampleableRegion* getGoalPtr() const;

        private:

            /// Level in sequence of Bundle-spaces
            unsigned int level_{0};

            //\brief Being on the k-th bundle space, we denote as baseSpace the k-1-th
            // bundle space (because it locally acts as the base space for the current class)
            BundleSpace *baseBundleSpace_{nullptr};

            //\brief Being on the k-th bundle space, we denote as parentSpace the k+1-th
            // bundle space
            BundleSpace *parentBundleSpace_{nullptr};

            ompl::base::SpaceInformationPtr totalSpace_{nullptr};
            ompl::base::SpaceInformationPtr baseSpace_{nullptr};

            ompl::base::StateSamplerPtr Bundle_sampler_;
            ompl::base::ValidStateSamplerPtr Bundle_valid_sampler_;

            /**\brief Call algorithm to solve the find section problem */
            virtual bool findSection();

            /** \brief Projection Operator to project and lift between bundle
             * and base space */
            ProjectionPtr projection_;

        protected:
            /// Check if Bundle-space is bounded
            void checkBundleSpaceMeasure(std::string name, 
                const ompl::base::StateSpacePtr space) const;
            void sanityChecks() const;

            /// Internal function implementing actual printing to stream
            virtual void print(std::ostream &out) const;

            /// A temporary state on Base
            ompl::base::State *xBaseTmp_{nullptr};
            /// A temporary state on Bundle
            ompl::base::State *xBundleTmp_{nullptr};

            static unsigned int counter_;

            /// Identity of space (to keep track of number of Bundle-spaces created)
            unsigned int id_{0};

            bool hasSolution_{false};
            bool firstRun_{true};

            bool isDynamic_{false};

            /** \brief Metric on bundle space */
            BundleSpaceMetricPtr metric_;

            /** \brief Propagator (steering or interpolation) on bundle space.
             * Note: currently just a stub for base::StatePropagator*/
            BundleSpacePropagatorPtr propagator_;
        };
    }  // namespace multilevel
}  // namespace ompl
#endif
