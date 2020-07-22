#include <ompl/multilevel/datastructures/BundleSpaceComponent.h>
#include <ompl/util/Exception.h>

ompl::multilevel::BundleSpaceComponent::BundleSpaceComponent(base::StateSpacePtr BundleSpace,
                                                            base::StateSpacePtr BaseSpace)
  : BundleSpace_(BundleSpace), BaseSpace_(BaseSpace)
{
}

void ompl::multilevel::BundleSpaceComponent::initFiberSpace()
{
    FiberSpace_ = computeFiberSpace();
}

ompl::base::StateSpacePtr ompl::multilevel::BundleSpaceComponent::getFiberSpace() const
{
    return FiberSpace_;
}

unsigned int ompl::multilevel::BundleSpaceComponent::getFiberDimension() const
{
    if (FiberSpace_)
        return FiberSpace_->getDimension();
    else
        return 0;
}

unsigned int ompl::multilevel::BundleSpaceComponent::getBaseDimension() const
{
    if (BaseSpace_)
        return BaseSpace_->getDimension();
    else
        return 0;
}

unsigned int ompl::multilevel::BundleSpaceComponent::getDimension() const
{
    return BundleSpace_->getDimension();
}

ompl::multilevel::BundleSpaceComponentType ompl::multilevel::BundleSpaceComponent::getType() const
{
    return type_;
}

void ompl::multilevel::BundleSpaceComponent::setType(BundleSpaceComponentType &type)
{
    type_ = type;
}

std::string ompl::multilevel::BundleSpaceComponent::stateTypeToString(base::StateSpacePtr space) const
{
    std::string tstr;
    int type = space->getType();
    if (type == base::STATE_SPACE_REAL_VECTOR)
    {
        int N = space->getDimension();
        tstr = "R";
        tstr += std::to_string(N);
    }
    else if (type == base::STATE_SPACE_SE2)
    {
        tstr = "SE2";
    }
    else if (type == base::STATE_SPACE_SE3)
    {
        tstr = "SE3";
    }
    else if (type == base::STATE_SPACE_SO2)
    {
        tstr = "SO2";
    }
    else if (type == base::STATE_SPACE_SO3)
    {
        tstr = "SO3";
    }
    else if (space->isCompound())
    {
        base::CompoundStateSpace *space_compound = space->as<base::CompoundStateSpace>();
        const std::vector<base::StateSpacePtr> space_decomposed = space_compound->getSubspaces();

        for (uint k = 0; k < space_decomposed.size(); k++)
        {
            base::StateSpacePtr s0 = space_decomposed.at(k);
            tstr = tstr + stateTypeToString(s0);
            if (k < space_decomposed.size() - 1)
                tstr += "x";
        }
    }
    else
    {
        throw Exception("Unknown State Space");
    }
    return tstr;
}

std::string ompl::multilevel::BundleSpaceComponent::getTypeAsString() const
{
    if (BaseSpace_)
    {
        std::string tstr = getBundleTypeAsString() + " -> " + getBaseTypeAsString();
        if (type_ == BUNDLE_SPACE_CONSTRAINED_RELAXATION)
        {
            tstr += " (relaxation)";
        }
        else if (type_ == BUNDLE_SPACE_IDENTITY_PROJECTION)
        {
            tstr += " (identity)";
        }
        return tstr;
    }
    else
    {
        return getBundleTypeAsString();
    }
}

std::string ompl::multilevel::BundleSpaceComponent::getFiberTypeAsString() const
{
    if (FiberSpace_)
        return stateTypeToString(FiberSpace_);
    else
        return "None";
}

std::string ompl::multilevel::BundleSpaceComponent::getBaseTypeAsString() const
{
    if (BaseSpace_)
        return stateTypeToString(BaseSpace_);
    else
        return "None";
}

std::string ompl::multilevel::BundleSpaceComponent::getBundleTypeAsString() const
{
    return stateTypeToString(BundleSpace_);
}

bool ompl::multilevel::BundleSpaceComponent::isDynamic() const
{
    return isDynamic_;
}

void ompl::multilevel::BundleSpaceComponent::print(std::ostream &out) const
{
    out << getTypeAsString() << std::endl;
}

namespace ompl
{
    namespace multilevel
    {
        std::ostream &operator<<(std::ostream &out, const BundleSpaceComponent &bundleSpaceComponent)
        {
            bundleSpaceComponent.print(out);
            return out;
        }
    }
}
