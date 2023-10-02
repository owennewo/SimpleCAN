#pragma once

enum FilterType : int
{
    REJECT_ALL = 0,
    ACCEPT_ALL = 1,
    MASK_STANDARD = 2,
    MASK_EXTENDED = 3
};

enum FrameType : int
{
    FILTER_DATA_FRAME = 0,
    FILTER_REMOTE_FRAME = 1,
    FILTER_ANY_FRAME = 2
};

class CanFilter
{
public:
    CanFilter(FilterType type)
        : type_(type), frameType_(FILTER_ANY_FRAME), identifier_(0), mask_(0){};
    CanFilter(FilterType type, unsigned int identifier, unsigned int mask, FrameType frameType = FILTER_ANY_FRAME)
        : type_(type), frameType_(frameType), identifier_(identifier), mask_(mask){};

    FilterType getType() const
    {
        return type_;
    }
    FrameType getFrameType() const
    {
        return frameType_;
    }
    unsigned int getIdentifier() const
    {
        return identifier_;
    }
    unsigned int getMask() const
    {
        return mask_;
    }

private:
    FilterType type_;
    FrameType frameType_;
    unsigned int identifier_;
    unsigned int mask_;
};
