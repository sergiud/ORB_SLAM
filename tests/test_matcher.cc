#include <format>
#include <set>

#include "Check.h"
#include "ORBmatcher.h"

namespace {

using ORB_SLAM::tests::check;

constexpr int fullTurn = 360;
// Width of a bin of the rotation histogram in degrees.
constexpr float binWidth = 12;

// Exposes how the matcher sorts rotations into its histogram.
class RotationMatcher : public ORB_SLAM::ORBmatcher
{
public:
    using ORB_SLAM::ORBmatcher::HISTO_LENGTH;
    using ORB_SLAM::ORBmatcher::RotationBin;
};

// The rotations of a full turn fill every bin of the histogram.
void rotationsReachEveryBin()
{
    std::set<int> bins;

    for (int angle = 0; angle < fullTurn; ++angle) {
        bins.insert(RotationMatcher::RotationBin(static_cast<float>(angle), 0));
    }

    check(static_cast<int>(bins.size()) == RotationMatcher::HISTO_LENGTH,
          std::format("a full turn reaches {} bins, got {}",
                      RotationMatcher::HISTO_LENGTH, bins.size()));
}

// Small rotations in either direction share the bin of no rotation, so that
// a camera that barely rolls has one peak in the histogram.
void smallRotationsShareBin()
{
    constexpr float small = 5;

    const int clockwise = RotationMatcher::RotationBin(0, small);
    const int counterclockwise = RotationMatcher::RotationBin(small, 0);

    check(clockwise == 0 && counterclockwise == 0,
          std::format("rotations of -{0} and {0} degrees fall into bin 0, got "
                      "{1} and {2}",
                      small, clockwise, counterclockwise));
}

// Rotations one bin width apart fall into neighboring bins.
void rotationsByBinWidthAreNeighbors()
{
    const int counterclockwise = RotationMatcher::RotationBin(binWidth, 0);
    const int clockwise = RotationMatcher::RotationBin(0, binWidth);
    const int last = RotationMatcher::HISTO_LENGTH - 1;

    check(counterclockwise == 1,
          std::format("a rotation of {} degrees falls into bin 1, got {}",
                      binWidth, counterclockwise));
    check(clockwise == last,
          std::format("a rotation of -{} degrees falls into bin {}, got {}",
                      binWidth, last, clockwise));
}

} // namespace

int main()
{
    rotationsReachEveryBin();
    smallRotationsShareBin();
    rotationsByBinWidthAreNeighbors();

    return ORB_SLAM::tests::exitStatus();
}
