#include "abclib/path/eta3_path.hpp"
#include <Eigen/Core>
#include <vector>
#include <algorithm>
#include <cmath>

// ArcLengthPath wraps Eta3Path to provide true arc-length
// parameterization. Requires Eta3Path to expose:
//   int numSegments() const;
//   const Eta3PathSegment& getSegment(int i) const;

class ArcLengthPath {
public:
    explicit ArcLengthPath(const Eta3Path& path);

    double totalLength() const;
    Eigen::Vector3d q(double s) const;
    Eigen::Vector3d qdot(double s) const;
    Eigen::Vector3d qddot(double s) const;

private:
    const Eta3Path& path_;
    std::vector<double> cum_length_;

    // Find segment index idx and invert true arc-length within that segment
    void locate(double s, int& idx, double& u) const;
    double invertSegmentParam(int idx, double s_local) const;
};