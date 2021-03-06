#ifndef __STATE_DATA_H_INCLUDED
#define __STATE_DATA_H_INCLUDED

#include <ostream>
#include "geometry_utils.h"

struct Occupancy {
  double prob_occ;
  double estimation_quality;

  // TODO: is NaN better for as a dflt value?
  Occupancy(double prob = 0, double quality = 0) :
    prob_occ(prob), estimation_quality(quality) {}

  operator double() const { return prob_occ; }

  bool operator==(const Occupancy &that) const {
    if (!is_valid() && !that.is_valid()) { return true; }
    return are_equal(prob_occ, that.prob_occ) &&
           are_equal(estimation_quality, that.estimation_quality);
  }

  bool is_valid() const {
    return !std::isnan(prob_occ) && !std::isnan(estimation_quality);
  }

  static Occupancy invalid() {
    static Occupancy invalid{std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::quiet_NaN()};
    return invalid;
  }

};

std::ostream &operator<<(std::ostream &stream, const Occupancy &occ) {
  if (!occ.is_valid()) {
    return stream << "invalid";
  }
  return stream << "{" << occ.prob_occ << ", " << occ.estimation_quality << "}";
}

#endif
