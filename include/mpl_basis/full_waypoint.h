/*
 * Copyright (c) 2019, Ramkumar Natarajan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * \file   full_waypoint.h
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   12/23/19
 */

#ifndef MPL_FULL_WAYPOINT_H
#define MPL_FULL_WAYPOINT_H

#include <bitset>
#include <boost/functional/hash.hpp>
#include <iostream>

#include <mpl_basis/waypoint.h>

// @TODO: Change the comments and documentation according to Full Waypoint class
/**
 * @brief FullWaypoint class derived from Waypoint class
 *
 * State includes position, velocity, acceleration and jerk in \f$R^n\f$, where
 * the dimension \f$n\f$ can be either 2 or 3. Yaw is contained by default. The
 * anonymous union is used to set control flag.
 */
template <int Dim>
struct FullWaypoint : Waypoint<Dim> {
  /// Empty constructor
  FullWaypoint()
  {
    this->control = Control::NONE;
  }

  /**
   * @brief Simple constructor
   * @param c control value
   */
  FullWaypoint(Control::Control c)
  {
    this->control = c;
  }

  Vecf<Dim> ang_pos{Vecf<Dim>::Zero()};  ///< RPY in \f$R^{n}\f$
  Vecf<Dim> ang_vel{Vecf<Dim>::Zero()};  ///< angular velocity in body frame
  Vecf<Dim> ang_acc{Vecf<Dim>::Zero()};  ///< anglular acceleration in body frame
  Vecf<Dim> ang_jrk{Vecf<Dim>::Zero()};  ///< angular jerk in body frame

};

/// Generate hash value for FullWaypoint class
template <int Dim>
std::size_t hash_value(const FullWaypoint<Dim>& key) {
  Waypoint<Dim> base_key = key;
  std::size_t val = hash_value(base_key);

  for (int i = 0; i < Dim; i++) {
    if (key.use_pos) {
      int id = std::round(key.ang_pos(i) / 0.01);
      boost::hash_combine(val, id);
    }
    if (key.use_vel) {
      int id = std::round(key.ang_vel(i) / 0.1);
      boost::hash_combine(val, id);
    }
    if (key.use_acc) {
      int id = std::round(key.ang_acc(i) / 0.1);
      boost::hash_combine(val, id);
    }
    if (key.use_jrk) {
      int id = std::round(key.ang_jrk(i) / 0.1);
      boost::hash_combine(val, id);
    }
  }

  return val;
}

/**
 * @brief  Check if two FullWaypoints are equivalent
 *
 * using the hash value
 */
template <int Dim>
bool operator==(const FullWaypoint<Dim>& l, const FullWaypoint<Dim>& r) {
  return hash_value(l) == hash_value(r);
}

/// Check if two FullWaypoints are not equivalent
template <int Dim>
bool operator!=(const FullWaypoint<Dim>& l, const FullWaypoint<Dim>& r) {
  return hash_value(l) != hash_value(r);
}

/// FullWaypoints for 2D
typedef FullWaypoint<2> FullWaypoint2D;

/// FullWaypoints for 3D
typedef FullWaypoint<3> FullWaypoint3D;

namespace std
{
  template <>
  struct hash<FullWaypoint3D>
  {
    std::size_t operator()(const FullWaypoint3D &waypoint) const
    {
      return hash_value(waypoint);
    }
  };
}

#endif //MPL_FULL_WAYPOINT_H
