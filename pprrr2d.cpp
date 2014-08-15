/* Trials with CHOMP.
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
   \file pprrr2d.cpp
   
   Interactive trials with CHOMP for 3-DOF planar arms mounted on a
   2-DOF planar vehicle moving holonomously in the plane.
   
   Otherwise pretty much the same as pp2d.cpp, except that we actually
   sum up the obstacle cost (and gradient) for a selected few body
   points instead of just the center of the base. There is a fixed
   start and goal configuration, and you can drag a circular obstacle
   around to see how the CHOMP algorithm reacts to that.  Some of the
   computations involve guesswork, for instance how best to compute
   velocities, so a simple first-order scheme has been used.  This
   appears to produce some unwanted drift of waypoints from the start
   configuration to the end configuration.  Parameters could also be
   tuned a bit better.  Other than that, it works pretty nicely.
*/

#include "gfx.hpp"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <stdlib.h>
#include <sys/time.h>
#include <err.h>

typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::Isometry3d Transform;

using namespace std;


//////////////////////////////////////////////////
// trajectory etc

Vector xi;			// the trajectory (q_1, q_2, ...q_n)
Vector qs;			// the start config a.k.a. q_0
Vector qe;			// the end config a.k.a. q_(n+1)
static size_t const nq (20);	// number of q stacked into xi
static size_t const cdim (5);	// dimension of config space
static size_t const xidim (nq * cdim); // dimension of trajectory, xidim = nq * cdim
static double const dt (1.0);	       // time step
static double const eta (100.0); // >= 1, regularization factor for gradient descent
static double const lambda (1.0); // weight of smoothness objective

//////////////////////////////////////////////////
// gradient descent etc

Matrix AA;			// metric
Vector bb;			// acceleration bias for start and end config
Matrix Ainv;			// inverse of AA

//////////////////////////////////////////////////
// gui stuff

enum { PAUSE, STEP, RUN } state;

struct handle_s {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  handle_s (double radius, double red, double green, double blue, double alpha)
    : point_(2),
      radius_(radius),
      red_(red),
      green_(green),
      blue_(blue),
      alpha_(alpha)
  {
  }
  
  Vector point_;
  double radius_, red_, green_, blue_, alpha_;
};

static handle_s repulsor (1.5, 0.0, 0.0, 1.0, 0.2);

static handle_s * handle[] = { &repulsor, 0 };
static handle_s * grabbed (0);
static Vector grab_offset (3);


//////////////////////////////////////////////////
// robot (one per waypoint)

class Robot
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Robot ()
    : pos_a_ (3),
      pos_b_ (3),
      pos_c_ (3)
  {
  }
  
  
  Transform frame (size_t node) const
  {
    Transform tf (Transform::Identity());
    switch (node) {
    case 0:
      tf.translation() << position_[0], position_[1], 0.0;
      break;
    case 1:
      tf.translation() << position_[0], position_[1], 0.0;
      tf.linear() << c2_, -s2_, 0.0, s2_, c2_, 0.0, 0.0, 0.0, 1.0;
      break;
    case 2:
      tf.translation() << pos_a_[0], pos_a_[1], 0.0;
      tf.linear() << c23_, -s23_, 0.0, s23_, c23_, 0.0, 0.0, 0.0, 1.0;
      break;
    case 3:
      tf.translation() << pos_b_[0], pos_b_[1], 0.0;
      tf.linear() << c234_, -s234_, 0.0, s234_, c234_, 0.0, 0.0, 0.0, 1.0;
      break;
    default:
      errx (EXIT_FAILURE, "Robot::frame() called on invalid node %zu", node);
    }
    return tf;
  }
  
  
  Matrix computeJxo (size_t node, Vector const & gpoint) const
  {
    Matrix Jxo (Matrix::Zero (6, 5));
    switch (node) {
    case 3:
      Jxo (0, 4) = pos_b_[1] - gpoint[1];
      Jxo (1, 4) = gpoint[0] - pos_b_[0];
      Jxo (5, 4) = 1.0;
    case 2:
      Jxo (0, 3) = pos_a_[1] - gpoint[1];
      Jxo (1, 3) = gpoint[0] - pos_a_[0];
      Jxo (5, 3) = 1.0;
    case 1:
      Jxo (0, 2) = position_[1] - gpoint[1];
      Jxo (1, 2) = gpoint[0]    - position_[0];
      Jxo (5, 2) = 1.0;
    case 0:
      Jxo (0, 0) = 1.0;
      Jxo (1, 1) = 1.0;
      break;
    default:
      errx (EXIT_FAILURE, "Robot::computeJxo() called on invalid node %zu", node);
    }
    return Jxo;
  }
  
  
  void update (Vector const & position)
  {
    if (position.size() != 5) {
      errx (EXIT_FAILURE, "Robot::update(): position has %zu DOF (but needs 5)",
	    (size_t) position.size());
    }
    position_ = position;
    
    c2_ = cos (position_[2]);
    s2_ = sin (position_[2]);
    ac2_ = len_a_ * c2_;
    as2_ = len_a_ * s2_;
    
    q23_ = position_[2] + position_[3];
    c23_ = cos (q23_);
    s23_ = sin (q23_);
    bc23_ = len_b_ * c23_;
    bs23_ = len_b_ * s23_;
    
    q234_ = q23_ + position_[4];
    c234_ = cos (q234_);
    s234_ = sin (q234_);
    cc234_ = len_c_ * c234_;
    cs234_ = len_c_ * s234_;
    
    pos_a_ <<
      position_[0] + ac2_,
      position_[1] + as2_,
      0.0;
    pos_b_ <<
      pos_a_[0] + bc23_,
      pos_a_[1] + bs23_,
      0.0;
    pos_c_ <<
      pos_b_[0] + cc234_,
      pos_b_[1] + cs234_,
      0.0;
  }
  
  
  void draw () const
  {
    // translucent disk for base
    gfx::set_pen (1.0, 0.7, 0.7, 0.7, 0.5);
    gfx::fill_arc (position_[0], position_[1], radius_, 0.0, 2.0 * M_PI);
    
    // thick circle outline for base
    gfx::set_pen (3.0, 0.2, 0.2, 0.2, 1.0);
    gfx::draw_arc (position_[0], position_[1], radius_, 0.0, 2.0 * M_PI);
    
    // thick line for arms
    gfx::set_pen (3.0, 0.2, 0.2, 0.2, 1.0);
    gfx::draw_line (position_[0], position_[1], pos_a_[0], pos_a_[1]);
    gfx::draw_line (pos_a_[0], pos_a_[1], pos_b_[0], pos_b_[1]);
    gfx::draw_line (pos_b_[0], pos_b_[1], pos_c_[0], pos_c_[1]);
  }
  
  static double const radius_;
  static double const len_a_;
  static double const len_b_;
  static double const len_c_;
  
  Vector position_;
  Vector pos_a_;
  Vector pos_b_;
  Vector pos_c_;
  
  double c2_;
  double s2_;
  double c23_;
  double s23_;
  double c234_;
  double s234_;
  double q23_;
  double q234_;
  double ac2_;
  double as2_;
  double bc23_;
  double bs23_;
  double cc234_;
  double cs234_;
};

double const Robot::radius_ (0.5);
double const Robot::len_a_ (0.8);
double const Robot::len_b_ (0.6);
double const Robot::len_c_ (0.3);

Robot rstart;
Robot rend;
vector <Robot> robots;


//////////////////////////////////////////////////

static void update_robots ()
{
  rstart.update (qs);
  rend.update (qe);
  if (nq != robots.size()) {
    robots.resize (nq);
  }
  for (size_t ii (0); ii < nq; ++ii) {
    robots[ii].update (xi.block (ii * cdim, 0, cdim, 1));
  }
}


static void init_chomp ()
{
  qs.resize (5);
  qs << -5.0, -5.0, M_PI/2, M_PI/2, -M_PI/2;
  xi = Vector::Zero (xidim);
  qe.resize (5);
  qe << 7.0, 7.0, -M_PI/2, -M_PI/2, M_PI/2;
  
  repulsor.point_ << 3.0, 0.0;
  
  // cout << "qs\n" << qs
  //      << "\nxi\n" << xi
  //      << "\nqe\n" << qe << "\n\n";
  
  AA = Matrix::Zero (xidim, xidim);
  for (size_t ii(0); ii < nq; ++ii) {
    AA.block (cdim * ii, cdim * ii, cdim , cdim) = 2.0 * Matrix::Identity (cdim, cdim);
    if (ii > 0) {
      AA.block (cdim * (ii-1), cdim * ii, cdim , cdim) = -1.0 * Matrix::Identity (cdim, cdim);
      AA.block (cdim * ii, cdim * (ii-1), cdim , cdim) = -1.0 * Matrix::Identity (cdim, cdim);
    }
  }
  AA /= dt * dt * (nq + 1);
  
  bb = Vector::Zero (xidim);
  bb.block (0,            0, cdim, 1) = qs;
  bb.block (xidim - cdim, 0, cdim, 1) = qe;
  bb /= - dt * dt * (nq + 1);
  
  // not needed anyhow
  // double cc (double (qs.transpose() * qs) + double (qe.transpose() * qe));
  // cc /= dt * dt * (nq + 1);
  
  Ainv = AA.inverse();
  
  // cout << "AA\n" << AA
  //      << "\nAinv\n" << Ainv
  //      << "\nbb\n" << bb << "\n\n";
}


static void cb_step ()
{
  state = STEP;
}


static void cb_run ()
{
  if (RUN == state) {
    state = PAUSE;
  }
  else {
    state = RUN;
  }
}


static void cb_jumble ()
{
  for (size_t ii (0); ii < xidim; ++ii) {
    if (ii % 5 < 2) {
      xi[ii] = double (rand()) / (0.1 * numeric_limits<int>::max()) - 5.0;
    }
    else {
      xi[ii] = double (rand()) / (numeric_limits<int>::max() / 2.0 / M_PI) - M_PI;
    }
  }
  update_robots();
}


static void cb_idle ()
{
  if (PAUSE == state) {
    return;
  }
  if (STEP == state) {
    state = PAUSE;
  }
  
  //////////////////////////////////////////////////
  // beginning of "the" CHOMP iteration
  
  Vector nabla_smooth (AA * xi + bb);
  Vector const & xidd (nabla_smooth); // indeed, it is the same in this formulation...
  
  Vector nabla_obs (Vector::Zero (xidim));
  for (size_t iq (0); iq < nq; ++iq) {
    Vector qd;
    if (iq == nq - 1) {
      qd = qe - xi.block (iq * cdim, 0, cdim, 1);
    }
    else {
      qd = xi.block ((iq+1) * cdim, 0, cdim, 1) - xi.block (iq * cdim, 0, cdim, 1);
    }
    for (size_t ib (0); ib < 4; ++ib) { // later: configurable number of body points
      Vector const xx (robots[iq].frame(ib).translation());
      Matrix const JJ (robots[iq].computeJxo (ib, xx) .block (0, 0, 2, 5)); // XXXX hardcoded indices
      Vector const xd (JJ * qd);
      double const vel (xd.norm());
      if (vel < 1.0e-3) {	// avoid div by zero further down
	continue;
      }
      Vector const xdn (xd / vel);
      Vector const xdd (JJ * xidd.block (iq * cdim, 0, cdim , 1));
      Matrix const prj (Matrix::Identity (2, 2) - xdn * xdn.transpose()); // hardcoded planar case
      Vector const kappa (prj * xdd / pow (vel, 2.0));
      Vector delta (xx.block (0, 0, 2, 1) - repulsor.point_);
      double const dist (delta.norm());
      static double const maxdist (4.0);
      if ((dist >= maxdist) || (dist < 1e-9)) {
	continue;
      }
      static double const gain (10.0);
      double const cost (gain * maxdist * pow (1.0 - dist / maxdist, 3.0) / 3.0);
      delta *= - gain * pow (1.0 - dist / maxdist, 2.0) / dist;
      nabla_obs.block (iq * cdim, 0, cdim, 1) += JJ.transpose() * vel * (prj * delta - cost * kappa);
    }
  }
  
  Vector dxi (Ainv * (nabla_obs + lambda * nabla_smooth));
  xi -= dxi / eta;
  
  // end of "the" CHOMP iteration
  //////////////////////////////////////////////////
  
  update_robots ();
}


static void cb_draw ()
{
  //////////////////////////////////////////////////
  // set bounds
  
  Vector bmin (qs);
  Vector bmax (qs);
  for (size_t ii (0); ii < 2; ++ii) {
    if (qe[ii] < bmin[ii]) {
      bmin[ii] = qe[ii];
    }
    if (qe[ii] > bmax[ii]) {
      bmax[ii] = qe[ii];
    }
    for (size_t jj (0); jj < nq; ++jj) {
      if (xi[ii + cdim * jj] < bmin[ii]) {
	bmin[ii] = xi[ii + cdim * jj];
      }
      if (xi[ii + cdim * jj] > bmax[ii]) {
	bmax[ii] = xi[ii + cdim * jj];
      }
    }
  }
  
  gfx::set_view (bmin[0] - 2.0, bmin[1] - 2.0, bmax[0] + 2.0, bmax[1] + 2.0);
  
  //////////////////////////////////////////////////
  // robots
  
  rstart.draw();
  for (size_t ii (0); ii < robots.size(); ++ii) {
    robots[ii].draw();
  }
  rend.draw();
  
  //////////////////////////////////////////////////
  // trj
  
  gfx::set_pen (1.0, 0.2, 0.2, 0.2, 1.0);
  gfx::draw_line (qs[0], qs[1], xi[0], xi[1]);
  for (size_t ii (1); ii < nq; ++ii) {
    gfx::draw_line (xi[(ii-1) * cdim], xi[(ii-1) * cdim + 1], xi[ii * cdim], xi[ii * cdim + 1]);
  }
  gfx::draw_line (xi[(nq-1) * cdim], xi[(nq-1) * cdim + 1], qe[0], qe[1]);
  
  gfx::set_pen (5.0, 0.8, 0.2, 0.2, 1.0);
  gfx::draw_point (qs[0], qs[1]);
  gfx::set_pen (5.0, 0.5, 0.5, 0.5, 1.0);
  for (size_t ii (0); ii < nq; ++ii) {
    gfx::draw_point (xi[ii * cdim], xi[ii * cdim + 1]);
  }
  gfx::set_pen (5.0, 0.2, 0.8, 0.2, 1.0);
  gfx::draw_point (qe[0], qe[1]);
  
  //////////////////////////////////////////////////
  // handles
  
  for (handle_s ** hh (handle); *hh != 0; ++hh) {
    gfx::set_pen (1.0, (*hh)->red_, (*hh)->green_, (*hh)->blue_, (*hh)->alpha_);
    gfx::fill_arc ((*hh)->point_[0], (*hh)->point_[1], (*hh)->radius_, 0.0, 2.0 * M_PI);
  }
}


static void cb_mouse (double px, double py, int flags)
{
  if (flags & gfx::MOUSE_PRESS) {
    for (handle_s ** hh (handle); *hh != 0; ++hh) {
      Vector offset ((*hh)->point_);
      offset[0] -= px;
      offset[1] -= py;
      if (offset.norm() <= (*hh)->radius_) {
    	grab_offset = offset;
    	grabbed = *hh;
    	break;
      }
    }
  }
  else if (flags & gfx::MOUSE_DRAG) {
    if (0 != grabbed) {
      grabbed->point_[0] = px;
      grabbed->point_[1] = py;
      grabbed->point_ += grab_offset;
    }
  }
  else if (flags & gfx::MOUSE_RELEASE) {
    grabbed = 0;
  }
}


int main()
{
  struct timeval tt;
  gettimeofday (&tt, NULL);
  srand (tt.tv_usec);
  
  init_chomp();
  update_robots();  
  state = PAUSE;
  
  gfx::add_button ("jumble", cb_jumble);
  gfx::add_button ("step", cb_step);
  gfx::add_button ("run", cb_run);
  gfx::main ("chomp", cb_idle, cb_draw, cb_mouse);
}
