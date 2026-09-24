// Re-implementation of Yahboom closed-source kinematics using Orocos-KDL.
// Provides the same API as dofbot_kinematics.h so kinemarics_dofbot.cpp
// links without /usr/lib/libkin_srv.so or libdofbot_kinematics.so.
//
// Chain: base_link -> Gripping_point_Link (5 movable joints arm1..arm5).
// Joint convention matches kinemarics_dofbot.cpp:
//   kdl_q[i] = (servo_deg[i] - 90) * DE2RA, i.e. 0 rad == 90 deg servo.
// FK in:  5 joint values [rad, 0-centered]. Out: XYZ [m] + RPY [rad].
// IK in:  XYZ [m] + RPY [rad]. Out: 5 joint values [rad, 0-centered].
#include "dofbot_kinematics.h"

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <cmath>
#include <random>

namespace {

const char *kRootLink = "base_link";
const char *kTipLink = "Gripping_point_Link";
constexpr double kJointMin = -1.57;
constexpr double kJointMax = 1.57;

bool buildChain(const std::string &urdf_file, KDL::Chain &chain) {
  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdf_file, tree)) {
    std::cerr << "[dofbot_kinematics] cannot parse URDF: " << urdf_file << std::endl;
    return false;
  }
  if (!tree.getChain(kRootLink, kTipLink, chain)) {
    std::cerr << "[dofbot_kinematics] cannot get chain " << kRootLink << " -> " << kTipLink
              << std::endl;
    return false;
  }
  if (chain.getNrOfJoints() < 5) {
    std::cerr << "[dofbot_kinematics] unexpected nr of joints: " << chain.getNrOfJoints()
              << std::endl;
    return false;
  }
  return true;
}

}  // namespace

namespace dofbot_kinematics {

bool getFK(const std::string &urdf_file, std::vector<double> &joints,
           std::vector<double> &currentPos) {
  KDL::Chain chain;
  if (!buildChain(urdf_file, chain)) return false;
  const unsigned nj = chain.getNrOfJoints();
  if (joints.size() < nj) {
    std::cerr << "[dofbot_kinematics] FK needs " << nj << " joints, got " << joints.size()
              << std::endl;
    return false;
  }
  KDL::JntArray q(nj);
  for (unsigned i = 0; i < nj; ++i) q(i) = joints[i];
  KDL::ChainFkSolverPos_recursive fk(chain);
  KDL::Frame tip;
  if (fk.JntToCart(q, tip) < 0) {
    std::cerr << "[dofbot_kinematics] FK solve failed" << std::endl;
    return false;
  }
  double r, p, y;
  tip.M.GetRPY(r, p, y);
  currentPos.resize(6);
  currentPos[0] = tip.p.x();
  currentPos[1] = tip.p.y();
  currentPos[2] = tip.p.z();
  currentPos[3] = r;
  currentPos[4] = p;
  currentPos[5] = y;
  return true;
}

bool getIK(const std::string &urdf_file, std::vector<double> &targetXYZ,
           std::vector<double> &targetRPY, std::vector<double> &outjoints) {
  if (targetXYZ.size() < 3 || targetRPY.size() < 3) {
    std::cerr << "[dofbot_kinematics] IK needs XYZ(3)+RPY(3)" << std::endl;
    return false;
  }
  KDL::Chain chain;
  if (!buildChain(urdf_file, chain)) return false;
  const unsigned nj = chain.getNrOfJoints();

  KDL::Vector pos(targetXYZ[0], targetXYZ[1], targetXYZ[2]);
  KDL::Rotation rot = KDL::Rotation::RPY(targetRPY[0], targetRPY[1], targetRPY[2]);
  KDL::Frame target(rot, pos);

  KDL::JntArray q_min(nj), q_max(nj), q_sol(nj);
  for (unsigned i = 0; i < nj; ++i) {
    q_min(i) = kJointMin;
    q_max(i) = kJointMax;
  }
  KDL::ChainFkSolverPos_recursive fk(chain);
  KDL::ChainIkSolverVel_pinv ikvel(chain);
  KDL::ChainIkSolverPos_NR_JL ik(chain, q_min, q_max, fk, ikvel, 500, 1e-5);
  // Multi-seed restarts: NR is local and the workspace has singularities
  // (e.g. gimbal lock near pitch -90deg). Bent working pose first so the
  // arm stays in the table region instead of elbow-flipping; home last.
  // J1 spread covers lateral (Y) offsets across the camera view.
  const double seeds[][5] = {
      {0.0, 0.5, -0.5, 0.3, 0.0},
      {0.15, 0.5, -0.5, 0.3, -0.15},
      {-0.15, 0.5, -0.5, 0.3, 0.15},
      {0.3, 0.5, -0.5, 0.3, -0.3},
      {-0.3, 0.5, -0.5, 0.3, 0.3},
      {0.5, 0.5, -0.5, 0.3, -0.5},
      {-0.5, 0.5, -0.5, 0.3, 0.5},
      {0.0, 0.9, -0.9, 0.5, 0.0},
      {0.2, 0.9, -0.9, 0.5, -0.2},
      {-0.2, 0.9, -0.9, 0.5, 0.2},
      {0.5, 0.3, -0.3, 0.2, 0.3},
      {-0.5, 0.3, -0.3, 0.2, -0.3},
      {0.0, 0.0, 0.0, 0.0, 0.0},
  };
  bool ok = false;
  for (const auto &s : seeds) {
    KDL::JntArray q_init(nj);
    for (unsigned i = 0; i < nj; ++i) q_init(i) = s[i];
    if (ik.CartToJnt(q_init, target, q_sol) >= 0) {
      ok = true;
      break;
    }
  }
  // The full-pose NR solver already found a valid bounded IK solution.
  // Previously the code ignored this result, ran the relaxed fallback anyway,
  // and could report failure even though q_sol was valid.
  if (ok) {
    outjoints.resize(nj);
    for (unsigned i = 0; i < nj; ++i) outjoints[i] = q_sol(i);
    return true;
  }
  // Fallback: position-first IK with random restarts (orientation relaxed).
  // Single-seed pinv servoing gets trapped in joint clamps; sampling many
  // starts escapes local minima. Accept if position < 3mm and pitch within
  // tolerance. Verified by FK. Deterministic seed for repeatability.
  {
    KDL::ChainFkSolverPos_recursive fk2(chain);
    KDL::ChainIkSolverVel_pinv ikvel2(chain);
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> uni(kJointMin + 0.05, kJointMax - 0.05);
    double best_err = 1e9;
    double best_compatible_err = 1e9;
    bool compatible_solution = false;
    KDL::JntArray qbest(nj), qcompatible(nj);
    for (unsigned i = 0; i < nj; ++i) {
      qbest(i) = 0.0;
      qcompatible(i) = 0.0;
    }
    KDL::JntArray q(nj), qd(nj);
    for (int r = 0; r < 200; ++r) {
      for (unsigned i = 0; i < nj; ++i) q(i) = uni(rng);
      for (int k = 0; k < 40; ++k) {
        KDL::Frame cur;
        if (fk2.JntToCart(q, cur) < 0) break;
        KDL::Vector pe = target.p - cur.p;
        if (pe.Norm() < 0.002) break;
        KDL::Vector pv = pe;
        if (pv.Norm() > 0.05) pv = pv * (0.05 / pv.Norm());
        KDL::Twist twist(pv, KDL::Vector(0.0, 0.0, 0.0));
        if (ikvel2.CartToJnt(q, twist, qd) < 0) break;
        for (unsigned i = 0; i < nj; ++i) {
          q(i) += qd(i) * 0.5;
          if (q(i) < kJointMin) q(i) = kJointMin;
          if (q(i) > kJointMax) q(i) = kJointMax;
        }
      }
      KDL::Frame cur;
      if (fk2.JntToCart(q, cur) < 0) continue;
      double err = (target.p - cur.p).Norm();
      if (err < best_err) {
        best_err = err;
        qbest = q;
      }
      double cr, cp, cy;
      cur.M.GetRPY(cr, cp, cy);
      double pitch_err = targetRPY[1] - cp;
      while (pitch_err > M_PI) pitch_err -= 2 * M_PI;
      while (pitch_err < -M_PI) pitch_err += 2 * M_PI;
      const double abs_pitch_err = std::fabs(pitch_err);
      if (err < 0.003 && abs_pitch_err < 0.35 && err < best_compatible_err) {
        best_compatible_err = err;
        qcompatible = q;
        compatible_solution = true;
        // Stop only after both position and pitch constraints are satisfied.
        if (err < 0.002) break;
      }
    }
    if (compatible_solution) {
      outjoints.resize(nj);
      for (unsigned i = 0; i < nj; ++i) outjoints[i] = qcompatible(i);
      return true;
    }
    KDL::Frame best_frame;
    double best_pitch_error = 1e9;
    if (fk2.JntToCart(qbest, best_frame) >= 0) {
      double br, bp, by;
      best_frame.M.GetRPY(br, bp, by);
      double pe = targetRPY[1] - bp;
      while (pe > M_PI) pe -= 2 * M_PI;
      while (pe < -M_PI) pe += 2 * M_PI;
      best_pitch_error = std::fabs(pe);
    }
    std::cerr << "[dofbot_kinematics] position-only fallback rejected: best_position_error="
              << best_err << " m, pitch_error=" << best_pitch_error
              << " rad (limit 0.35)" << std::endl;
  }
  std::cerr << "[dofbot_kinematics] IK failed (all seeds) target=(" << targetXYZ[0] << ","
            << targetXYZ[1] << "," << targetXYZ[2] << ")" << std::endl;
  return false;
  outjoints.resize(nj);
  for (unsigned i = 0; i < nj; ++i) outjoints[i] = q_sol(i);
  return true;
}

}  // namespace dofbot_kinematics

void DOFBOT_Pro::readme(void) {
  std::cout << "DOFBOT KDL kinematics (open re-implementation). Chain: base_link -> "
               "Gripping_point_Link, LMA-IK limits +-1.57 rad."
            << std::endl;
}

bool DOFBOT_Pro::dofbot_getFK(const char *urdf_file, std::vector<double> &joints,
                              std::vector<double> &currentPos) {
  std::string f = urdf_file ? urdf_file : "";
  return dofbot_kinematics::getFK(f, joints, currentPos);
}

bool DOFBOT_Pro::dofbot_getIK(const char *urdf_file, std::vector<double> &targetXYZ,
                              std::vector<double> &targetRPY, std::vector<double> &outjoints) {
  std::string f = urdf_file ? urdf_file : "";
  return dofbot_kinematics::getIK(f, targetXYZ, targetRPY, outjoints);
}
