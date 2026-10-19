// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

// Ruckig as the reference for plem.drive's SCurve (test only).
//
// stdin, one case per line:
//   dofs  (start goal vmax amax jmax) x dofs  samples  n  t_1 .. t_n
// stdout, per case:
//   case <duration> <phase> <boundaries> <samples+1> <n>
//   then <boundaries> lines "t p v a ..." at the limiting profile's segment ends
//   (zero-length segments included), then <samples+1> lines at t = T k / samples,
//   then <n> lines at the given t_i.
// <phase> is 1 when every axis got the limiting axis' timing (a straight line),
// 0 when Ruckig fell back to time synchronisation.
// Offline, rest to rest (current/target velocity and acceleration 0),
// Synchronization::Phase. A failed calculation prints "error <code>".
#include <cstdio>
#include <iostream>
#include <ruckig/ruckig.hpp>
#include <sstream>
#include <string>
#include <vector>

using ruckig::DynamicDOFs;

static void print_state(const ruckig::Trajectory<DynamicDOFs>& traj, double t, size_t dofs) {
    std::vector<double> p(dofs), v(dofs), a(dofs);
    traj.at_time(t, p, v, a);
    std::printf("%.17g", t);
    for (size_t i = 0; i < dofs; ++i)
        std::printf(" %.17g %.17g %.17g", p[i], v[i], a[i]);
    std::printf("\n");
}

int main() {
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream in(line);
        size_t dofs = 0, samples = 0;
        if (!(in >> dofs))
            continue;
        ruckig::Ruckig<DynamicDOFs> otg(dofs);
        ruckig::InputParameter<DynamicDOFs> inp(dofs);
        ruckig::Trajectory<DynamicDOFs> traj(dofs);
        for (size_t i = 0; i < dofs; ++i) {
            in >> inp.current_position[i] >> inp.target_position[i] >> inp.max_velocity[i] >> inp.max_acceleration[i] >>
                inp.max_jerk[i];
            inp.current_velocity[i] = inp.current_acceleration[i] = 0.0;
            inp.target_velocity[i] = inp.target_acceleration[i] = 0.0;
        }
        size_t n = 0;
        in >> samples >> n;
        std::vector<double> extra(n);
        for (auto& t : extra)
            in >> t;
        inp.synchronization = ruckig::Synchronization::Phase;
        const auto result = otg.calculate(inp, traj);
        if (result != ruckig::Result::Working) {
            std::printf("error %d\n", static_cast<int>(result));
            continue;
        }
        const auto all_profiles = traj.get_profiles();  // by value: keep it alive
        const auto& profiles = all_profiles[0];
        size_t limiting = 0;
        for (size_t i = 0; i < dofs; ++i)
            if (profiles[i].t_sum.back() > profiles[limiting].t_sum.back())
                limiting = i;
        bool phase = true;
        for (size_t i = 0; i < dofs; ++i)
            if (inp.current_position[i] != inp.target_position[i] && profiles[i].t != profiles[limiting].t)
                phase = false;
        const double T = traj.get_duration();
        std::printf("case %.17g %d 8 %zu %zu\n", T, phase ? 1 : 0, samples + 1, n);
        print_state(traj, 0.0, dofs);
        for (double t : profiles[limiting].t_sum)
            print_state(traj, t, dofs);
        for (size_t k = 0; k <= samples; ++k)
            print_state(traj, T * k / samples, dofs);
        for (double t : extra)
            print_state(traj, t, dofs);
    }
    return 0;
}
