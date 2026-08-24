#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

// Report "every case skipped" to ctest as SKIPPED rather than PASSED.
//
// The render-backed tests self-skip when no usable context exists — GL below 4.5, or
// no QRhi backend at all. GoogleTest exits 0 in that case, so ctest records a PASS
// and the summary counts them among the passing tests. On macOS that means nine
// suites report success while running nothing, and a green "243 passed" says nothing
// whatsoever about the OpenGL renderer. That false signal is not hypothetical: it was
// mistaken for real coverage repeatedly during the QRhi port.
//
// Pairing this with SKIP_RETURN_CODE 77 in CMake turns the lie into a number —
// ctest then prints "Skipped" per suite and totals them separately.

#include <gtest/gtest.h>

namespace pj::scene3d::test {

/// ctest's conventional "this test did not run" exit code, matching the
/// SKIP_RETURN_CODE set on these targets in CMakeLists.txt.
inline constexpr int kSkipExitCode = 77;

/// Run the suite, returning kSkipExitCode when every case that was due to run
/// skipped. A partial skip still reports success: some coverage did happen, and
/// hiding that behind a blanket "skipped" would be its own false signal.
[[nodiscard]] inline int runTestsReportingSkip() {
  const int status = RUN_ALL_TESTS();
  const ::testing::UnitTest& unit = *::testing::UnitTest::GetInstance();
  const int due = unit.test_to_run_count();
  if (status == 0 && due > 0 && unit.skipped_test_count() == due) {
    return kSkipExitCode;
  }
  return status;
}

}  // namespace pj::scene3d::test
