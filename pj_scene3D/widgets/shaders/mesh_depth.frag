// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Deliberately empty: the shadow target has a depth attachment and no colour one, so
// there is nothing to write. The stage exists only because a QRhi graphics pipeline
// requires both, and depth is written by the fixed-function stage.
void main() {}
