#ifndef GAIN_TABLE_H
#define GAIN_TABLE_H

#include "gnc_types.h"
#include "math_utils.h"

static inline gains_interp_t gain_table_lookup(const gain_table_t *gt, float V) {
    gains_interp_t g;
    int n = gt->n_pts;
    g.KR      = interp1_linear(gt->V_bp, gt->KR,      n, V);
    g.KA      = interp1_linear(gt->V_bp, gt->KA,      n, V);
    g.KDC     = interp1_linear(gt->V_bp, gt->KDC,     n, V);
    g.K_phi   = interp1_linear(gt->V_bp, gt->K_phi,   n, V);
    g.Kp_roll = interp1_linear(gt->V_bp, gt->Kp_roll, n, V);
    g.Ki_roll = interp1_linear(gt->V_bp, gt->Ki_roll, n, V);
    return g;
}

#endif /* GAIN_TABLE_H */
