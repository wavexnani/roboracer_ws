#!/usr/bin/env python3
"""
Unit test verifying that the QP control cost formulation 0.5 * u^T * P * u + q^T * u
exactly matches the analytical control cost:
  J_control = sum_{k=0}^{N-1} u_k^T R u_k
            + (u_0 - u_prev)^T Rd (u_0 - u_prev)
            + sum_{k=1}^{N-1} (u_k - u_{k-1})^T Rd (u_k - u_{k-1})
up to an additive constant (u_prev^T Rd u_prev) across random control sequences.
"""

import numpy as np
import scipy.sparse as sp


def test_rate_cost_formulation():
    N = 10
    nu = 2
    w_a = 0.1
    w_delta = 0.35
    w_da = 0.2
    w_ddelta = 2.2

    R = np.diag([w_a, w_delta])
    Rd = np.diag([w_da, w_ddelta])
    r_diag = np.array([w_a, w_delta])
    rd_diag = np.array([w_da, w_ddelta])

    u_prev = np.array([0.45, -0.12])

    # Build P_u and q_u using the proposed sparse formulation
    P_rows = []
    P_cols = []
    P_data = []
    q_u = np.zeros(N * nu)

    for k in range(N):
        uidx = k * nu
        if N == 1:
            diag_val = (r_diag + rd_diag) * 2.0
        elif k == N - 1:
            diag_val = (r_diag + rd_diag) * 2.0
        else:
            diag_val = (r_diag + 2.0 * rd_diag) * 2.0

        for j in range(nu):
            P_rows.append(uidx + j)
            P_cols.append(uidx + j)
            P_data.append(diag_val[j])

        if k > 0:
            u_prev_idx = (k - 1) * nu
            for j in range(nu):
                # Off-diagonals
                P_rows.append(uidx + j)
                P_cols.append(u_prev_idx + j)
                P_data.append(-2.0 * rd_diag[j])
                P_rows.append(u_prev_idx + j)
                P_cols.append(uidx + j)
                P_data.append(-2.0 * rd_diag[j])

    q_u[0:nu] = -2.0 * rd_diag * u_prev

    P_sparse = sp.csc_matrix((P_data, (P_rows, P_cols)), shape=(N * nu, N * nu))

    const_expected = float(u_prev.T @ Rd @ u_prev)

    np.random.seed(42)
    differences = []

    for trial in range(50):
        u_seq = np.random.randn(N, nu)

        # Analytical cost
        J_effort = sum(u_seq[k].T @ R @ u_seq[k] for k in range(N))
        J_rate_0 = (u_seq[0] - u_prev).T @ Rd @ (u_seq[0] - u_prev)
        J_rate_k = sum((u_seq[k] - u_seq[k-1]).T @ Rd @ (u_seq[k] - u_seq[k-1]) for k in range(1, N))
        J_analytical = float(J_effort + J_rate_0 + J_rate_k)

        # QP cost
        u_vec = u_seq.flatten()
        J_qp = float(0.5 * (u_vec.T @ P_sparse @ u_vec) + q_u.T @ u_vec)

        diff = J_analytical - J_qp
        differences.append(diff)

    differences = np.array(differences)
    max_err = float(np.max(np.abs(differences - const_expected)))
    std_err = float(np.std(differences))

    print(f"Expected constant (u_prev^T Rd u_prev): {const_expected:.10f}")
    print(f"Max error from constant across 50 trials: {max_err:.2e}")
    print(f"Std deviation of difference: {std_err:.2e}")
    assert np.allclose(differences, const_expected, atol=1e-12), "Unit test failed: difference is not constant!"
    print("ALL UNIT TESTS PASSED SUCCESSFULLY!")


if __name__ == '__main__':
    test_rate_cost_formulation()
