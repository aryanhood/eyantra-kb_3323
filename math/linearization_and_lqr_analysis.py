## TASK 1A CODE

import sympy as sp
import numpy as np
import control as ct
from typing import List, Tuple, Callable

# ----------------------
# Symbolic variable setup
# ----------------------
x1, x2, u = sp.symbols('x1 x2 u')

# System differential equations
x1_dot = -x1 + x2 + 4*u
x2_dot = -x1 - x2 + 4*x1*(x2**2) + 2*u

# ----------------------
# Equilibrium points
# ----------------------
def find_equilibrium_points() -> List[Tuple[sp.Basic, sp.Basic]]:
    """
    Find all equilibrium points with u=0.
    Returns list of (x1, x2) symbolic tuples.
    """
    eq_x1_dot = x1_dot.subs(u, 0)
    eq_x2_dot = x2_dot.subs(u, 0)
    solutions = sp.solve((eq_x1_dot, eq_x2_dot), (x1, x2))
    
    # Normalize solutions to list of tuples
    equi_points = []
    if isinstance(solutions, list):
        for sol in solutions:
            if isinstance(sol, tuple):
                equi_points.append(sol)
            elif isinstance(sol, dict):
                equi_points.append((sol[x1], sol[x2]))
    elif isinstance(solutions, tuple):
        equi_points.append(solutions)
    elif isinstance(solutions, dict):
        equi_points.append((solutions[x1], solutions[x2]))
    else:
        raise ValueError(f"Unexpected solution type: {type(solutions)}")
    
    return equi_points

# ----------------------
# Jacobian matrices
# ----------------------
def get_jacobian_functions():
    """
    Returns numerical functions for A(x1,x2) and B(x1,x2)
    """
    A_sym = sp.Matrix([[sp.diff(x1_dot, x1), sp.diff(x1_dot, x2)],
                       [sp.diff(x2_dot, x1), sp.diff(x2_dot, x2)]])
    B_sym = sp.Matrix([[sp.diff(x1_dot, u)],
                       [sp.diff(x2_dot, u)]])
    
    # Convert symbolic matrices to fast numerical functions
    A_func = sp.lambdify((x1, x2), A_sym, modules="numpy")
    B_func = sp.lambdify((x1, x2), B_sym, modules="numpy")
    
    return A_func, B_func

def evaluate_jacobians(eq_points: List[Tuple[sp.Basic, sp.Basic]]) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """
    Evaluate A and B matrices numerically at each equilibrium point.
    """
    A_func, B_func = get_jacobian_functions()
    A_matrices, B_matrices = [], []
    
    for pt in eq_points:
        x1_val = float(pt[0])
        x2_val = float(pt[1])
        A_matrices.append(np.array(A_func(x1_val, x2_val), dtype=np.float64))
        B_matrices.append(np.array(B_func(x1_val, x2_val), dtype=np.float64).reshape(-1,1))
    
    return A_matrices, B_matrices

# ----------------------
# Eigenvalues and stability
# ----------------------
def find_eigen_values(A_matrices: List[np.ndarray]) -> Tuple[List[np.ndarray], List[str]]:
    eigen_values = []
    stability = []
    
    for A in A_matrices:
        eigvals = np.linalg.eigvals(A)
        eigen_values.append(eigvals)
        stability.append("Stable" if np.all(np.real(eigvals) < 0) else "Unstable")
    
    return eigen_values, stability

# ----------------------
# LQR gain computation
# ----------------------
def compute_lqr_gain(A_matrices: List[np.ndarray], B_matrices: List[np.ndarray], eq_points: List[Tuple[sp.Basic, sp.Basic]]) -> np.ndarray:
    """
    Compute LQR gain K for equilibrium point (-1,1)
    """
    # Find index of (-1,1)
    idx = next(i for i, pt in enumerate(eq_points) if np.isclose(float(pt[0]), -1) and np.isclose(float(pt[1]), 1))
    
    Q = np.eye(2)
    R = np.array([[1]])
    
    K, _, _ = ct.lqr(A_matrices[idx], B_matrices[idx], Q, R)
    return np.asarray(K).flatten()

# ----------------------
# Main function
# ----------------------
def main_function():
    eq_points = find_equilibrium_points()
    if not eq_points:
        print("No equilibrium points found.")
        return None, None, None, None, None
    
    A_matrices, B_matrices = evaluate_jacobians(eq_points)
    eigen_values, stability = find_eigen_values(A_matrices)
    K = compute_lqr_gain(A_matrices, B_matrices, eq_points)
    
    return eq_points, A_matrices, eigen_values, stability, K

# ----------------------
# Output function
# ----------------------
def task1a_output(eq_points, A_matrices, eigen_values, stability, K):
    print("Equilibrium Points:")
    for i, pt in enumerate(eq_points):
        print(f"  Point {i+1}: x1 = {pt[0]}, x2 = {pt[1]}")
    
    print("\nJacobian Matrices at Equilibrium Points:")
    for i, A in enumerate(A_matrices):
        print(f"  At Point {i+1}:\n{A}")
    
    print("\nEigenvalues at Equilibrium Points:")
    for i, eigvals in enumerate(eigen_values):
        print(f"  At Point {i+1}: {eigvals}")
    
    print("\nStability of Equilibrium Points:")
    for i, status in enumerate(stability):
        print(f"  At Point {i+1}: {status}")
    
    print("\nLQR Gain Matrix K at equilibrium (-1,1):")
    print(K)

# ----------------------
# Script execution
# ----------------------
if __name__ == "__main__":
    eq_points, A_matrices, eigen_values, stability, K = main_function()
    task1a_output(eq_points, A_matrices, eigen_values, stability, K)
    