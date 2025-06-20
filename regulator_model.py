import numpy as np
from scipy.linalg import solve_discrete_are
import warnings
    
class RegulatorModel:
    def __init__(self, N, q, m, n):
        self.A = None
        self.B = None
        self.C = None
        self.Q = None
        self.R = None
        self.N = N
        self.q = q #  output dimension
        self.m = m #  input dimension
        self.n = n #  state dimension

    def compute_H_and_F(self, S_bar, T_bar, Q_bar, R_bar):
        # Compute H
        H = np.dot(S_bar.T, np.dot(Q_bar, S_bar)) + R_bar

        # Compute F
        F = np.dot(S_bar.T, np.dot(Q_bar, T_bar))

        return H, F

    def propagation_model_regulator_fixed_std(self):
        S_bar = np.zeros((self.N*self.q, self.N*self.m))
        T_bar = np.zeros((self.N*self.q, self.n))
        Q_bar = np.zeros((self.N*self.q, self.N*self.q))
        R_bar = np.zeros((self.N*self.m, self.N*self.m))

        for k in range(1, self.N + 1):
            for j in range(1, k + 1):
                S_bar[(k-1)*self.q:k*self.q, (k-j)*self.m:(k-j+1)*self.m] = np.dot(np.dot(self.C, np.linalg.matrix_power(self.A, j-1)), self.B)

            T_bar[(k-1)*self.q:k*self.q, :self.n] = np.dot(self.C, np.linalg.matrix_power(self.A, k))

            Q_bar[(k-1)*self.q:k*self.q, (k-1)*self.q:k*self.q] = self.Q
            R_bar[(k-1)*self.m:k*self.m, (k-1)*self.m:k*self.m] = self.R

        return S_bar, T_bar, Q_bar, R_bar
    
    def updateSystemMatrices(self,sim,cur_x,cur_u):
        """
        Get the system matrices A and B according to the dimensions of the state and control input.
        
        Parameters:
        num_states, number of system states
        num_controls, number oc conttrol inputs
        cur_x, current state around which to linearize
        cur_u, current control input around which to linearize
       
        
        Returns:
        A: State transition matrix
        B: Control input matrix
        """
        # Check if state_x_for_linearization and cur_u_for_linearization are provided
        if cur_x is None or cur_u is None:
            raise ValueError(
                "state_x_for_linearization and cur_u_for_linearization are not specified.\n"
                "Please provide the current state and control input for linearization.\n"
                "Hint: Use the goal state (e.g., zeros) and zero control input at the beginning.\n"
                "Also, ensure that you implement the linearization logic in the updateSystemMatrices function."
            )
        
        A =[]
        B = []
        num_states = self.n
        num_controls = self.m
        num_outputs = self.q

        # Extract components of current state and control input
        x0, y0, theta0 = cur_x
        v0, omega0 = cur_u

        # Get time step from simulation
        delta_t = sim.GetTimeStep()

        # ..................................................................

        # # Compute continuous-time matrices A_c and B_c
        # A_c = np.array([
        #     [0, 0, -v0 * np.sin(theta0)],
        #     [0, 0,  v0 * np.cos(theta0)],
        #     [0, 0, 0]
        # ])

        # B_c = np.array([
        #     [np.cos(theta0), 0],
        #     [np.sin(theta0), 0],
        #     [0, 1]
        # ])

        # # Discretize A_c and B_c to get A and B
        # A = np.eye(3) + delta_t * A_c
        # B = delta_t * B_c

        # ..................................................................

        # Implementing Euclidean logic for A nd B matrices
        A = np.zeros((num_states,num_states)) #3x3 Matrix for state matrix
        A[0,2] = -(cur_u[0]) * delta_t * np.sin(0)
        A[1,2] = cur_u[0] * delta_t * np.cos(0)
        
        euclidean_distance_Y = np.sqrt((cur_x[1] - 0)**2)
        
        if(euclidean_distance_Y > 0.1):
            A[1,1] = 1  # Align with Y-Axis
            # print("Aligning with Just Y Axis")
        else:
            A[1,1] = 0  # Align with Y-Axis
            A[2,2] = 1 # Align theta - 0 degrees
            # print("Aligning with theta")
            if(cur_x[2] < 0.1):
                    A[0,0] = 1 # Align with X-Axis
                    A[1,1] = 1  # Align with Y-Axis
                    A[2,2] = 1 # Align theta - 0 degrees
                    # print("Aligning with X Axis")
        
        B = np.array([
            [np.cos(cur_x[2]), 0],
            [np.sin(cur_x[2]), 0],
            [0, 1]
        ]) * delta_t  

        # ..................................................................

        self.A = A
        self.B = B
        self.C = np.eye(num_outputs)
        
    # TODO you can change this function to allow for more passing a vector of gains
    def setCostMatrices(self, Qcoeff, Rcoeff):
        """
        Set the cost matrices Q and R for the MPC controller.

        Parameters:
        Qcoeff: float or array-like
            State cost coefficient(s). If scalar, the same weight is applied to all states.
            If array-like, should have a length equal to the number of states.

        Rcoeff: float or array-like
            Control input cost coefficient(s). If scalar, the same weight is applied to all control inputs.
            If array-like, should have a length equal to the number of control inputs.

        Sets:
        self.Q: ndarray
            State cost matrix.
        self.R: ndarray
            Control input cost matrix.
        """
        import numpy as np

        num_states = self.n
        num_controls = self.m

        # Process Qcoeff
        if np.isscalar(Qcoeff):
            # If Qcoeff is a scalar, create an identity matrix scaled by Qcoeff
            Q = Qcoeff * np.eye(num_states)
        else:
            # Convert Qcoeff to a numpy array
            Qcoeff = np.array(Qcoeff)
            if Qcoeff.ndim != 1 or len(Qcoeff) != num_states:
                raise ValueError(f"Qcoeff must be a scalar or a 1D array of length {num_states}")
            # Create a diagonal matrix with Qcoeff as the diagonal elements
            Q = np.diag(Qcoeff)

        # Process Rcoeff
        if np.isscalar(Rcoeff):
            # If Rcoeff is a scalar, create an identity matrix scaled by Rcoeff
            R = Rcoeff * np.eye(num_controls)
        else:
            # Convert Rcoeff to a numpy array
            Rcoeff = np.array(Rcoeff)
            if Rcoeff.ndim != 1 or len(Rcoeff) != num_controls:
                raise ValueError(f"Rcoeff must be a scalar or a 1D array of length {num_controls}")
            # Create a diagonal matrix with Rcoeff as the diagonal elements
            R = np.diag(Rcoeff)

        # Assign the matrices to the object's attributes
        self.Q = Q
        self.R = R
    
    def compute_terminal_P(self):
        """
        Compute the terminal weight matrix P using the Discrete Algebraic Riccati Equation.
        """
        if self.A is None or self.B is None or self.Q is None or self.R is None:
            raise ValueError("A, B, Q, and R must be set before computing terminal P.")
        
        # Solve for P using the Discrete Algebraic Riccati Equation (DARE)
        self.P = solve_discrete_are(self.A, self.B, self.Q, self.R)
        print("Terminal matrix P computed and stored.")

    def iterative_P(self, tol=1e-8):
        """
        Computes the matrix P using an iterative method based on the discrete algebraic Riccati equation.
        """
        P = self.Q.copy()  # Starting guess
        max_iter = 500

        for i in range(max_iter):
            # Compute the new P using the Riccati equation
            P_next = (self.Q + 
                    self.A.T @ P @ self.A - 
                    self.A.T @ P @ self.B @ np.linalg.inv(self.R + self.B.T @ P @ self.B) @ self.B.T @ P @ self.A)
            
            # Check for convergence
            if np.linalg.norm(P_next - P) < tol:
                break
            
            P = P_next
        
        # Check if the maximum number of iterations was reached without convergence
        if i == max_iter - 1:
            warnings.warn("Maximum iterations reached in iterative_P without convergence.", UserWarning)


        self.P = P  # Assign the final P to the instance variable
        return self.P

    def compute_P(self):
        """
        Computes the matrix P using the iterative approach for the discrete algebraic Riccati equation
        """
        # Ensure Q is positive semi-definite and R is positive definite
        if not np.all(np.linalg.eigvals(self.Q) >= 0):
            raise ValueError("Matrix Q must be positive semi-definite.")
        if not np.all(np.linalg.eigvals(self.R) > 0):
            raise ValueError("Matrix R must be positive definite.")
        
        # Compute P using the iterative method
        self.P = self.iterative_P()
        
        # Check if P is positive semi-definite
        eigvals = np.linalg.eigvals(self.P)
        if not np.all(eigvals >= 0):
            raise ValueError("Matrix P must be positive semi-definite.")
        
        return self.P
