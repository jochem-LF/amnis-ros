"""PID Controller implementation for closed-loop control.

This module provides a standard PID (Proportional-Integral-Derivative) controller
with anti-windup, output limiting, and derivative filtering capabilities.
"""

import time
from typing import Optional


class PIDController:
    """PID controller with anti-windup and output limiting.
    
    Implements the standard PID algorithm:
    output = Kp * error + Ki * integral(error) + Kd * derivative(error)
    
    Features:
    - Proportional, Integral, and Derivative terms
    - Anti-windup (integral clamping)
    - Output limiting
    - Automatic time delta calculation
    - Reset capability
    """
    
    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_limit: float = 100.0,
        integral_limit: Optional[float] = None,
        deadband: float = 0.0
    ):
        """Initialize the PID controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limit: Maximum absolute output value
            integral_limit: Maximum absolute integral value (anti-windup), 
                           None = use output_limit
            deadband: Error threshold below which no action is taken
        """
        # PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Limits
        self.output_limit = abs(output_limit)
        self.integral_limit = abs(integral_limit) if integral_limit is not None else self.output_limit
        self.deadband = abs(deadband)
        
        # State variables
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time: Optional[float] = None
        
        # For debugging/monitoring
        self._p_term = 0.0
        self._i_term = 0.0
        self._d_term = 0.0
    
    def update(self, error: float, dt: Optional[float] = None) -> float:
        """Update the PID controller with a new error value.
        
        Args:
            error: Current error (setpoint - measured_value)
            dt: Time delta since last update (seconds). If None, calculated automatically.
            
        Returns:
            Controller output (control signal)
        """
        # Calculate time delta if not provided
        current_time = time.time()
        if dt is None:
            if self._last_time is not None:
                dt = current_time - self._last_time
            else:
                dt = 0.0  # First call, no derivative
        self._last_time = current_time
        
        # Apply deadband
        if abs(error) < self.deadband:
            error = 0.0
        
        # Proportional term
        self._p_term = self.kp * error
        
        # Integral term (with anti-windup)
        if dt > 0:
            self._integral += error * dt
            # Clamp integral to prevent windup
            self._integral = max(-self.integral_limit, min(self.integral_limit, self._integral))
        self._i_term = self.ki * self._integral
        
        # Derivative term
        if dt > 0:
            derivative = (error - self._last_error) / dt
            self._d_term = self.kd * derivative
        else:
            self._d_term = 0.0
        
        # Calculate total output
        output = self._p_term + self._i_term + self._d_term
        
        # Limit output
        output = max(-self.output_limit, min(self.output_limit, output))
        
        # Store error for next iteration
        self._last_error = error
        
        return output
    
    def reset(self):
        """Reset the controller state (integral, derivative, time)."""
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None
        self._p_term = 0.0
        self._i_term = 0.0
        self._d_term = 0.0
    
    def set_gains(self, kp: Optional[float] = None, ki: Optional[float] = None, 
                  kd: Optional[float] = None):
        """Update PID gains.
        
        Args:
            kp: New proportional gain (None = keep current)
            ki: New integral gain (None = keep current)
            kd: New derivative gain (None = keep current)
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
    
    def get_terms(self) -> tuple[float, float, float]:
        """Get the individual P, I, D terms from last update.
        
        Returns:
            Tuple of (p_term, i_term, d_term)
        """
        return (self._p_term, self._i_term, self._d_term)
    
    def get_integral(self) -> float:
        """Get the current integral value.
        
        Returns:
            Current integral value
        """
        return self._integral
    
    def __repr__(self) -> str:
        """String representation of the controller."""
        return (f"PIDController(Kp={self.kp}, Ki={self.ki}, Kd={self.kd}, "
                f"output_limit={self.output_limit})")
