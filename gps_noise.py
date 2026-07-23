import math
import random
import time

class GPSNoiseGenerator:
    """
    Simulates GPS noise by adding zero-mean Gaussian white noise and 
    a First-Order Gauss-Markov process (bounded random walk drift) to 3D position 
    and velocity vectors.

    Horizontal (North, East) and Vertical (Down / Up) noise standard deviations 
    can be configured independently for both position and velocity.
    """

    def __init__(
        self,
        enabled=True,
        tau_s=30.0,
        pos_horiz_white_std=0.5,
        pos_horiz_walk_std=0.2,
        pos_vert_white_std=1.0,
        pos_vert_walk_std=0.4,
        vel_horiz_white_std=0.05,
        vel_horiz_walk_std=0.02,
        vel_vert_white_std=0.1,
        vel_vert_walk_std=0.04,
        seed=None
    ):
        self.enabled = enabled
        self.tau_s = max(0.001, float(tau_s))

        # Position noise parameters
        self.pos_horiz_white_std = float(pos_horiz_white_std)
        self.pos_horiz_walk_std = float(pos_horiz_walk_std)
        self.pos_vert_white_std = float(pos_vert_white_std)
        self.pos_vert_walk_std = float(pos_vert_walk_std)

        # Velocity noise parameters
        self.vel_horiz_white_std = float(vel_horiz_white_std)
        self.vel_horiz_walk_std = float(vel_horiz_walk_std)
        self.vel_vert_white_std = float(vel_vert_white_std)
        self.vel_vert_walk_std = float(vel_vert_walk_std)

        # Random generator state
        self._rng = random.Random(seed)

        # Internal state for Gauss-Markov drift components (North, East, Down)
        self.last_pos_time = None
        self.pos_walk_n = 0.0
        self.pos_walk_e = 0.0
        self.pos_walk_d = 0.0

        self.last_vel_time = None
        self.vel_walk_n = 0.0
        self.vel_walk_e = 0.0
        self.vel_walk_d = 0.0

    @classmethod
    def from_dict(cls, config_dict):
        """Constructs GPSNoiseGenerator from a dictionary configuration."""
        if not config_dict:
            return cls(enabled=False)

        pos_cfg = config_dict.get("position", {})
        vel_cfg = config_dict.get("velocity", {})

        return cls(
            enabled=config_dict.get("enabled", True),
            tau_s=config_dict.get("tau_s", 30.0),
            pos_horiz_white_std=pos_cfg.get("horizontal_white_std_m", 0.5),
            pos_horiz_walk_std=pos_cfg.get("horizontal_walk_std_m", 0.2),
            pos_vert_white_std=pos_cfg.get("vertical_white_std_m", 1.0),
            pos_vert_walk_std=pos_cfg.get("vertical_walk_std_m", 0.4),
            vel_horiz_white_std=vel_cfg.get("horizontal_white_std_ms", 0.05),
            vel_horiz_walk_std=vel_cfg.get("horizontal_walk_std_ms", 0.02),
            vel_vert_white_std=vel_cfg.get("vertical_white_std_ms", 0.1),
            vel_vert_walk_std=vel_cfg.get("vertical_walk_std_ms", 0.04),
            seed=config_dict.get("seed", None)
        )

    def reset(self):
        """Resets all random walk drift states."""
        self.last_pos_time = None
        self.pos_walk_n = 0.0
        self.pos_walk_e = 0.0
        self.pos_walk_d = 0.0

        self.last_vel_time = None
        self.vel_walk_n = 0.0
        self.vel_walk_e = 0.0
        self.vel_walk_d = 0.0

    def _update_gauss_markov(self, current_val, walk_std, dt):
        """
        Updates a 1D First-Order Gauss-Markov state:
        x(k+1) = exp(-dt/tau) * x(k) + w(k)
        where w(k) ~ N(0, walk_std * sqrt(1 - exp(-2*dt/tau)))
        """
        if dt <= 0.0 or walk_std <= 0.0:
            return current_val

        beta = 1.0 / self.tau_s
        alpha = math.exp(-beta * dt)
        # Driven process variance to maintain steady-state variance of walk_std^2
        driven_std = walk_std * math.sqrt(max(0.0, 1.0 - math.exp(-2.0 * beta * dt)))
        w = self._rng.gauss(0.0, driven_std)
        return alpha * current_val + w

    def apply_position_noise(self, current_time, north, east, down):
        """
        Applies position noise (Gaussian white + Gauss-Markov drift) to (North, East, Down).
        Returns (noisy_north, noisy_east, noisy_down).
        """
        if not self.enabled:
            return north, east, down

        # Compute dt for Gauss-Markov update
        if self.last_pos_time is not None:
            dt = current_time - self.last_pos_time
            if 0.0 < dt < 10.0:  # Ignore negative or excessively long time jumps
                self.pos_walk_n = self._update_gauss_markov(self.pos_walk_n, self.pos_horiz_walk_std, dt)
                self.pos_walk_e = self._update_gauss_markov(self.pos_walk_e, self.pos_horiz_walk_std, dt)
                self.pos_walk_d = self._update_gauss_markov(self.pos_walk_d, self.pos_vert_walk_std, dt)
            elif dt >= 10.0 or dt < 0.0:
                self.reset()
        self.last_pos_time = current_time

        # Generate Gaussian white noise
        white_n = self._rng.gauss(0.0, self.pos_horiz_white_std) if self.pos_horiz_white_std > 0 else 0.0
        white_e = self._rng.gauss(0.0, self.pos_horiz_white_std) if self.pos_horiz_white_std > 0 else 0.0
        white_d = self._rng.gauss(0.0, self.pos_vert_white_std) if self.pos_vert_white_std > 0 else 0.0

        noisy_north = north + self.pos_walk_n + white_n
        noisy_east = east + self.pos_walk_e + white_e
        noisy_down = down + self.pos_walk_d + white_d

        return noisy_north, noisy_east, noisy_down

    def apply_velocity_noise(self, current_time, vn, ve, vd):
        """
        Applies velocity noise (Gaussian white + Gauss-Markov drift) to (Vn, Ve, Vd).
        Returns (noisy_vn, noisy_ve, noisy_vd).
        """
        if not self.enabled:
            return vn, ve, vd

        # Compute dt for Gauss-Markov update
        if self.last_vel_time is not None:
            dt = current_time - self.last_vel_time
            if 0.0 < dt < 10.0:
                self.vel_walk_n = self._update_gauss_markov(self.vel_walk_n, self.vel_horiz_walk_std, dt)
                self.vel_walk_e = self._update_gauss_markov(self.vel_walk_e, self.vel_horiz_walk_std, dt)
                self.vel_walk_d = self._update_gauss_markov(self.vel_walk_d, self.vel_vert_walk_std, dt)
            elif dt >= 10.0 or dt < 0.0:
                self.reset()
        self.last_vel_time = current_time

        # Generate Gaussian white noise
        white_vn = self._rng.gauss(0.0, self.vel_horiz_white_std) if self.vel_horiz_white_std > 0 else 0.0
        white_ve = self._rng.gauss(0.0, self.vel_horiz_white_std) if self.vel_horiz_white_std > 0 else 0.0
        white_vd = self._rng.gauss(0.0, self.vel_vert_white_std) if self.vel_vert_white_std > 0 else 0.0

        noisy_vn = vn + self.vel_walk_n + white_vn
        noisy_ve = ve + self.vel_walk_e + white_ve
        noisy_vd = vd + self.vel_walk_d + white_vd

        return noisy_vn, noisy_ve, noisy_vd
