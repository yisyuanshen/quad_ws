import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class LegModel:
    def __init__(self, d, l_upper, l_lower):
        self.w = d / 2
        self.l_upper = l_upper
        self.l_lower = l_lower
        self.state = {}

    def forward_kinematics(self, theta_left, theta_right, theta_abad=0.0):
        geom_2d = self._solve_2d_geometry(theta_left, theta_right)
        
        if geom_2d is None:
            return None

        state_3d = self._transform_to_3d(geom_2d, theta_abad)
        self.state = state_3d
        return state_3d['P']

    def inverse_kinematics(self, x, y, z):
        theta_abad = np.arctan2(y, -z)
        
        x_2d = x
        z_2d = -np.sqrt(y**2 + z**2)
        p_2d = np.array([x_2d, z_2d])
        
        origin_left = np.array([-self.w, 0])
        origin_right = np.array([self.w, 0])
        
        theta_left = self._solve_single_leg_ik(p_2d, origin_left, is_left_leg=True)
        theta_right = self._solve_single_leg_ik(p_2d, origin_right, is_left_leg=False)

        if theta_left is None or theta_right is None:
            return None

        return theta_left, theta_right, theta_abad

    def compute_trajectory(self, thetas_left, thetas_right, thetas_abad):
        trajectory = []
        if np.isscalar(thetas_abad):
            thetas_abad = np.full_like(thetas_left, thetas_abad)

        for tl, tr, ta in zip(thetas_left, thetas_right, thetas_abad):
            pos = self.forward_kinematics(tl, tr, ta)
            if pos is not None:
                trajectory.append(self.state.copy())
            else:
                trajectory.append(None)
        return trajectory

    def plot(self, theta_left, theta_right, theta_abad=0.0):
        if isinstance(theta_left, (list, np.ndarray)) and len(theta_left) > 1:
            frames = self.compute_trajectory(theta_left, theta_right, theta_abad)
            self._animate(frames)
        else:
            self.forward_kinematics(theta_left, theta_right, theta_abad)
            self._draw_static()

    def _solve_2d_geometry(self, theta_l, theta_r):
        origin_l = np.array([-self.w, 0])
        origin_r = np.array([self.w, 0])

        knee_l = origin_l + self.l_upper * np.array([np.cos(theta_l), np.sin(theta_l)])
        knee_r = origin_r + self.l_upper * np.array([np.cos(theta_r), np.sin(theta_r)])

        vec_lr = knee_r - knee_l
        dist_lr = np.linalg.norm(vec_lr)
        half_dist = dist_lr / 2

        if half_dist >= self.l_lower:
            return None

        h = np.sqrt(self.l_lower**2 - half_dist**2)
        mid_point = (knee_l + knee_r) / 2

        direction = vec_lr / dist_lr
        perp_vector = np.array([-direction[1], direction[0]])

        p1 = mid_point + h * perp_vector
        p2 = mid_point - h * perp_vector

        foot_p = p1 if p1[1] < p2[1] else p2

        return {
            'O_L': origin_l, 'O_R': origin_r,
            'L': knee_l, 'R': knee_r,
            'H': mid_point, 'P': foot_p
        }

    def _transform_to_3d(self, geom_2d, theta_abad):
        c, s = np.cos(theta_abad), np.sin(theta_abad)
        rot_x = np.array([
            [1, 0, 0],
            [0, c, -s],
            [0, s, c]
        ])

        def rotate(p_2d):
            p_3d_local = np.array([p_2d[0], 0, p_2d[1]]) 
            return rot_x @ p_3d_local

        return {k: rotate(v) for k, v in geom_2d.items()}

    def _solve_single_leg_ik(self, target_p, origin, is_left_leg):
        vec = target_p - origin
        dist = np.linalg.norm(vec)

        if dist > (self.l_upper + self.l_lower) or dist < abs(self.l_upper - self.l_lower):
            return None

        cos_alpha = (self.l_upper**2 + dist**2 - self.l_lower**2) / (2 * self.l_upper * dist)
        alpha = np.arccos(np.clip(cos_alpha, -1.0, 1.0))
        phi = np.arctan2(vec[1], vec[0])

        return phi - alpha if is_left_leg else phi + alpha

    def _draw_static(self):
        if not self.state: return
        fig, ax = plt.subplots(figsize=(8, 8))
        self._render_frame(ax, self.state)
        self._configure_axes(ax)
        
        pos = self.state['P']
        ax.set_title(f"Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
        plt.show()

    def _animate(self, frames):
        valid_frames = [f for f in frames if f is not None]
        if not valid_frames: return

        fig, ax = plt.subplots(figsize=(8, 8))

        def update(idx):
            ax.clear()
            self._render_frame(ax, valid_frames[idx])
            self._configure_axes(ax)

        ani = FuncAnimation(fig, update, frames=len(valid_frames), interval=30)
        plt.show()

    def _render_frame(self, ax, s):
        ax.plot([s['O_L'][0], s['O_R'][0]], [s['O_L'][2], s['O_R'][2]], 'k-', lw=4, alpha=0.6)
        
        ax.plot([s['O_L'][0], s['L'][0]], [s['O_L'][2], s['L'][2]], 'r-o', lw=2)
        ax.plot([s['L'][0], s['P'][0]],   [s['L'][2], s['P'][2]],   'r--', lw=1.5)
        
        ax.plot([s['O_R'][0], s['R'][0]], [s['O_R'][2], s['R'][2]], 'b-o', lw=2)
        ax.plot([s['R'][0], s['P'][0]],   [s['R'][2], s['P'][2]],   'b--', lw=1.5)
        
        ax.plot(s['P'][0], s['P'][2], 'g^', markersize=10)

    def _configure_axes(self, ax):
        limit = self.l_upper + self.l_lower + self.w
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit * 0.5)
        ax.set_aspect('equal')
        ax.grid(True)


def main():
    leg = LegModel(d=0.07, l_upper=0.08, l_lower=0.12)
    
    target = (0, 0, -0.15)
    ik_result = leg.inverse_kinematics(*target)

    if ik_result:
        t_l, t_r, t_abad = ik_result
        print(f"\nIK Angles (deg) -> L: {np.rad2deg(t_l):.1f}, R: {np.rad2deg(t_r):.1f}, ABAD: {np.rad2deg(t_abad):.1f}")
        
        fk_pos = leg.forward_kinematics(t_l, t_r, t_abad)
        print(f"FK Verification -> X: {fk_pos[0]:.2f}, Y:{fk_pos[1]:.2f}, Z:{fk_pos[2]:.2f}\n")
        leg.plot(t_l, t_r, t_abad)
    
    steps = 100
    time = np.linspace(0, 4 * np.pi, steps)
    traj_l = np.deg2rad(-135) + 0.5 * np.cos(time)
    traj_r = np.deg2rad(-45) - 0.5 * np.sin(time)
    traj_abad = np.zeros_like(time)
    
    print("Running Animation...\n")
    leg.plot(traj_l, traj_r, traj_abad)

if __name__ == "__main__":
    main()