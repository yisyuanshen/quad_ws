import numpy as np
import matplotlib.pyplot as plt

from leg_model_py.LegModel import LegModel


class LegAnalyzer(LegModel):
    def get_analytical_jacobian(self, theta_l, theta_r, theta_abad):
        """
        計算解析 Jacobian 矩陣 (3x3)。
        基於速度投影法 (Velocity Projection Method)。
        """
        geom = self._solve_2d_geometry(theta_l, theta_r)
        if geom is None: return None

        # 提取關鍵向量 (2D)
        P = geom['P']
        L = geom['L']
        R = geom['R']
        
        # 下連桿向量 (膝蓋 -> 腳尖)
        n_L = P - L
        n_R = P - R
        len_L = np.linalg.norm(n_L)
        len_R = np.linalg.norm(n_R)
        
        # 單位向量
        u_L = n_L / len_L
        u_R = n_R / len_R

        # 上連桿切線方向 (膝蓋速度方向)
        # 上連桿向量: (l_upper*cos, l_upper*sin) -> 切線: (-sin, cos)
        t_L = np.array([-np.sin(theta_l), np.cos(theta_l)])
        t_R = np.array([-np.sin(theta_r), np.cos(theta_r)])
        
        # 建立線性方程 A * v_2d = B * q_dot
        # u_L . v_p = (u_L . t_L) * l_upper * q_dot_L
        A = np.array([
            [u_L[0], u_L[1]],
            [u_R[0], u_R[1]]
        ])
        
        # 對角矩陣元素 (投影分量 * 力臂)
        b_L = np.dot(u_L, t_L) * self.l_upper
        b_R = np.dot(u_R, t_R) * self.l_upper
        B = np.diag([b_L, b_R])

        try:
            # J_2d = [dx/dq, dz/dq] (2x2)
            J_2d = np.linalg.solve(A, B)
        except np.linalg.LinAlgError:
            return None 

        # 擴展至 3D
        s_a, c_a = np.sin(theta_abad), np.cos(theta_abad)
        z_2d = P[1] 

        J = np.zeros((3, 3))
        
        # Column 0, 1 (Theta_L, Theta_R)
        # dx = dx_2d
        J[0, :2] = J_2d[0, :]
        # dy = -sin(a) * dz_2d
        J[1, :2] = -s_a * J_2d[1, :]
        # dz = cos(a) * dz_2d
        J[2, :2] = c_a * J_2d[1, :]

        # Column 2 (Theta_Abad)
        # dx/da = 0
        J[0, 2] = 0
        # dy/da = -z_2d * cos(a)
        J[1, 2] = -z_2d * c_a
        # dz/da = -z_2d * sin(a)
        J[2, 2] = -z_2d * s_a

        return J

    def compute_isotropic_index(self, J):
        """計算各向同性指標 (0~1)，即 Condition Number 的倒數"""
        if J is None: return np.nan
        try:
            U, S, Vh = np.linalg.svd(J)
            if S[0] == 0: return 0.0
            return S[-1] / S[0] # sigma_min / sigma_max
        except:
            return np.nan

    def scan_full_workspace(self, res=200):
        # 自動計算最大物理範圍
        max_reach = self.l_upper + self.l_lower
        limit = max_reach * 1.1 # 多留一點邊界
        
        x_vals = np.linspace(-limit, limit, res)
        # Z 軸只看負半平面 (0 到 -max_reach)
        z_vals = np.linspace(-limit, 0.02, res) 
        
        X, Z = np.meshgrid(x_vals, z_vals)
        Quality = np.zeros_like(X)

        valid_mask = np.zeros_like(X, dtype=bool)

        print(f"Analyzing workspace... Resolution: {res}x{res}")

        for i in range(res):
            for j in range(res):
                x = X[i, j]
                z = Z[i, j]
                
                # 假設切面在 y=0 (Abad=0)
                ik = self.inverse_kinematics(x, 0, z)
                
                if ik:
                    t_l, t_r, t_a = ik
                    J = self.get_analytical_jacobian(t_l, t_r, t_a)
                    score = self.compute_isotropic_index(J)
                    
                    if not np.isnan(score):
                        Quality[i, j] = score
                        valid_mask[i, j] = True
                    else:
                        Quality[i, j] = np.nan
                else:
                    Quality[i, j] = np.nan

        return X, Z, Quality, valid_mask

    def plot_heatmap(self, res=200):
        X, Z, Q, mask = self.scan_full_workspace(res)

        fig, ax = plt.subplots(figsize=(10, 8))
        ax.set_facecolor('#e0e0e0') # 無解區域為灰色

        # 繪圖參數
        levels = np.linspace(0, 1, 256) # 256 階層保證平滑
        cmap = plt.get_cmap('RdYlGn')
        
        # 繪製 Contour
        # 使用 extend='neither' 確保顏色條嚴格對應 0-1
        cont = ax.contourf(X, Z, Q, levels=levels, cmap=cmap, extend='neither')
        
        # 繪製 Colorbar
        cbar = fig.colorbar(cont, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Kinematic Isotropy ($\sigma_{min}/\sigma_{max}$)', rotation=270, labelpad=20, fontsize=12)
        cbar.set_ticks([0, 0.25, 0.5, 0.75, 1.0])
        
        # 繪製機構基座
        ax.plot([-self.w, self.w], [0, 0], 'k-', lw=5, solid_capstyle='round', label='Base')
        ax.scatter([-self.w, self.w], [0, 0], s=100, c='k', zorder=5)

        # 調整顯示範圍 (自動裁切到有資料的區域)
        if np.any(mask):
            x_valid = X[mask]
            z_valid = Z[mask]
            margin = 0.05
            ax.set_xlim(x_valid.min() - margin, x_valid.max() + margin)
            ax.set_ylim(z_valid.min() - margin, 0.05)
        
        ax.set_aspect('equal')
        ax.set_title('Analytical Jacobian Workspace Analysis', fontsize=14, fontweight='bold', pad=15)
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Z Position (m)')
        ax.grid(True, linestyle='--', alpha=0.3, color='black')

        plt.tight_layout()
        plt.show()
        

def main():
    # 參數設定
    d_base = 0.07
    l_up = 0.08
    l_low = 0.12
    
    analyzer = LegAnalyzer(d_base, l_up, l_low)
    analyzer.plot_heatmap(res=1000)