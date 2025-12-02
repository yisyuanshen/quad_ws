#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class FKNode(Node):
    def __init__(self):
        super().__init__('fk_node')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.t = 0.0

        # --- 1. 機構參數 (必須與 URDF const.xacro 一致) ---
        self.L1 = 0.10  # 上連桿長
        self.L2 = 0.12  # 下連桿長
        
        # 馬達軸心偏置：前後馬達距離中心各 0.02
        self.MOTOR_X_OFFSET = 0.02 

        # --- 2. 定義所有關節名稱 ---
        self.all_joints = [
            # 左前 (LF)
            'lf_abad_joint', 'lf_motor_l_joint', 'lf_knee_l_joint', 'lf_motor_r_joint', 'lf_knee_r_joint',
            # 右前 (RF)
            'rf_abad_joint', 'rf_motor_l_joint', 'rf_knee_l_joint', 'rf_motor_r_joint', 'rf_knee_r_joint',
            # 左後 (LH)
            'lh_abad_joint', 'lh_motor_l_joint', 'lh_knee_l_joint', 'lh_motor_r_joint', 'lh_knee_r_joint',
            # 右後 (RH)
            'rh_abad_joint', 'rh_motor_l_joint', 'rh_knee_l_joint', 'rh_motor_r_joint', 'rh_knee_r_joint'
        ]

    def solve_2link_fk(self, local_target_x, local_target_z, direction):
        """
        計算單一條鏈的 FK (餘弦定理)
        direction: 1 = 膝蓋向前彎 (凸向 x+), -1 = 膝蓋向後彎 (凸向 x-)
        """
        # 1. 距離計算
        D = math.sqrt(local_target_x**2 + local_target_z**2)
        
        # 安全限制：避免目標超出腿長導致數學錯誤
        max_reach = self.L1 + self.L2
        if D > max_reach: 
            D = max_reach - 0.0001
        
        # 2. 計算馬達角度 (Theta)
        phi = math.atan2(local_target_z, local_target_x)
        
        # 餘弦定理求 alpha (上連桿與虛擬連線的夾角)
        cos_alpha = (self.L1**2 + D**2 - self.L2**2) / (2 * self.L1 * D)
        cos_alpha = max(min(cos_alpha, 1.0), -1.0) # 數值保護
        alpha = math.acos(cos_alpha)
        
        # 決定膝蓋彎曲方向 (Forward/Backward)
        if direction == 1:
            theta = phi + alpha # 膝蓋在前
        else:
            theta = phi - alpha # 膝蓋在後
            
        # 3. 計算膝蓋角度 (僅供 Rviz 視覺化閉合使用)
        cos_beta = (self.L1**2 + self.L2**2 - D**2) / (2 * self.L1 * self.L2)
        cos_beta = max(min(cos_beta, 1.0), -1.0)
        beta = math.acos(cos_beta)
        
        # 根據 direction 決定膝蓋轉向
        if direction == 1:
            knee_angle = -(math.pi - beta) 
        else:
            knee_angle = (math.pi - beta)
        
        return theta, knee_angle

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.all_joints
        
        # 初始化所有角度為 0
        positions = [0.0] * len(self.all_joints)
        
        # --- 3. 產生畫圓軌跡 (所有腳做一樣動作) ---
        radius = 0.06
        center_z = -0.15 
        gx = radius * math.sin(self.t)
        gz = center_z + radius * math.cos(self.t)
        
        # ==========================================
        # 4. 核心 FK 計算 (算出 "前構型" 與 "後構型" 的角度)
        # ==========================================
        
        # --- A. 後側馬達 FK (Back Motor Logic) ---
        # 位於負 X (-0.02)，膝蓋向後彎 (direction=-1) 形成菱形後半部 <
        # 注意：根據您的測試，這裡 direction 設為 -1 是正確的後半部形狀
        x_back_target = gx - (-self.MOTOR_X_OFFSET)
        theta_back, knee_back = self.solve_2link_fk(x_back_target, gz, direction=-1)
        
        # --- B. 前側馬達 FK (Front Motor Logic) ---
        # 位於正 X (+0.02)，膝蓋向前彎 (direction=1) 形成菱形前半部 >
        x_front_target = gx - (self.MOTOR_X_OFFSET)
        theta_front, knee_front = self.solve_2link_fk(x_front_target, gz, direction=1)
        
        # ==========================================
        # 5. 分配數據到四條腿 (Mapping)
        # ==========================================
        # 根據您驗證的結果：
        # - 左側腿 (LF, LH) 的 L Joint 接 theta_front, R Joint 接 theta_back
        # - 右側腿 (RF, RH) 的 L Joint 接 theta_back,  R Joint 接 theta_front
        
        # --- 左前 (LF) & 左後 (LH) ---
        for prefix in ['lf', 'lh']:
            idx_l = self.all_joints.index(f'{prefix}_motor_l_joint')
            idx_lk = self.all_joints.index(f'{prefix}_knee_l_joint')
            idx_r = self.all_joints.index(f'{prefix}_motor_r_joint')
            idx_rk = self.all_joints.index(f'{prefix}_knee_r_joint')
            
            # 左側腿映射邏輯
            positions[idx_l] = theta_front + math.pi/2
            positions[idx_lk] = knee_front
            positions[idx_r] = theta_back + math.pi/2
            positions[idx_rk] = knee_back

        # --- 右前 (RF) & 右後 (RH) ---
        for prefix in ['rf', 'rh']:
            idx_l = self.all_joints.index(f'{prefix}_motor_l_joint')
            idx_lk = self.all_joints.index(f'{prefix}_knee_l_joint')
            idx_r = self.all_joints.index(f'{prefix}_motor_r_joint')
            idx_rk = self.all_joints.index(f'{prefix}_knee_r_joint')
            
            # 右側腿映射邏輯 (左右馬達定義相反)
            positions[idx_l] = theta_back + math.pi/2
            positions[idx_lk] = knee_back
            positions[idx_r] = theta_front + math.pi/2
            positions[idx_rk] = knee_front

        # 發送指令
        msg.position = positions
        self.publisher_.publish(msg)
        self.t += 0.05

def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()