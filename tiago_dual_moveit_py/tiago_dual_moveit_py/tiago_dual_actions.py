from tiago_dual_moveit_py.tiago_dual_moveit_py import TiagoDualPy

class TiagoDualActions(TiagoDualPy):
    def __init__(self, name='tiago_dual_actions_moveit_py'):
        super().__init__(name)
        

    def change_hands(self, giver_arm, receiver_arm, vel_factor = 0.2, sleep_time=0.1):
        if (giver_arm != 'right' or giver_arm != 'left') or (receiver_arm != 'right' or receiver_arm != 'left'):
            self.logger.error('Wrong Arm Selected: Arm must be "left" or "right".')
        else:
            self.logger.info("Going to change hands position...")
            self.arm_go_to_named_pose(arm=giver_arm, pose_name='pre-give', vel_factor=vel_factor, sleep_time=sleep_time)
            self.arm_go_to_named_pose(arm=receiver_arm, pose_name='pre-collect', vel_factor=vel_factor, sleep_time=sleep_time)
            self.open_gripper(arm=receiver_arm, vel_factor=vel_factor, sleep_time=sleep_time)

            self.logger.info("Changing hands...")
            self.arm_go_to_named_pose(arm=giver_arm, pose_name='give', vel_factor=vel_factor, sleep_time=sleep_time)
            self.arm_go_to_named_pose(arm=receiver_arm, pose_name='collect', vel_factor=vel_factor, sleep_time=vel_factor)

            self.close_gripper(receiver_arm, vel_factor=vel_factor, sleep_time=sleep_time)
            self.open_gripper(giver_arm, vel_factor=vel_factor, sleep_time=sleep_time)

            self.arm_go_to_named_pose(arm=giver_arm, pose_name='pre-give', vel_factor=vel_factor, sleep_time=sleep_time)

            self.arm_go_to_named_pose(arm=receiver_arm, pose_name='pre-collect', vel_factor=vel_factor, sleep_time=sleep_time)
            self.logger.info("Hands changed.")