import pybullet as p
import pybullet_data
import time

class TableTennisEnv:
    def __init__(self, show_gui = False):
        self.step_count = 0
        self.show_gui = show_gui
        self._plane = None
        self._surface_body_id = None
        self._table_body_id = None
        # self._table2_body_id = None
        self._robotic_arm = None
        self._ball = None
        self.state = self.init_state()

        # Throw the ball
        # self.throw_ball()
        # print(self.state)


    def init_state(self):
        if self.show_gui:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # load the plane
        self._plane = p.loadURDF('plane.urdf')
        p.changeDynamics(self._plane, -1, lateralFriction = 0.5)

        # load the robotic arm
        self._robotic_arm = p.loadURDF('TableTennisEnvironmentV0/tennis_player_bot/tennis_player_bot.urdf', basePosition=[1.7,0,1.25], baseOrientation=[0,0,0,1], globalScaling=1.0, useFixedBase=True)
        p.changeDynamics(self._robotic_arm, 9, restitution = 0.8)
        # p.changeDynamics(self._robotic_arm, 9, restitution = 1.0)


        # set the arm position 
        for i in range(100):
            joint_1 = 0
            joint_2 = 0
            joint_3 = 0
            joint_4 = 0
            joint_5 = 0
            joint_6 = 0
        
            joint_angles_vector = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6,]
            
            p.setJointMotorControlArray(self._robotic_arm, range(6), 
                                        p.POSITION_CONTROL,
                                        targetPositions=joint_angles_vector)

            p.stepSimulation()

        #load the platform
        surface_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[5, 3, 0.1])
        surface_visual_id = -1  # No visual representation for now
        self._surface_body_id = p.createMultiBody(baseMass=300.0, baseCollisionShapeIndex=surface_id, baseVisualShapeIndex=surface_visual_id, basePosition=[0, 0, 0.1])
        p.changeDynamics(self._surface_body_id, -1, restitution=0.8, lateralFriction = 0.5)  # Adjust the restitution as needed (e.g., 0.8 for a good bounce)
        # p.changeDynamics(self._surface_body_id, -1, restitution=1.0)  # Adjust the restitution as needed (e.g., 0.8 for a good bounce)


        # load the table for the tennis
        table_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1.37, 0.75, 0.5])
        table_visual_id = -1  # No visual representation for now
        self._table_body_id = p.createMultiBody(baseMass=200.0, baseCollisionShapeIndex=table_id, baseVisualShapeIndex=table_visual_id, basePosition=[0, 0, 0.7])
        p.changeDynamics(self._table_body_id, -1, restitution=0.8, lateralFriction = 0.5)  # Adjust the restitution as needed (e.g., 0.8 for a good bounce)
        # p.changeDynamics(self._table_body_id, -1, restitution=1.0)  # Adjust the restitution as needed (e.g., 0.8 for a good bounce)


        # load the secondary table for the robot
        # table2_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
        # table2_visual_id = -1  # No visual representation for now
        # self._table2_body_id = p.createMultiBody(baseMass=50.0, baseCollisionShapeIndex=table2_id, baseVisualShapeIndex=table2_visual_id, basePosition=[2, 0, 0.7])
        # p.changeDynamics(self._table2_body_id, -1, restitution=0.8, lateralFriction = 0.5)  # Adjust the restitution as needed (e.g., 0.8 for a good bounce)


        

        # load the tennis ball
        self._ball = p.loadURDF('TableTennisEnvironmentV0/tennis_ball2/urdf/tennis_ball2.urdf', basePosition = [-1, 0, 2])
        p.changeDynamics(self._ball, -1, restitution = 0.8)
        # p.changeDynamics(self._ball, -1, restitution = 1.0)
        # p.changeDynamics(self._ball, -1, mass = 0.02)
        # force_vector_ball = [3.05, 0.8, 2.4]
        # p.applyExternalForce(self._ball, -1, forceObj=force_vector_ball, posObj=[0, 0, 0], flags=p.LINK_FRAME)


        self.state = self.get_state()
        return self.state


    # get states at any time
    def get_state(self): 
        position_ball, orientation_ball = p.getBasePositionAndOrientation(self._ball)
        position_bat, orientation_bat, _, _, _, _ = p.getLinkState(self._robotic_arm, 5) ## it is the position of the end effector
        state = {
            'ball': {
                'position': position_ball,
                'orientation': orientation_ball,
            },
            'bat': {
                'position': position_bat,
                'orientation': orientation_bat,
            },
        }

        return state


    # reset the TableTennisEnvironmentV0 to initial config
    def reset(self):
        p.disconnect()
        self.state = self.init_state()
        self.step_count = 0


    def shutdown(self):
        p.disconnect()


    
    # calling this function will create a new ball 
    def get_new_ball(self, position = [-1, 0, 2]):
        # load the tennis ball
        self._ball = p.loadURDF('TableTennisEnvironmentV0/tennis_ball2/urdf/tennis_ball2.urdf', basePosition = position)
        p.changeDynamics(self._ball, -1, restitution = 0.8)
    
    # throw ball function --> velocity required : meters/sec
    def throw_ball(self, velocity_vector = [3.05, 0.8, 2.4]):  # set the force[force_x, force_y, force_z] # unit in Newton
        # p.applyExternalForce(self._ball, -1, forceObj=force_vector, posObj=[0, 0, 0], flags=p.LINK_FRAME)
        # p.stepSimulation()
        p.resetBaseVelocity(self._ball, velocity_vector)
        # self.state = self.get_state()


    # control joint velocity of the robot --> velocity is radians/sec
    def set_joint_velocity(self, target_velocities):
        for joint_index in range(6):
            # Set the joint velocity for each joint
            p.setJointMotorControl2(bodyUniqueId=self._robotic_arm,
                                    jointIndex=joint_index,
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=target_velocities[joint_index],
                                    force=500)  # You can adjust the force parameter


    def calculate_joint_angles(self, end_effector_pos, end_effector_orientation):
        """
        Calculates the joint angles for a given end-effector position and orientation.
    
        Args:
            robot_id (int): Unique ID of the robot in the PyBullet simulation.
            end_effector_pos (tuple of float): The target position of the end-effector (x, y, z).
            end_effector_orientation (tuple of float): The target orientation of the end-effector as a quaternion (x, y, z, w).
    
        Returns:
            list of float: The joint angles required to achieve the given end-effector position and orientation.
        """
        # Index of the end effector link
        end_effector_link_index = 6  # Assuming the last link is the end effector, adjust as needed
    
        # Calculate the Inverse Kinematics
        joint_angles = p.calculateInverseKinematics(
            bodyUniqueId=self._robotic_arm,
            endEffectorLinkIndex=end_effector_link_index,
            targetPosition=end_effector_pos,
            targetOrientation=end_effector_orientation,
            maxNumIterations=20000,
            residualThreshold=0.0001,
            solver = p.IK_DLS,
            # Additional parameters can be adjusted as needed, including joint limits, rest poses, etc.
        )
    
        return joint_angles

    def move_paddle_to_position(self, end_effector_pos):
        # Desired end-effector position and orientation (example)
        # end_effector_pos = (1.35, 0.0, 1.35)  # Adjust as needed
        end_effector_orientation = p.getQuaternionFromEuler([-3.5, -1.5, 0])  # No rotation
        joint_angles = self.calculate_joint_angles(end_effector_pos, end_effector_orientation)
        for i, angle in enumerate(joint_angles):
            p.setJointMotorControl2(bodyIndex=self._robotic_arm,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=angle,
                                   )
        
        # Step simulation to see the result
        
        for i in range(300):
            p.stepSimulation()
            time.sleep(1./240.)
        

    ## calculate the reward function
    def calc_reward(self):
        reward = -1
        return reward



    ## define the done condition
    def is_done(self):
        if self.step_count > 50000:
            return True
        else:
            return False



    ## step function
    def step(self, action): # action = [joint1, joint2, joint3, joint4, joint5, joint6] --> joint velocities
        self.step_count += 1
        joint_1 = action[0]
        joint_2 = action[1]
        joint_3 = action[2]
        joint_4 = action[3]
        joint_5 = action[4]
        joint_6 = action[5]
    
        joint_velocity_vector = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
        
        self.set_joint_velocity(joint_velocity_vector)


        # step
        p.stepSimulation()

        # update the state
        self.state = self.get_state()

        if self.is_done(): # setting done condition --> episode ends
            # set reward
            self.reset()
            reward = self.calc_reward()  # arbitrary reward currently, needs to be updated
            done = True
            return reward, done
        
        # if not done
        done = False
        reward = self.calc_reward() # arbitrary reward

        return reward, done