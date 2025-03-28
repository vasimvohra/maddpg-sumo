import numpy as np
import time
import random
import math
import numpy as np
import time
import random
import math
import traci
import os

os.environ["SUMO_HOME"] = "C:/Program Files (x86)/Eclipse/Sumo"  # Replace with your SUMO path


np.random.seed(1376)


class V2Vchannels:
    # Simulator of the V2V Channels

    def __init__(self):
        self.t = 0
        self.h_bs = 1.5
        self.h_ms = 1.5
        self.fc = 2  # GHz
        self.decorrelation_distance = 10
        self.shadow_std = 3

    def get_path_loss(self, position_A, position_B):
        d1 = abs(position_A[0] - position_B[0])
        d2 = abs(position_A[1] - position_B[1])
        d = math.hypot(d1, d2) + 0.001
        d_bp = 4 * (self.h_bs - 1) * (self.h_ms - 1) * self.fc * (10 ** 9) / (3 * 10 ** 8)

        def PL_Los(d):
            if d <= 3:
                return 22.7 * np.log10(3) + 41 + 20 * np.log10(self.fc / 5)
            else:
                if d < d_bp:
                    return 22.7 * np.log10(d) + 41 + 20 * np.log10(self.fc / 5)
                else:
                    return 40.0 * np.log10(d) + 9.45 - 17.3 * np.log10(self.h_bs) - 17.3 * np.log10(
                        self.h_ms) + 2.7 * np.log10(self.fc / 5)

        def PL_NLos(d_a, d_b):
            n_j = max(2.8 - 0.0024 * d_b, 1.84)
            return PL_Los(d_a) + 20 - 12.5 * n_j + 10 * n_j * np.log10(d_b) + 3 * np.log10(self.fc / 5)

        if min(d1, d2) < 7:
            PL = PL_Los(d)
        else:
            PL = min(PL_NLos(d1, d2), PL_NLos(d2, d1))
        return PL  # + self.shadow_std * np.random.normal()

    def get_shadowing(self, delta_distance, shadowing):
        return np.exp(-1 * (delta_distance / self.decorrelation_distance)) * shadowing \
            + math.sqrt(1 - np.exp(-2 * (delta_distance / self.decorrelation_distance))) * np.random.normal(0,
                                                                                                            3)  # standard dev is 3 db


class V2Ichannels:

    # Simulator of the V2I channels

    def __init__(self):
        self.h_bs = 25
        self.h_ms = 1.5
        self.Decorrelation_distance = 50
        self.BS_position = [750 / 2, 1299 / 2]  # center of the grids
        self.shadow_std = 8

    def get_path_loss(self, position_A):
        d1 = abs(position_A[0] - self.BS_position[0])
        d2 = abs(position_A[1] - self.BS_position[1])
        distance = math.hypot(d1, d2)
        return 128.1 + 37.6 * np.log10(
            math.sqrt(distance ** 2 + (self.h_bs - self.h_ms) ** 2) / 1000)  # + self.shadow_std * np.random.normal()

    def get_shadowing(self, delta_distance, shadowing):
        nVeh = len(shadowing)
        self.R = np.sqrt(0.5 * np.ones([nVeh, nVeh]) + 0.5 * np.identity(nVeh))
        return np.multiply(np.exp(-1 * (delta_distance / self.Decorrelation_distance)), shadowing) \
            + np.sqrt(1 - np.exp(-2 * (delta_distance / self.Decorrelation_distance))) * np.random.normal(0, 8, nVeh)


class Vehicle:

    # Vehicle simulator: include all the information for a vehicle
    def __init__(self, start_position, start_direction, velocity):
        self.position = start_position
        self.direction = start_direction
        self.velocity = velocity
        self.neighbors = []
        self.destinations = []


class Environ:

    def __init__(self, down_lane, up_lane, left_lane, right_lane, width, height, n_veh, size_platoon, n_RB,
                 V2I_min, BW, V2V_SIZE, Gap,
                 net_file="highway.net.xml",  # SUMO network file
                 route_file="highway.rou.xml",  # SUMO route file
                 sumo_gui=False):  # SUMO GUI option
        self.down_lanes = down_lane
        self.up_lanes = up_lane
        self.left_lanes = left_lane
        self.right_lanes = right_lane
        self.width = width
        self.height = height

        self.V2Vchannels = V2Vchannels()
        self.V2Ichannels = V2Ichannels()
        self.vehicles = []  # You might not need this directly

        self.V2V_Shadowing = []
        self.V2I_Shadowing = []
        self.delta_distance = []
        self.V2V_channels_abs = []
        self.V2I_channels_abs = []
        self.V2V_pathloss = []
        self.V2V_channels_abs = []

        self.V2I_min = V2I_min
        self.sig2_dB = -114
        self.bsAntGain = 8
        self.bsNoiseFigure = 5
        self.vehAntGain = 3
        self.vehNoiseFigure = 9
        self.sig2 = 10 ** (self.sig2_dB / 10)
        self.gap = Gap
        self.v_length = 0

        self.change_direction_prob = 0.4
        self.n_RB = n_RB
        self.n_Veh = n_veh
        self.size_platoon = size_platoon
        self.time_fast = 0.001
        self.time_slow = 0.1  # update slow fading/vehicle position every 100 ms
        self.bandwidth = BW  # bandwidth per RB, 180,000 MHz
        self.V2V_demand_size = V2V_SIZE  # V2V payload: 4000 Bytes every 100 ms

        self.Interference_all = np.zeros(int(self.n_Veh / self.size_platoon)) + self.sig2

        self.net_file = net_file  # SUMO network file
        self.route_file = route_file  # SUMO route file
        self.sumo_gui = sumo_gui

    def start_sumo(self):
        if 'SUMO_HOME' not in os.environ:
            raise EnvironmentError("Please set the SUMO_HOME environment variable")

        sumo_cmd = [
            os.path.join(os.environ['SUMO_HOME'], "bin", "sumo-gui" if self.sumo_gui else "sumo"),
            "-c", "my_sumo_config.sumocfg",  # Use a SUMO config file
            "--start",
            "--step-length", "0.1",
            "--delay", "100",
            "--duration-log.disable", "true"
        ]

        traci.start(sumo_cmd)

    def close_sumo(self):
        if traci.isLoaded():
            traci.close()

    # =================================================================
    #  REMOVED: add_new_platoon, add_new_platoon_by_number
    #  SUMO will handle vehicle creation
    # =================================================================

    # =================================================================
    #   REMOVED: renew_positions
    #   SUMO will handle position updates
    # =================================================================

    def renew_channel(self, number_vehicle, size_platoon):
        """ Renew slow fading channel """

        # Get vehicle positions from SUMO using TraCI
        vehicle_positions = []
        vehicle_ids = traci.vehicle.getIDList()  # Get IDs of vehicles in SUMO
        for vid in vehicle_ids:
            vehicle_positions.append(traci.vehicle.getPosition(vid))

        self.V2V_pathloss = np.zeros((len(vehicle_positions), len(vehicle_positions))) + 50 * np.identity(
            len(vehicle_positions))
        self.V2I_pathloss = np.zeros((len(vehicle_positions)))

        self.V2V_channels_abs = np.zeros((len(vehicle_positions), len(vehicle_positions)))
        self.V2I_channels_abs = np.zeros((len(vehicle_positions)))
        for i in range(len(vehicle_positions)):
            for j in range(i + 1, len(vehicle_positions)):
                # delta_distance needs to be calculated using traci.vehicle.getSpeed()
                speed_i = traci.vehicle.getSpeed(vehicle_ids[i])
                speed_j = traci.vehicle.getSpeed(vehicle_ids[j])
                delta_distance_i = speed_i * self.time_slow
                delta_distance_j = speed_j * self.time_slow

                self.V2V_Shadowing[j][i] = self.V2V_Shadowing[i][j] = \
                    self.V2Vchannels.get_shadowing(delta_distance_i + delta_distance_j,
                                                   self.V2V_Shadowing[i][j])
                self.V2V_pathloss[j, i] = self.V2V_pathloss[i][j] = \
                    self.V2Vchannels.get_path_loss(vehicle_positions[i], vehicle_positions[j])

        self.V2V_channels_abs = self.V2V_pathloss + self.V2V_Shadowing

        # delta_distance needs to be calculated using traci.vehicle.getSpeed()
        delta_distance_v2i = []
        for vid in vehicle_ids:
            speed = traci.vehicle.getSpeed(vid)
            delta_distance_v2i.append(speed * self.time_slow)

        self.V2I_Shadowing = self.V2Ichannels.get_shadowing(delta_distance_v2i, self.V2I_Shadowing)
        for i in range(len(vehicle_positions)):
            self.V2I_pathloss[i] = self.V2Ichannels.get_path_loss(vehicle_positions[i])

        self.V2I_channels_abs = self.V2I_pathloss + self.V2I_Shadowing

    def renew_channels_fastfading(self):
        """ Renew fast fading channel """
        V2V_channels_with_fastfading = np.repeat(self.V2V_channels_abs[:, :, np.newaxis], self.n_RB, axis=2)
        self.V2V_channels_with_fastfading = V2V_channels_with_fastfading - 20 * np.log10(
            np.abs(np.random.normal(0, 1, V2V_channels_with_fastfading.shape) +
                   1j * np.random.normal(0, 1, V2V_channels_with_fastfading.shape)) / math.sqrt(2))

        V2I_channels_with_fastfading = np.repeat(self.V2I_channels_abs[:, np.newaxis], self.n_RB, axis=1)
        self.V2I_channels_with_fastfading = V2I_channels_with_fastfading - 20 * np.log10(
            np.abs(np.random.normal(0, 1, V2I_channels_with_fastfading.shape) +
                   1j * np.random.normal(0, 1, V2I_channels_with_fastfading.shape)) / math.sqrt(2))

    def Revenue_function(self, quantity, threshold):
        # G function definition in the paper
        revenue = 0
        if quantity >= threshold:
            revenue = 1
        else:
            revenue = 0
        return revenue

    def Compute_Performance_Reward_Train(self, platoons_actions):
        """
        Calculates performance metrics, reward components, and Age of Information (AoI) for training.

        Args:
            platoons_actions: A numpy array containing actions taken by each platoon.
                              It is assumed to have the shape (num_platoons, 3), where:
                              - platoons_actions[:, 0]: Sub-channel selection.
                              - platoons_actions[:, 1]: Platoon communication decision
                                                        (0 for Inter-platoon, 1 for Intra-platoon).
                              - platoons_actions[:, 2]: Power selection.

        Returns:
            platoons_AoI:          Age of Information for each platoon.
            interplatoon_rate:     Data rates for inter-platoon (V2I) communication.
            intraplatoon_rate:     Data rates for intra-platoon (V2V) communication.
            V2V_demand:            Remaining data demand for V2V communication.
            reward_elements:       Individual reward components for each platoon.
        """

        num_platoons = int(self.n_Veh / self.size_platoon)  # Number of platoons

        # 1. Extract Actions
        sub_channel_selection = platoons_actions[:, 0].astype(int).reshape(num_platoons, 1)
        platoon_communication_decision = platoons_actions[:, 1].astype(int).reshape(num_platoons, 1)
        power_selection = platoons_actions[:, 2].reshape(num_platoons, 1)

        # 2. Initialize Interference and Signal Arrays
        self.platoon_V2I_Interference = np.zeros(num_platoons)  # V2I interference
        self.platoon_V2I_Signal = np.zeros(num_platoons)  # V2I signal strength
        self.platoon_V2V_Interference = np.zeros((num_platoons, self.size_platoon - 1))  # V2V interference
        self.platoon_V2V_Signal = np.zeros((num_platoons, self.size_platoon - 1))  # V2V signal strength

        # 3. Calculate Interference
        for resource_block_index in range(self.n_RB):
            # Find platoons using the current resource block
            using_resource_block = np.argwhere(sub_channel_selection == resource_block_index)

            for j_index in range(len(using_resource_block)):
                for k_index in range(len(using_resource_block)):
                    j = using_resource_block[j_index, 0]  # Platoon index j
                    k = using_resource_block[k_index, 0]  # Platoon index k

                    # Inter-platoon interference (V2I)
                    if j != k and platoon_communication_decision[j, 0] == 0:
                        self.platoon_V2I_Interference[j] += \
                            10 ** ((power_selection[k, 0] - self.V2I_channels_with_fastfading[
                                k * self.size_platoon, resource_block_index] +
                                    self.vehAntGain + self.bsAntGain - self.bsNoiseFigure) / 10)

                    # Intra-platoon interference (V2V)
                    if j != k and platoon_communication_decision[j, 0] == 1:
                        for neighbor_link_index in range(self.size_platoon - 1):
                            self.platoon_V2V_Interference[j, neighbor_link_index] += \
                                10 ** ((power_selection[k, 0] - self.V2V_channels_with_fastfading[
                                    k * self.size_platoon, j * self.size_platoon + (
                                                neighbor_link_index + 1), resource_block_index] +
                                        2 * self.vehAntGain - self.vehNoiseFigure) / 10)

        # 4. Calculate Signal Strength
        for resource_block_index in range(self.n_RB):
            # Find platoons using the current resource block
            using_resource_block = np.argwhere(sub_channel_selection == resource_block_index)

            for j_index in range(len(using_resource_block)):
                j = using_resource_block[j_index, 0]  # Platoon index j

                # Inter-platoon signal (V2I)
                if platoon_communication_decision[j, 0] == 0:
                    self.platoon_V2I_Signal[j] = \
                        10 ** ((power_selection[j, 0] - self.V2I_channels_with_fastfading[
                            j * self.size_platoon, resource_block_index] +
                                self.vehAntGain + self.bsAntGain - self.bsNoiseFigure) / 10)

                # Intra-platoon signal (V2V)
                elif platoon_communication_decision[j, 0] == 1:
                    for neighbor_link_index in range(self.size_platoon - 1):
                        self.platoon_V2V_Signal[j, neighbor_link_index] = \
                            10 ** ((power_selection[j, 0] - self.V2V_channels_with_fastfading[
                                j * self.size_platoon, j * self.size_platoon + (
                                            neighbor_link_index + 1), resource_block_index] +
                                    2 * self.vehAntGain - self.vehNoiseFigure) / 10)

        # 5. Calculate Communication Rates
        V2I_Rate = np.log2(1 + np.divide(self.platoon_V2I_Signal, (self.platoon_V2I_Interference + self.sig2)))
        V2V_Rate = np.log2(1 + np.divide(self.platoon_V2V_Signal, (self.platoon_V2V_Interference + self.sig2)))

        self.interplatoon_rate = V2I_Rate * self.time_fast * self.bandwidth
        self.intraplatoon_rate = np.min(V2V_Rate, axis=1) * self.time_fast * self.bandwidth

        # 6. Calculate Age of Information (AoI)
        platoons_AoI = self.Age_of_Information(self.interplatoon_rate)

        # 7. Update Data Demand
        self.V2V_demand -= self.intraplatoon_rate
        self.V2V_demand[self.V2V_demand <= 0] = 0

        # 8. Update Active Links and Calculate Reward Elements
        self.individual_time_limit -= self.time_fast
        self.active_links[np.multiply(self.active_links, self.V2V_demand <= 0)] = 0
        reward_elements = self.intraplatoon_rate / 10000
        reward_elements[self.V2V_demand <= 0] = 1

        return platoons_AoI, self.interplatoon_rate, self.intraplatoon_rate, self.V2V_demand, reward_elements
    def Age_of_Information(self, V2I_rate):
        # computing the platoons age of information

        for i in range(int(self.n_Veh / self.size_platoon)):
            if V2I_rate[i] >= self.V2I_min:
                self.AoI[i] = 1
            else:
                self.AoI[i] += 1
                if self.AoI[i] >= (self.time_slow / self.time_fast):
                    self.AoI[i] = (self.time_slow / self.time_fast)
        return self.AoI

    def act_for_training(self, actions):

        per_user_reward = np.zeros(int(self.n_Veh / self.size_platoon))
        action_temp = actions.copy()
        platoon_AoI, C_rate, V_rate, Demand, elements = self.Compute_Performance_Reward_Train(action_temp)
        V2V_success = 1 - np.sum(self.active_links) / (int(self.n_Veh / self.size_platoon))  # V2V success rates

        for i in range(int(self.n_Veh / self.size_platoon)):
            per_user_reward[i] = (-4.95) * (Demand[i] / self.V2V_demand_size) - \
                                 platoon_AoI[i] / 20 + (0.05) * self.Revenue_function(C_rate[i], self.V2I_min) - \
                                 0.5 * math.log(action_temp[i, 2], 5)
        global_reward = np.mean(per_user_reward)
        return per_user_reward, global_reward, platoon_AoI, C_rate, V_rate, Demand, V2V_success

    def act_for_testing(self, actions):

        action_temp = actions.copy()
        platoon_AoI, C_rate, V_rate, Demand, elements = self.Compute_Performance_Reward_Train(action_temp)
        V2V_success = 1 - np.sum(self.active_links) / (int(self.n_Veh / self.size_platoon))  # V2V success rates

        return platoon_AoI, C_rate, V_rate, Demand, elements, V2V_success

    def Compute_Interference(self, platoons_actions):

        sub_selection = platoons_actions[:, 0].copy().astype('int').reshape(int(self.n_Veh / self.size_platoon), 1)
        platoon_decision = platoons_actions[:, 1].copy().astype('int').reshape(int(self.n_Veh / self.size_platoon), 1)
        power_selection = platoons_actions[:, 2].copy().reshape(int(self.n_Veh / self.size_platoon), 1)
        # ------------ Compute Interference --------------------
        V2I_Interference_state = np.zeros(int(self.n_Veh / self.size_platoon)) + self.sig2
        V2V_Interference_state = np.zeros([int(self.n_Veh / self.size_platoon), self.size_platoon - 1]) + self.sig2

        for i in range(self.n_RB):
            indexes = np.argwhere(sub_selection == i)
            for j in range(len(indexes)):
                for k in range(len(indexes)):
                    if indexes[j, 0] != indexes[k, 0] and platoon_decision[indexes[j, 0], 0] == 0:
                        # if not self.active_links[indexes[k, 0]] and platoon_decision[indexes[k, 0], 0] == 1:
                        #     continue
                        V2I_Interference_state[indexes[j, 0]] += \
                            10 ** ((power_selection[indexes[k, 0], 0] - self.V2I_channels_with_fastfading[
                                indexes[k, 0] * self.size_platoon, i] +
                                    self.vehAntGain + self.bsAntGain - self.bsNoiseFigure) / 10)
                    if indexes[j, 0] != indexes[k, 0] and platoon_decision[indexes[j, 0], 0] == 1:
                        # if not self.active_links[indexes[k, 0]] and platoon_decision[indexes[k, 0], 0] == 1:
                        #     continue
                        for l in range(self.size_platoon - 1):
                            V2V_Interference_state[indexes[j, 0], l] += \
                                10 ** ((power_selection[indexes[k, 0], 0] - self.V2V_channels_with_fastfading[
                                    indexes[k, 0] * self.size_platoon, indexes[j, 0] * self.size_platoon + (l + 1), i] +
                                        2 * self.vehAntGain - self.vehNoiseFigure) / 10)

        self.V2I_Interference_all = 10 * np.log10(V2I_Interference_state)
        self.V2V_Interference_all = 10 * np.log10(V2V_Interference_state)
        for i in range(int(self.n_Veh / self.size_platoon)):
            if platoon_decision[i, 0] == 0:
                self.Interference_all[i] = self.V2I_Interference_all[i]
            else:
                self.Interference_all[i] = np.max(self.V2V_Interference_all[i, :])

    def new_random_game(self, n_Veh=0):

        # make a new game
        self.vehicles = []
        print(self.n_Veh)
        print
        if n_Veh > 0:
            self.n_Veh = n_Veh
       # self.add_new_platoon_by_number(int(self.n_Veh), self.size_platoon)
        self.renew_channel(int(self.n_Veh), self.size_platoon)
        self.renew_channels_fastfading()

        self.V2V_demand = self.V2V_demand_size * np.ones(int(self.n_Veh / self.size_platoon), dtype=np.float16)
        self.individual_time_limit = self.time_slow * np.ones(int(self.n_Veh / self.size_platoon), dtype=np.float16)
        self.active_links = np.ones((int(self.n_Veh / self.size_platoon)), dtype='bool')
        self.AoI = np.ones(int(self.n_Veh / self.size_platoon), dtype=np.float16) * 100
