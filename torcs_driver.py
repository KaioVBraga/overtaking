# torcs_driver_fuzzy.py
# Refatoração do TorcsDriver com análise fuzzy e correções de marcha/aceleração
# Correções críticas para execução adequada de curvas e resolução do travamento em 54 km/h
# Ajustes para melhorar curvatura e aumentar velocidade máxima

import numpy as np
import time
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from log import logger


class TorcsDriver:
    def __init__(self):
        # Estado do controlador
        self.last_steer = 0.0
        self.last_speed = 0.0
        self.launch_phase = True
        self.launch_timer = 0
        self.fuzzy_valid = False
        self.last_valid_state_time = time.time()
        self.danger_history = []
        self.curve_type = "straight"
        self.curve_start_time = 0
        self.oscillation_counter = 0
        self.last_direction = 0
        self.prev_path_error = 0.0
        self.last_gear_change_time = 0.0
        self.last_accel = 0.5
        self.curve_exit_timer = 0  # Para acelerar após sair de curvas
        self.last_curve_type = "straight"

        # thresholds for desired gear by speed (km/h): index = gear-1
        # Ajustados para permitir subida de marcha mais cedo e velocidades mais altas
        self.gear_speed_thresholds = [0.0, 22.0, 50.0, 85.0, 130.0, 180.0]

        # Configura os sistemas fuzzy
        self._setup_fuzzy_systems()

    def _setup_fuzzy_systems(self):
        """Cria controlador fuzzy de sensores e assistente de direção/aceleração."""
        try:
            left = ctrl.Antecedent(np.arange(0, 201, 1), 'left')
            center = ctrl.Antecedent(np.arange(0, 201, 1), 'center')
            right = ctrl.Antecedent(np.arange(0, 201, 1), 'right')
            speed = ctrl.Antecedent(np.arange(0, 301, 1), 'speed')

            curvature = ctrl.Consequent(np.arange(-0.8, 0.81, 0.01), 'curvature')
            offset = ctrl.Consequent(np.arange(-1.5, 1.51, 0.01), 'offset')
            danger = ctrl.Consequent(np.arange(0.0, 1.01, 0.01), 'danger')

            for var in (left, center, right):
                var['very_close'] = fuzz.trimf(var.universe, [0, 0, 20])
                var['close'] = fuzz.trimf(var.universe, [10, 30, 60])
                var['medium'] = fuzz.trimf(var.universe, [40, 80, 130])
                var['far'] = fuzz.trimf(var.universe, [110, 150, 190])
                var['very_far'] = fuzz.trimf(var.universe, [170, 200, 200])

            speed['slow'] = fuzz.trimf(speed.universe, [0, 0, 60])
            speed['medium'] = fuzz.trimf(speed.universe, [40, 110, 180])
            speed['fast'] = fuzz.trimf(speed.universe, [140, 220, 300])

            curvature['sharp_left'] = fuzz.trimf(curvature.universe, [-0.8, -0.7, -0.4])
            curvature['left'] = fuzz.trimf(curvature.universe, [-0.5, -0.3, -0.05])
            curvature['straight'] = fuzz.trimf(curvature.universe, [-0.08, 0.0, 0.08])
            curvature['right'] = fuzz.trimf(curvature.universe, [0.05, 0.3, 0.5])
            curvature['sharp_right'] = fuzz.trimf(curvature.universe, [0.4, 0.7, 0.8])

            offset['far_left'] = fuzz.trimf(offset.universe, [-1.5, -1.5, -0.9])
            offset['left'] = fuzz.trimf(offset.universe, [-1.1, -0.6, -0.2])
            offset['center'] = fuzz.trimf(offset.universe, [-0.3, 0.0, 0.3])
            offset['right'] = fuzz.trimf(offset.universe, [0.2, 0.6, 1.1])
            offset['far_right'] = fuzz.trimf(offset.universe, [0.9, 1.5, 1.5])

            danger['low'] = fuzz.trimf(danger.universe, [0.0, 0.0, 0.25])
            danger['medium'] = fuzz.trimf(danger.universe, [0.15, 0.45, 0.75])
            danger['high'] = fuzz.trimf(danger.universe, [0.6, 0.85, 1.0])

            rules = []

            # Danger rules - APRIMORADAS PARA DETECÇÃO MAIS PRECISA
            rules.append(ctrl.Rule(center['very_close'], danger['high']))
            rules.append(ctrl.Rule(center['close'], danger['medium']))
            rules.append(ctrl.Rule(center['medium'] & speed['fast'], danger['medium']))
            rules.append(ctrl.Rule(center['medium'] & speed['medium'], danger['low']))
            rules.append(ctrl.Rule(center['far'], danger['low']))
            rules.append(ctrl.Rule(curvature['sharp_left'] | curvature['sharp_right'], danger['high']))
            rules.append(ctrl.Rule(curvature['left'] | curvature['right'], danger['medium']))
            rules.append(ctrl.Rule(curvature['straight'], danger['low']))
            rules.append(ctrl.Rule((center['medium'] | center['close']) & (curvature['left'] | curvature['right']) & speed['fast'], danger['high']))
            rules.append(ctrl.Rule((center['medium'] | center['close']) & (curvature['left'] | curvature['right']) & speed['medium'], danger['medium']))
            rules.append(ctrl.Rule((center['medium'] | center['close']) & (curvature['left'] | curvature['right']) & speed['slow'], danger['low']))
            rules.append(ctrl.Rule((left['very_far'] & left['close']) | (right['very_far'] & right['close']), danger['medium']))
            rules.append(ctrl.Rule((curvature['sharp_left'] | curvature['sharp_right']) & (center['far'] | center['very_far']), danger['medium']))
            
            # Regras adicionais para melhor antecipação
            rules.append(ctrl.Rule((left['very_far'] & left['medium'] & left['close']) | 
                                  (right['very_far'] & right['medium'] & right['close']), danger['medium']))
            rules.append(ctrl.Rule((left['far'] & left['close']) | (right['far'] & right['close']), danger['high']))

            # Curvature rules
            rules.append(ctrl.Rule(left['very_close'] & right['far'], curvature['right']))
            rules.append(ctrl.Rule(left['close'] & right['far'], curvature['right']))
            rules.append(ctrl.Rule(right['very_close'] & left['far'], curvature['sharp_left']))
            rules.append(ctrl.Rule(right['close'] & left['far'], curvature['left']))
            rules.append(ctrl.Rule(left['medium'] & right['medium'], curvature['straight']))
            rules.append(ctrl.Rule(left['far'] & right['far'] & center['far'], curvature['straight']))
            rules.append(ctrl.Rule((left['very_far'] & left['close']) & (right['far'] | right['very_far']), curvature['right']))
            rules.append(ctrl.Rule((right['very_far'] & right['close']) & (left['far'] | left['very_far']), curvature['left']))
            
            # Regras adicionais para melhor detecção de curvas longas
            rules.append(ctrl.Rule((left['very_far'] & left['medium'] & left['close']) & 
                                  (right['far'] | right['very_far']), curvature['right']))
            rules.append(ctrl.Rule((right['very_far'] & right['medium'] & right['close']) & 
                                  (left['far'] | left['very_far']), curvature['left']))

            # Offset rules
            rules.append(ctrl.Rule(curvature['sharp_left'], offset['far_left']))
            rules.append(ctrl.Rule(curvature['left'], offset['left']))
            rules.append(ctrl.Rule(curvature['straight'], offset['center']))
            rules.append(ctrl.Rule(curvature['right'], offset['right']))
            rules.append(ctrl.Rule(curvature['sharp_right'], offset['far_right']))
            rules.append(ctrl.Rule(left['far'] & right['close'], offset['right']))
            rules.append(ctrl.Rule(right['far'] & left['close'], offset['left']))
            
            # Regras adicionais para melhor centralização
            rules.append(ctrl.Rule(center['very_far'] & curvature['straight'], offset['center']))
            rules.append(ctrl.Rule((curvature['left'] | curvature['right']) & danger['medium'], offset['center']))

            sensor_ctrl = ctrl.ControlSystem(rules)
            self.sensor_sim = ctrl.ControlSystemSimulation(sensor_ctrl)

            # Drive assist fuzzy
            track_pos = ctrl.Antecedent(np.arange(-2, 2.01, 0.01), 'track_pos')
            angle = ctrl.Antecedent(np.arange(-1.6, 1.61, 0.01), 'angle')
            speed_x = ctrl.Antecedent(np.arange(0, 301, 1), 'speed_x')
            steer = ctrl.Consequent(np.arange(-1, 1.01, 0.01), 'steer')
            accel = ctrl.Consequent(np.arange(0.0, 1.01, 0.01), 'accel')

            track_pos['left'] = fuzz.trimf(track_pos.universe, [0.3, 1.0, 1.8])
            track_pos['center'] = fuzz.trimf(track_pos.universe, [-0.4, 0.0, 0.4])
            track_pos['right'] = fuzz.trimf(track_pos.universe, [-1.8, -1.0, -0.3])

            angle['left'] = fuzz.trimf(angle.universe, [-1.6, -0.6, -0.1])
            angle['straight'] = fuzz.trimf(angle.universe, [-0.2, 0.0, 0.2])
            angle['right'] = fuzz.trimf(angle.universe, [0.1, 0.6, 1.6])

            speed_x['slow'] = fuzz.trimf(speed_x.universe, [0, 0, 60])
            speed_x['medium'] = fuzz.trimf(speed_x.universe, [40, 110, 180])
            speed_x['fast'] = fuzz.trimf(speed_x.universe, [140, 220, 300])

            steer['hard_left'] = fuzz.trimf(steer.universe, [0.5, 0.8, 1.0])
            steer['left'] = fuzz.trimf(steer.universe, [0.2, 0.5, 0.75])
            steer['zero'] = fuzz.trimf(steer.universe, [-0.05, 0.0, 0.05])
            steer['right'] = fuzz.trimf(steer.universe, [-0.75, -0.5, -0.2])
            steer['hard_right'] = fuzz.trimf(steer.universe, [-1.0, -0.8, -0.5])

            accel['low'] = fuzz.trimf(accel.universe, [0.0, 0.0, 0.4])
            accel['med'] = fuzz.trimf(accel.universe, [0.3, 0.55, 0.75])
            accel['high'] = fuzz.trimf(accel.universe, [0.6, 0.85, 1.0])

            drive_rules = [
                ctrl.Rule(angle['left'], steer['left']),
                ctrl.Rule(angle['right'], steer['right']),
                ctrl.Rule(track_pos['center'] & angle['straight'], steer['zero']),
                ctrl.Rule(angle['straight'] & speed_x['fast'], accel['high']),
                ctrl.Rule(angle['straight'] & speed_x['medium'], accel['med']),
                ctrl.Rule(angle['straight'] & speed_x['slow'], accel['high']),
                ctrl.Rule(~angle['straight'] & speed_x['fast'], accel['low']),
                ctrl.Rule(~angle['straight'] & speed_x['medium'], accel['med']),
            ]

            drive_ctrl = ctrl.ControlSystem(drive_rules)
            self.drive_sim = ctrl.ControlSystemSimulation(drive_ctrl)

            self.fuzzy_valid = True
            logger.info("Fuzzy systems initialized")
        except Exception as e:
            self.fuzzy_valid = False
            logger.critical(f"Fuzzy init failed: {e}")

    def _validate_car_state(self, car_state):
        """Valida formato mínimo do estado do carro."""
        if not car_state or not isinstance(car_state, dict):
            return False
        keys = ['speedX', 'trackPos', 'angle', 'track', 'rpm', 'gear']
        for k in keys:
            if k not in car_state:
                return False
        if not isinstance(car_state['track'], list) or len(car_state['track']) != 19:
            return False
        return True

    def _analyze_track_sensors(self, track_sensor, speed_x):
        """Retorna (curvature, desired_offset, danger) usando fuzzy sobre left/center/right."""
        if not track_sensor or len(track_sensor) != 19:
            return 0.0, 0.0, 0.0

        s = []
        for v in track_sensor:
            try:
                fv = float(v)
            except Exception:
                fv = 200.0
            s.append(max(0.1, fv))

        left_avg = float(np.mean(s[:9]))
        center_val = float(s[9])
        right_avg = float(np.mean(s[10:]))
        speed_x = float(max(0.0, speed_x))

        try:
            self.sensor_sim.input['left'] = left_avg
            self.sensor_sim.input['center'] = center_val
            self.sensor_sim.input['right'] = right_avg
            self.sensor_sim.input['speed'] = speed_x
            self.sensor_sim.compute()

            cur = float(self.sensor_sim.output.get('curvature', 0.0))
            off = float(self.sensor_sim.output.get('offset', 0.0))
            dang = float(self.sensor_sim.output.get('danger', 0.0))

            cur = np.clip(cur, -0.8, 0.8)
            off = np.clip(off, -1.5, 1.5)
            dang = np.clip(dang, 0.0, 1.0)

            self.danger_history.append(dang)
            if len(self.danger_history) > 6:
                self.danger_history.pop(0)

            # Atualização do tipo de curva com histerese para evitar mudanças rápidas
            if dang > 0.75 and abs(cur) > 0.55:
                self.curve_type = "corner"
                self.curve_start_time = time.time()
            elif dang > 0.45 and abs(cur) > 0.25:
                self.curve_type = "long_curve"
                self.curve_start_time = time.time()
            else:
                self.curve_type = "straight"

            return cur, off, dang
        except Exception as e:
            logger.warning(f"Sensor fuzzy falhou: {e}")
            denom = max(0.0001, left_avg + right_avg)
            cur = (right_avg - left_avg) / denom
            cur = np.clip(cur, -0.8, 0.8)
            off = np.clip(cur * 0.9, -1.5, 1.5)

            # Cálculo mais robusto do danger
            track_widths = [s[i] + s[18 - i] for i in range(9)]
            narrowing_rate = track_widths[0] - track_widths[4]
            
            # Análise da consistência da curva
            width_changes = [track_widths[i] - track_widths[i+1] for i in range(8)]
            consistent_narrowing = all(change > 1.0 for change in width_changes[:3])
            
            dang = 0.0
            if narrowing_rate > 4.0 and consistent_narrowing:
                dang = min(0.95, narrowing_rate * 0.22)
            if abs(cur) > 0.2:
                dang = max(dang, min(abs(cur) * 0.9, 0.9))
            
            # Ajuste baseado na velocidade
            if speed_x > 120:
                dang = min(1.0, dang * 1.3)
            elif speed_x < 30:
                dang = max(0.0, dang * 0.6)
            
            dang = np.clip(dang, 0.0, 1.0)

            # Atualização do tipo de curva com histerese
            if dang > 0.75 and abs(cur) > 0.55:
                self.curve_type = "corner"
                self.curve_start_time = time.time()
            elif dang > 0.45 and abs(cur) > 0.25:
                self.curve_type = "long_curve"
                self.curve_start_time = time.time()
            else:
                self.curve_type = "straight"

            self.danger_history.append(dang)
            if len(self.danger_history) > 6:
                self.danger_history.pop(0)

            return cur, off, dang

    def _get_steering(self, car_state, fuzzy_drive_output, speed_kmh, curvature, desired_offset, danger):
        """Calcula o steer combinando saída fuzzy, centralização e predição de curva."""
        track_pos = float(car_state.get('trackPos', 0.0))
        angle = float(car_state.get('angle', 0.0))

        base_steer = fuzzy_drive_output.get('steer', 0.0)
        path_error = track_pos - desired_offset

        # Detecção de oscilação com histerese
        current_direction = np.sign(self.last_steer) if abs(self.last_steer) > 0.1 else 0
        direction_change = (current_direction != 0) and (current_direction != self.last_direction)
        significant_change = abs(path_error - self.prev_path_error) > 0.25

        if direction_change and significant_change:
            self.oscillation_counter += 1.2  # Aumento mais agressivo para detectar oscilação real
        else:
            self.oscillation_counter = max(0, self.oscillation_counter - 0.4)
        self.last_direction = current_direction
        self.prev_path_error = path_error

        # Configuração de ganhos com maior adaptação às curvas
        base_gain = 0.55
        speed_factor = 1.0 - min(speed_kmh / 260.0, 0.92)

        # AJUSTE CRÍTICO: AUMENTO DOS GANHOS PARA MELHORAR CURVATURA
        # Ajuste específico para tipo de curva com maior granularidade
        if self.curve_type == "corner":
            # Curvas fechadas requerem precisão máxima
            curve_factor = 1.0 - min(abs(curvature), 0.88)
            danger_factor = 1.0 - min(danger * 0.75, 0.82)
            base_gain = 0.75  # AUMENTADO DE 0.68 PARA 0.75 para melhor curvatura
        elif self.curve_type == "long_curve":
            # Curvas longas precisam de suavidade
            curve_factor = 1.0 - min(abs(curvature) * 0.75, 0.75)
            danger_factor = 1.0 - min(danger * 0.55, 0.75)
            base_gain = 0.70  # AUMENTADO DE 0.63 PARA 0.70 para melhor curvatura
        else:
            # Retas - foco em estabilidade
            curve_factor = 1.0 - min(abs(curvature) * 0.15, 0.35)
            danger_factor = 1.0
            base_gain = 0.42  # Ganho reduzido para minimizar zig-zag
            
            # Redução adicional se próximo do centro
            if abs(track_pos) < 0.25:
                base_gain *= 0.65

        # Ganho final com todos os fatores
        centering_gain = base_gain * speed_factor * curve_factor * danger_factor
        centering_gain = max(0.15, centering_gain)  # AUMENTADO DE 0.12 PARA 0.15

        # CORREÇÃO CRÍTICA: DIREÇÃO CORRETA DO CENTERING FORCE
        # Se path_error > 0 (estamos à direita do caminho desejado), 
        # queremos virar à esquerda (steer positivo)
        centering = path_error * centering_gain

        # CORREÇÃO: DIREÇÃO CORRETA DA CORREÇÃO DE ÂNGULO
        angle_correction = 0.0
        if abs(angle) > 0.06:
            # Se angle > 0 (carro apontando para esquerda), queremos virar para direita (steer negativo)
            angle_correction = -np.sign(angle) * min(abs(angle) * 0.45, 0.25)  # AUMENTADO DE 0.22 PARA 0.25
            
            # Reduzir correção de ângulo em retas
            if self.curve_type == "straight" and abs(angle) < 0.25:
                angle_correction *= 0.55

        # CORREÇÃO: DIREÇÃO CORRETA DO STEERING PREDITIVO
        predictive = 0.0
        if self.curve_type != "straight" and abs(curvature) > 0.18:
            # Se curvature > 0 (curva para direita), queremos steer negativo (virar para direita)
            predictive = -curvature * 0.9  # AUMENTADO DE 0.8 PARA 0.9 para melhor curvatura
            
            # Ajuste especial para curvas muito fechadas
            if danger > 0.8 and abs(curvature) > 0.6:
                predictive = -curvature * 0.75  # AUMENTADO DE 0.65 PARA 0.75

        # Combine components com pesos adaptativos baseados no danger
        if danger > 0.7:
            weights = [0.1, 0.4, 0.5]  # AUMENTADO O PESO DO PREDITIVO DE 0.4 PARA 0.5
        elif danger > 0.4:
            weights = [0.3, 0.3, 0.4]  # AUMENTADO O PESO DO PREDITIVO DE 0.3 PARA 0.4
        else:
            weights = [0.55, 0.25, 0.2]
        
        raw = (base_steer * weights[0] + 
               centering * weights[1] + 
               predictive * weights[2] +
               angle_correction)

        # Zona morta para reduzir oscilações menores
        dead_zone = 0.04
        if abs(raw) < dead_zone:
            raw = 0.0
        elif raw > 0:
            raw -= dead_zone
        else:
            raw += dead_zone

        # Bloqueio dinâmico de steering
        steer_lock = 0.98  # AUMENTADO DE 0.95 PARA 0.98 para permitir mais steering
        if self.curve_type == "corner":  # Curvas fechadas
            steer_lock = max(0.5, 1.08 - (speed_kmh / 190.0))  # AUMENTADO DE 0.45 PARA 0.5
        elif self.curve_type == "long_curve":  # Curvas longas
            steer_lock = max(0.7, 1.18 - (speed_kmh / 210.0))  # AUMENTADO DE 0.65 PARA 0.7
        else:  # Retas
            steer_lock = max(0.5, 0.98 - (speed_kmh / 190.0))  # AUMENTADO DE 0.45 PARA 0.5
            
        raw = np.clip(raw, -steer_lock, steer_lock)

        # Suavização adaptativa
        alpha = 0.1 if speed_kmh > 110 else 0.3  # LEVE REDUÇÃO PARA MAIOR REATIVIDADE
        if self.curve_type == "straight":
            alpha = max(alpha, 0.92)
            if abs(track_pos) < 0.35:
                alpha = max(alpha, 0.96)
                
        if self.oscillation_counter > 3.5:
            alpha = max(alpha, 0.88 + (self.oscillation_counter - 3.5) * 0.04)
            
        final = alpha * raw + (1 - alpha) * self.last_steer
        final = np.clip(final, -1.0, 1.0)
        self.last_steer = final
        return final

    def _get_acceleration_and_brake(self, car_state, fuzzy_drive_output, speed_kmh, danger, corner_distance):
        """Calcula aceleração/freio antecipando curvas e favorecendo aceleração em retas seguras."""
        accel = fuzzy_drive_output.get('accel', 0.5) if self.fuzzy_valid else 0.4
        brake = 0.0

        # Launch control com tempo ajustado
        if self.launch_phase:
            if speed_kmh < 45 or self.launch_timer < 55:
                self.launch_timer += 1
                return 1.0, 0.0
            else:
                self.launch_phase = False

        # CRÍTICO: CORREÇÃO PARA O TRAVAMENTO EM 54 KM/H
        # Ajuste especial para acelerar após subir de marcha
        if self.last_accel < 0.9 and speed_kmh > 40 and self.curve_type == "straight":
            # Se a aceleração está baixa mas estamos em reta, forçar aceleração
            accel = max(accel, 0.85)
        
        # Em retas seguras, priorizar aceleração gradual para permitir subir marchas
        if self.curve_type == 'straight' and danger < 0.2:
            # AJUSTE CRÍTICO: AUMENTO DA VELOCIDADE MÁXIMA
            # Ajuste mais agressivo para evitar travamento em 54 km/h
            # MUDANÇA: 0.92 - (speed_kmh / 500.0) PARA 0.95 - (speed_kmh / 1000.0)
            # Isso permite aceleração mais forte em altas velocidades
            accel = max(accel, min(1.0, 0.95 - (speed_kmh / 1000.0)))
            
            # Aceleração extra após sair de curvas
            if self.last_curve_type != "straight" and time.time() - self.curve_start_time < 2.0:
                accel = min(1.0, accel + 0.15)

        # Cálculo robusto de distância de frenagem
        if speed_kmh > 35:
            # Fórmula mais realista de distância de frenagem
            base_braking_distance = (speed_kmh * speed_kmh) / 165.0
            
            # Ajustar com base na severidade da curva
            braking_factor = 1.0
            if danger > 0.35:
                braking_factor = 1.0 + max(0.0, (danger - 0.35) * 2.8)
            
            required_braking_distance = base_braking_distance * braking_factor

            if corner_distance is None:
                # Estimativa alternativa usando o histórico de danger
                if len(self.danger_history) > 2:
                    danger_increase = self.danger_history[-1] - self.danger_history[-3]
                    if danger_increase > 0.1:
                        corner_distance = min(45.0, 8.0 / danger_increase)
                    else:
                        corner_distance = 100.0
                else:
                    corner_distance = 100.0

            # Aplicar frenagem proporcional apenas quando necessário
            if required_braking_distance > 0 and corner_distance < required_braking_distance * 1.3:
                brake_intensity = max(0.0, 1.0 - (corner_distance / (required_braking_distance * 1.3)))
                brake = min(0.88, brake_intensity * 1.15)
                accel = max(0.05, accel * (1.0 - brake * 0.65))

        # Tratamento especial para curvas muito fechadas
        if danger > 0.75 and speed_kmh > 22:
            brake = min(0.97, brake + (danger - 0.75) * 2.2)
            accel = 0.0
            if danger > 0.92 and speed_kmh < 18:
                brake = min(brake, 0.68)
        
        # Freio de emergência se fora da pista
        track_pos = float(car_state.get('trackPos', 0.0))
        if abs(track_pos) > 1.55 and speed_kmh > 12:
            additional_brake = min(0.38, (abs(track_pos) - 1.55) * 0.55)
            brake = max(brake, additional_brake)
            accel = 0.0

        # Estabilidade em curvas de alta velocidade
        angle = float(car_state.get('angle', 0.0))
        if speed_kmh > 65 and abs(angle) > 0.22:
            accel = max(0.05, accel - 0.32)

        # Gerenciamento de aceleração por tipo de curva
        if self.curve_type == "corner" and speed_kmh < 23:
            time_in_corner = time.time() - self.curve_start_time
            if time_in_corner > 1.0:  # Tempo para estabilizar
                accel = min(0.38, accel + 0.04)
        elif self.curve_type == "long_curve":
            curve_duration = time.time() - self.curve_start_time
            if curve_duration > 2.0:  # Após a entrada
                accel = min(0.68, accel + 0.025)
        
        # Recuperação de oscilação
        if self.oscillation_counter > 4.5 and speed_kmh > 22:
            accel = max(0.07, accel * 0.3)
            brake = min(0.58, brake + 0.14)

        # Armazenar para uso futuro
        self.last_accel = accel
        
        # Atualizar última curva para aceleração pós-curva
        if self.curve_type != self.last_curve_type:
            self.last_curve_type = self.curve_type

        # garantir limites
        accel = float(np.clip(accel, 0.0, 1.0))
        brake = float(np.clip(brake, 0.0, 1.0))
        if brake > 0.1:
            accel = 0.0
        return accel, brake

    def _get_gear(self, rpm, gear, speed_x, danger):
        """Decide marcha com base em speed thresholds, rpm e proteção contra oscilações."""
        try:
            rpm = float(rpm)
            speed_x = float(speed_x)
            gear = int(gear)
        except Exception:
            return max(1, min(6, int(gear)))

        # Nunca voltar para 1a em velocidade
        if speed_x > 22 and gear == 1:
            return 2

        # Determina marcha desejada por velocidade - LIMIARES AJUSTADOS
        desired = 1
        for i, thr in enumerate(self.gear_speed_thresholds):
            if speed_x >= thr:
                desired = i + 1
        desired = max(1, min(6, desired))

        now = time.time()
        min_time_between_changes = 0.3  # Reduzido para permitir mudanças mais rápidas

        # Se desejada maior que atual, suba progressivamente
        if desired > gear:
            # Subir de marcha mais cedo para evitar travamento em 54 km/h
            if (rpm > 3000 or speed_x > self.gear_speed_thresholds[gear] + 4.5) and (now - self.last_gear_change_time) > min_time_between_changes:
                self.last_gear_change_time = now
                return gear + 1
            return gear

        # Se desejada menor que atual, reduza com condições seguras
        if desired < gear:
            # Reduzir de marcha mais tarde para manter aceleração
            if (rpm < 2600 or speed_x < self.gear_speed_thresholds[gear - 2] - 4) and (now - self.last_gear_change_time) > min_time_between_changes:
                self.last_gear_change_time = now
                return gear - 1
            return gear

        # Condições de rpm extremo - AJUSTADAS
        if rpm > 7000 and gear < 6 and (now - self.last_gear_change_time) > min_time_between_changes:
            self.last_gear_change_time = now
            return gear + 1
        if rpm < 2000 and gear > 2 and (now - self.last_gear_change_time) > min_time_between_changes:
            self.last_gear_change_time = now
            return gear - 1

        # Mantém marcha
        return gear

    def drive(self, car_state):
        """Loop principal: valida estado, computa fuzzy e retorna comandos."""
        now = time.time()
        if not self._validate_car_state(car_state):
            if now - self.last_valid_state_time > 1.5:
                self.last_steer = 0.0
                self.launch_phase = True
                self.launch_timer = 0
                self.oscillation_counter = 0
                self.last_direction = 0
            self.last_valid_state_time = now
            return {'accel': 0.0, 'brake': 0.0, 'steer': 0.0, 'gear': 1, 'clutch': 0.0, 'focus': -1}

        self.last_valid_state_time = now

        try:
            current_speed = float(car_state.get('speedX', 0.0))
            self.last_speed = max(0.0, current_speed)
        except Exception:
            current_speed = self.last_speed

        fuzzy_drive_output = {'steer': 0.0, 'accel': 0.5}
        if self.fuzzy_valid:
            try:
                self.drive_sim.input['track_pos'] = float(car_state.get('trackPos', 0.0))
                self.drive_sim.input['angle'] = float(car_state.get('angle', 0.0))
                self.drive_sim.input['speed_x'] = current_speed
                self.drive_sim.compute()
                fuzzy_drive_output = {'steer': float(self.drive_sim.output.get('steer', 0.0)),
                                      'accel': float(self.drive_sim.output.get('accel', 0.5))}
            except Exception:
                self.fuzzy_valid = False

        track_sensor = car_state.get('track', [200.0] * 19)
        curvature, desired_offset, danger = self._analyze_track_sensors(track_sensor, current_speed)

        # Cálculo mais robusto da distância até a curva
        try:
            # Usar sensores médios para uma estimativa mais estável
            corner_distance = float(np.mean([x for x in track_sensor[7:12] if x is not None]))
        except Exception:
            corner_distance = None

        steer = self._get_steering(car_state, fuzzy_drive_output, current_speed, curvature, desired_offset, danger)
        accel, brake = self._get_acceleration_and_brake(car_state, fuzzy_drive_output, current_speed, danger, corner_distance)
        gear = self._get_gear(car_state.get('rpm', 0.0), car_state.get('gear', 1), current_speed, danger)

        control = {
            'accel': float(np.clip(accel, 0.0, 1.0)),
            'brake': float(np.clip(brake, 0.0, 1.0)),
            'steer': float(np.clip(steer, -1.0, 1.0)),
            'gear': int(max(1, min(6, gear))),
            'clutch': 0.0,
            'focus': -1
        }

        if int(now) % 2 == 0:
            danger_trend = "stable"
            if len(self.danger_history) >= 2:
                if self.danger_history[-1] > self.danger_history[-2] + 0.1:
                    danger_trend = "rising"
                elif self.danger_history[-1] < self.danger_history[-2] - 0.1:
                    danger_trend = "falling"
            logger.debug(f"S:{control['steer']:.2f} A:{control['accel']:.2f} B:{control['brake']:.2f} G:{control['gear']} "
                         f"Speed:{current_speed:.1f} Danger:{danger:.2f}({danger_trend}) Curv:{curvature:.2f} Curve:{self.curve_type}")

        return control