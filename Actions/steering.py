# steering.py
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from log import logger

def steering_aggressiveness_model(self):
    """
    Cria um modelo fuzzy para determinar a agressividade do controle de direção.
    Entradas:
        - speed: velocidade atual (0 a 350 km/h)
        - severity: severidade da curva (0 a 1)
        - dist_to_turn: distância estimada até a curva (0 a 100m)
    Saída:
        - aggressiveness: [0.1 a 1.0] → fator de suavização do steer
    """
    speed = ctrl.Antecedent(np.linspace(0, 350, 351), 'speed')
    severity = ctrl.Antecedent(np.linspace(0, 1, 101), 'severity')
    dist = ctrl.Antecedent(np.linspace(0, 100, 101), 'dist_to_turn')
    agg = ctrl.Consequent(np.linspace(0.05, 1.0, 91), 'aggressiv(0.05, 1.0, 91)eness')

    # Speed
    speed['low'] = fuzz.trimf(speed.universe, [0, 0, 60])
    speed['mid'] = fuzz.trimf(speed.universe, [40, 120, 200])
    speed['high'] = fuzz.trimf(speed.universe, [150, 250, 350])

    # Severity
    severity['low'] = fuzz.trimf(severity.universe, [0.0, 0.0, 0.3])
    severity['medium'] = fuzz.trimf(severity.universe, [0.2, 0.5, 0.8])
    severity['high'] = fuzz.trimf(severity.universe, [0.6, 1.0, 1.0])

    # Distance to Turn
    dist['very_close'] = fuzz.trimf(dist.universe, [0, 0, 20])
    dist['close'] = fuzz.trimf(dist.universe, [10, 30, 50])
    dist['far'] = fuzz.trimf(dist.universe, [40, 100, 100])

    # Aggressiveness
    agg['gentle'] = fuzz.trimf(agg.universe, [0.1, 0.1, 0.4])
    agg['normal'] = fuzz.trimf(agg.universe, [0.3, 0.5, 0.7])
    agg['aggressive'] = fuzz.trimf(agg.universe, [0.6, 1.0, 1.0])

    rules = [
        # Curvas fechadas → sempre suave, mesmo perto
        ctrl.Rule(severity['high'] & speed['high'], agg['gentle']),
        ctrl.Rule(severity['high'] & speed['mid'], agg['gentle']),
        ctrl.Rule(severity['high'] & speed['low'], agg['normal']),

        # Curvas médias → ajustar com base na distância
        ctrl.Rule(severity['medium'] & dist['far'], agg['normal']),
        ctrl.Rule(severity['medium'] & dist['close'], agg['gentle']),
        ctrl.Rule(severity['medium'] & dist['very_close'], agg['gentle']),

        # Curvas leves → pode ser agressivo
        ctrl.Rule(severity['low'] & dist['far'], agg['aggressive']),
        ctrl.Rule(severity['low'] & speed['low'], agg['normal']),
        ctrl.Rule(severity['low'] & speed['high'] & dist['close'], agg['normal']),

        # Alta velocidade → sempre mais suave
        ctrl.Rule(speed['high'] & severity['medium'], agg['gentle']),
        ctrl.Rule(speed['high'] & dist['close'], agg['gentle']),
    ]

    system = ctrl.ControlSystem(rules)
    self.steering_aggressiveness_ctrl = ctrl.ControlSystemSimulation(system)


def estimate_distance_to_turn(track):
    """
    Estima a distância até o início da curva com base na assimetria crescente.
    Usa os sensores laterais para detectar quando a curva começa a "aparecer".
    Retorna distância estimada em "unidades de sensor" (0 a ~100).
    """
    if len(track) < 5:
        return 100.0  # sem dados → longe

    t = np.array(track, dtype=float)
    center_idx = len(t) // 2

    left = t[:center_idx]
    right = t[center_idx + 1:]

    # Calcular assimetria em blocos (ex: 3 sensores mais próximos, médios, distantes)
    n = min(len(left), len(right))
    if n == 0:
        return 100.0

    # Dividir em zonas: próximas, médias, distantes
    near_size = max(1, n // 3)
    mid_size = max(1, n // 3)
    far_size = n - near_size - mid_size

    # Zonas: da frente para trás (mais distante → mais próxima)
    left_far = left[-(far_size + mid_size + near_size):-mid_size - near_size] if far_size > 0 else left[-1:]
    right_far = right[-(far_size + mid_size + near_size):-mid_size - near_size] if far_size > 0 else right[-1:]

    left_mid = left[-(mid_size + near_size):-near_size] if mid_size > 0 else left[-1:]
    right_mid = right[-(mid_size + near_size):-near_size] if mid_size > 0 else right[-1:]

    left_near = left[-near_size:]
    right_near = right[-near_size:]

    # Diferença absoluta normalizada por zona
    def norm_diff(a, b):
        ma = np.mean(a) if len(a) > 0 and np.all(a > 0) else 0
        mb = np.mean(b) if len(b) > 0 and np.all(b > 0) else 0
        total = ma + mb
        return abs(ma - mb) / (total if total > 1e-6 else 1)

    diff_far = norm_diff(left_far, right_far)
    diff_mid = norm_diff(left_mid, right_mid)
    diff_near = norm_diff(left_near, right_near)

    # Se a assimetria aumenta conforme nos aproximamos → curva está perto
    if diff_near > diff_mid > diff_far:
        return 20.0
    elif diff_near > diff_mid:
        return 40.0
    elif diff_mid > diff_far:
        return 70.0
    else:
        return 100.0  # curva longe ou reta


def steering_controller(self, sensors, override_aggressiveness=None):
    """
    Controlador de direção com:
    - Estimativa de distância até a curva.
    - Agressividade ajustada por fuzzy.
    - Histerese temporal (atualiza a cada N ticks).
    """
    track = sensors.get('track', None)
    speed = float(sensors.get('speedX', 0.0))
    severity = float(getattr(self, '_last_severity', 0.0))

    if track is None or len(track) == 0:
        return getattr(self, 'steering', 0.0)

    # === 1. Estimar distância até a curva ===
    try:
        dist_to_turn = self._dist_to_turn
    except:
        dist_to_turn = 100.0

    # === 2. Calcular agressividade com fuzzy (se não for sobrescrita) ===
    if override_aggressiveness is None:
        if not hasattr(self, 'steering_aggressiveness_ctrl'):
            steering_aggressiveness_model(self)  # lazy init

        try:
            self.steering_aggressiveness_ctrl.input['speed'] = np.clip(speed, 0.0, 350.0)
            self.steering_aggressiveness_ctrl.input['severity'] = np.clip(severity, 0.0, 1.0)
            self.steering_aggressiveness_ctrl.input['dist_to_turn'] = np.clip(dist_to_turn, 0.0, 100.0)
            self.steering_aggressiveness_ctrl.compute()
            aggressiveness = float(self.steering_aggressiveness_ctrl.output['aggressiveness'])
        except Exception as e:
            logger.warning(f"Erro no fuzzy de agressividade: {e} → usando 0.7")
            aggressiveness = 0.7
    else:
        aggressiveness = float(override_aggressiveness)

    # === 3. Calcular steer bruto com mediana ===
    try:
        t = np.array(track, dtype=float)
        center_idx = len(t) // 2
        center = float(t[center_idx])
        left = t[:center_idx]
        right = t[center_idx + 1:]

        left_med = np.median(left[left > 0]) if np.any(left > 0) else center
        right_med = np.median(right[right > 0]) if np.any(right > 0) else center

        if left_med + right_med < 1e-6:
            steer_raw = 0.0
        else:
            steer_raw = (left_med - right_med) / (left_med + right_med)
            steer_raw = float(np.clip(steer_raw, -1.0, 1.0))

    except Exception as e:
        logger.warning(f"Erro no cálculo de steer: {e}")
        steer_raw = getattr(self, 'steering', 0.0)

    # === 4. Histerese temporal: só atualiza a cada N ticks ===
    UPDATE_EVERY = 3  # atualiza direção a cada 3 ticks
    if hasattr(self, 'tick'):
        if self.tick % UPDATE_EVERY != 0:
            return getattr(self, 'steering', steer_raw)  # mantém último

    # === 5. Aplicar agressividade e limitar ===
    final_steer = steer_raw * aggressiveness
    final_steer = float(np.clip(final_steer, -1.0, 1.0))

    # Armazenar para uso futuro
    if not hasattr(self, 'steering'):
        self.steering = 0.0
    self.steering = final_steer

    # Log opcional
    if hasattr(self, 'tick') and self.tick % 200 == 0:
        logger.debug(
            f"Steer: raw={steer_raw:.2f}, "
            f"agg={aggressiveness:.2f}, "
            f"final={final_steer:.2f}, "
            f"spd={speed:.1f}, "
            f"sev={severity:.2f}, "
            f"dist_to_turn={dist_to_turn:.1f}"
        )

    return final_steer