# track.py (versão melhorada)
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from log import logger

def turn_classifier_model(self):
    """
    Modelo fuzzy melhorado: universo ajustado, MFs mais realistas.
    """
    center = ctrl.Antecedent(np.linspace(0, 100, 101), 'center')  # foco em 0-100m
    side = ctrl.Antecedent(np.linspace(0, 100, 101), 'side')
    turn_severity = ctrl.Consequent(np.linspace(0, 1, 101), 'turn_severity')

    # MFs ajustadas
    center['very_close'] = fuzz.trimf(center.universe, [0, 0, 30])
    center['close']      = fuzz.trimf(center.universe, [20, 40, 60])
    center['far']        = fuzz.trimf(center.universe, [50, 100, 100])

    side['small']  = fuzz.trimf(side.universe, [0, 0, 20])
    side['medium'] = fuzz.trimf(side.universe, [10, 30, 50])
    side['large']  = fuzz.trimf(side.universe, [40, 100, 100])

    turn_severity['straight']   = fuzz.trimf(turn_severity.universe, [0.0, 0.0, 0.2])
    turn_severity['long_turn']  = fuzz.trimf(turn_severity.universe, [0.1, 0.3, 0.5])
    turn_severity['medium']     = fuzz.trimf(turn_severity.universe, [0.35, 0.55, 0.75])
    turn_severity['sharp']      = fuzz.trimf(turn_severity.universe, [0.6, 1.0, 1.0])

    # Regras completas
    rules = [
        ctrl.Rule(center['far'] & side['small'], turn_severity['straight']),
        ctrl.Rule(center['far'] & side['medium'], turn_severity['long_turn']),
        ctrl.Rule(center['far'] & side['large'], turn_severity['long_turn']),
        ctrl.Rule(center['close'] & side['small'], turn_severity['long_turn']),
        ctrl.Rule(center['close'] & side['medium'], turn_severity['medium']),
        ctrl.Rule(center['close'] & side['large'], turn_severity['sharp']),
        ctrl.Rule(center['very_close'] & side['small'], turn_severity['medium']),
        ctrl.Rule(center['very_close'] & side['medium'], turn_severity['sharp']),
        ctrl.Rule(center['very_close'] & side['large'], turn_severity['sharp']),
    ]

    system = ctrl.ControlSystem(rules)
    self.turn_classifier = ctrl.ControlSystemSimulation(system)

def turn_classifier_controller(self, sensors):
    import numpy as _np

    track = sensors.get('track', None)
    if track is None or len(track) == 0:
        return 'straight', 0.0

    center_idx = len(track) // 2
    center_raw = float(track[center_idx])

    # Filtrar valores válidos (>0)
    left_vals = [float(v) for v in track[:center_idx] if float(v) > 0.0]
    right_vals = [float(v) for v in track[center_idx+1:] if float(v) > 0.0]

    # Usar mediana para robustez
    if center_raw > 0.0:
        center_dist = center_raw
    else:
        candidates = []
        if left_vals:
            candidates.append(_np.median(left_vals))
        if right_vals:
            candidates.append(_np.median(right_vals))
        center_dist = _np.median(candidates) if candidates else 100.0

    left_med = _np.median(left_vals) if left_vals else center_dist
    right_med = _np.median(right_vals) if right_vals else center_dist
    side_diff = abs(left_med - right_med)

    try:
        self.turn_classifier.input['center'] = _np.clip(center_dist, 0, 100)
        self.turn_classifier.input['side'] = _np.clip(side_diff, 0, 100)
        self.turn_classifier.compute()
        turn_severity = float(self.turn_classifier.output['turn_severity'])

        # Suavização temporal (filtro exponencial)
        alpha = 0.3
        last_smooth = getattr(self, '_last_turn_severity_smooth', turn_severity)
        smoothed = alpha * turn_severity + (1 - alpha) * last_smooth
        self._last_turn_severity_smooth = smoothed

    except Exception as e:
        logger.warning(f"Erro no fuzzy: {e}; usando fallback 0.3")
        smoothed = 0.3

    # Thresholds alinhados com MFs
    if smoothed < 0.2:
        cls = 'straight'
    elif smoothed < 0.4:
        cls = 'long_turn'
    elif smoothed < 0.65:
        cls = 'medium_turn'
    else:
        cls = 'sharp_turn'

    return cls, smoothed