# track.py
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from log import logger

def turn_classifier_model(self):
    """
    Cria o modelo fuzzy de classificação de curva e anexa em self.turn_classifier
    """
    center = ctrl.Antecedent(np.linspace(0, 200, 201), 'center_dist')
    side = ctrl.Antecedent(np.linspace(0, 200, 201), 'side_diff')
    turn_severity = ctrl.Consequent(np.linspace(0, 1, 101), 'turn_severity')

    center['very_close'] = fuzz.trimf(center.universe, [0, 0, 40])
    center['close']      = fuzz.trimf(center.universe, [20, 50, 90])
    center['far']        = fuzz.trimf(center.universe, [70, 120, 200])

    side['small']  = fuzz.trimf(side.universe, [0, 0, 30])
    side['medium'] = fuzz.trimf(side.universe, [15, 50, 90])
    side['large']  = fuzz.trimf(side.universe, [60, 120, 200])

    turn_severity['straight']   = fuzz.trimf(turn_severity.universe, [0.0, 0.0, 0.25])
    turn_severity['long_turn']  = fuzz.trimf(turn_severity.universe, [0.15, 0.3, 0.5])
    turn_severity['medium']     = fuzz.trimf(turn_severity.universe, [0.35, 0.55, 0.75])
    turn_severity['sharp']      = fuzz.trimf(turn_severity.universe, [0.6, 1.0, 1.0])

    rules = [
        ctrl.Rule(center['far'] & side['small'], turn_severity['straight']),
        ctrl.Rule(center['far'] & side['medium'], turn_severity['long_turn']),
        ctrl.Rule(center['close'] & side['medium'], turn_severity['medium']),
        ctrl.Rule(center['very_close'] & side['large'], turn_severity['sharp']),
        ctrl.Rule(center['close'] & side['large'], turn_severity['sharp']),
        ctrl.Rule(center['far'] & side['large'], turn_severity['medium'])
    ]

    system = ctrl.ControlSystem(rules)
    self.turn_classifier = ctrl.ControlSystemSimulation(system)

def turn_classifier_controller(self, sensors):
    """
    Usa apenas sensors['track'] (lista) e sensors['angle'].
    Retorna: (classification:str, severity:float)
    """
    track = sensors.get('track', None)
    if track is None or len(track) == 0:
        return 'straight', 0.0

    center_idx = len(track) // 2
    center_dist = float(track[center_idx])

    left = np.array(track[:center_idx]) if center_idx > 0 else np.array([])
    right = np.array(track[center_idx+1:]) if center_idx+1 <= len(track) else np.array([])

    left_mean = float(np.mean(left)) if left.size > 0 else center_dist
    right_mean = float(np.mean(right)) if right.size > 0 else center_dist

    side_diff = abs(left_mean - right_mean)
    # alimenta fuzzy
    try:
        self.turn_classifier.input['center_dist'] = np.clip(center_dist, 0, 200)
        self.turn_classifier.input['side_diff'] = np.clip(side_diff, 0, 200)
        self.turn_classifier.compute()
        turn_severity = float(self.turn_classifier.output['turn_severity'])
    except Exception as e:
        logger.warning(f"Erro no classifier fuzzy: {e}")
        turn_severity = 0.0

    if turn_severity < 0.25:
        cls = 'straight'
    elif turn_severity < 0.45:
        cls = 'long_turn'
    elif turn_severity < 0.7:
        cls = 'medium_turn'
    else:
        cls = 'sharp_turn'

    return cls, turn_severity
