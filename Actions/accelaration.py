# accelaration.py
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from log import logger

def accel_brake_model(self):
    """
    Cria o controlador fuzzy que gera uma 'intention' [-1..1].
    Anexa em self.accel_brake_ctrl
    """
    turn = ctrl.Antecedent(np.linspace(0, 1, 101), 'turn_severity')
    speed = ctrl.Antecedent(np.linspace(0, 350, 351), 'speed')
    intention = ctrl.Consequent(np.linspace(-1, 1, 201), 'intention')

    turn['straight'] = fuzz.trimf(turn.universe, [0.0, 0.0, 0.25])
    turn['long']     = fuzz.trimf(turn.universe, [0.15, 0.3, 0.5])
    turn['medium']   = fuzz.trimf(turn.universe, [0.35, 0.55, 0.75])
    turn['sharp']    = fuzz.trimf(turn.universe, [0.6, 1.0, 1.0])

    speed['low']  = fuzz.trimf(speed.universe, [0, 0, 60])
    speed['mid']  = fuzz.trimf(speed.universe, [40, 120, 200])
    speed['high'] = fuzz.trimf(speed.universe, [150, 250, 350])

    intention['strong_brake'] = fuzz.trimf(intention.universe, [-1.0, -1.0, -0.5])
    intention['brake']        = fuzz.trimf(intention.universe, [-0.8, -0.4, -0.1])
    intention['coast']        = fuzz.trimf(intention.universe, [-0.2, 0.0, 0.2])
    intention['gentle_acc']   = fuzz.trimf(intention.universe, [0.1, 0.4, 0.7])
    intention['full_acc']     = fuzz.trimf(intention.universe, [0.5, 1.0, 1.0])

    rules = [
        ctrl.Rule(turn['straight'] & speed['low'], intention['full_acc']),
        ctrl.Rule(turn['straight'] & speed['mid'], intention['full_acc']),
        ctrl.Rule(turn['straight'] & speed['high'], intention['full_acc']),

        ctrl.Rule(turn['long'] & speed['low'], intention['gentle_acc']),
        ctrl.Rule(turn['long'] & speed['mid'], intention['coast']),
        ctrl.Rule(turn['long'] & speed['high'], intention['brake']),

        ctrl.Rule(turn['medium'] & speed['low'], intention['coast']),
        ctrl.Rule(turn['medium'] & speed['mid'], intention['brake']),
        ctrl.Rule(turn['medium'] & speed['high'], intention['strong_brake']),

        ctrl.Rule(turn['sharp'] & speed['low'], intention['brake']),
        ctrl.Rule(turn['sharp'] & speed['mid'], intention['strong_brake']),
        ctrl.Rule(turn['sharp'] & speed['high'], intention['strong_brake'])
    ]

    system = ctrl.ControlSystem(rules)
    self.accel_brake_ctrl = ctrl.ControlSystemSimulation(system)

def accel_brake_controller(self, sensors):
    """
    Usa self._last_severity e sensors para computar intention, depois converte para accel, brake.
    Retorna (accel, brake) ambos em 0..1
    """
    speed = float(sensors.get('speedX', 0.0))
    severity = float(getattr(self, '_last_severity', 0.0))

    try:
        self.accel_brake_ctrl.input['turn_severity'] = np.clip(severity, 0.0, 1.0)
        self.accel_brake_ctrl.input['speed'] = np.clip(speed, 0.0, 350.0)
        self.accel_brake_ctrl.compute()
        intention = float(self.accel_brake_ctrl.output['intention'])
    except Exception as e:
        logger.warning(f"Erro no accel/brake fuzzy: {e}")
        intention = 0.0

    # registra intenção num estado do driver para uso por outros módulos (e debug)
    self._last_intention = float(np.clip(intention, -1.0, 1.0))

    if intention >= 0.0:
        amountAccel = float(np.clip(intention, 0.0, 1.0))
        amountBrake = 0.0
    else:
        amountAccel = 0.0
        amountBrake = float(np.clip(-intention, 0.0, 1.0))

    return amountAccel, amountBrake
