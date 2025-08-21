# gear.py
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from log import logger

def build_gear_model(self):
    print ("Inicioooooooooooooooooooooooo\n\n\n\n\n\n")
    intention = ctrl.Antecedent(np.linspace(-1, 1, 201), 'intention')
    gear_in   = ctrl.Antecedent(np.linspace(1, 6, 6), 'gear_in')
    speed = ctrl.Antecedent(np.linspace(0, 350, 351), 'g_speed')
    rpm = ctrl.Antecedent(np.linspace(0, 10000, 101), 'g_rpm')
    gear_adj = ctrl.Consequent(np.linspace(-1, 1, 201), 'gear_adj')

    intention['braking'] = fuzz.trimf(intention.universe, [-1.0, -1.0, -0.2])
    intention['coast']   = fuzz.trimf(intention.universe, [-0.3, 0.0, 0.3])
    intention['accel']   = fuzz.trimf(intention.universe, [0.1, 1.0, 1.0])

    gear_in['low'] = fuzz.trimf(gear_in.universe, [1, 1, 2.5])
    gear_in['mid'] = fuzz.trimf(gear_in.universe, [2, 3, 4])
    gear_in['high'] = fuzz.trimf(gear_in.universe, [3.5, 6, 6])

    speed['low'] = fuzz.trimf(speed.universe, [0, 0, 60])
    speed['mid'] = fuzz.trimf(speed.universe, [40, 120, 200])
    speed['high'] = fuzz.trimf(speed.universe, [150, 250, 350])

    rpm['low'] = fuzz.trimf(rpm.universe, [0, 0, 3000])
    rpm['mid'] = fuzz.trimf(rpm.universe, [2000, 4000, 6000])
    rpm['high']= fuzz.trimf(rpm.universe, [5000, 8000, 10000])

    gear_adj['down'] = fuzz.trimf(gear_adj.universe, [-1.0, -1.0, -0.4])
    gear_adj['keep'] = fuzz.trimf(gear_adj.universe, [-0.3, 0.0, 0.3])
    gear_adj['up']   = fuzz.trimf(gear_adj.universe, [0.4, 1.0, 1.0])

    rules = [
        ctrl.Rule(intention['accel'] & rpm['high'] & gear_in['high'], gear_adj['up']),
        ctrl.Rule(intention['accel'] & rpm['mid'] & gear_in['mid'], gear_adj['keep']),
        ctrl.Rule(intention['accel'] & rpm['low'] & gear_in['low'], gear_adj['up']),
        ctrl.Rule(intention['braking'] & speed['low'], gear_adj['down']),
        ctrl.Rule(intention['braking'] & speed['mid'], gear_adj['down']),
        ctrl.Rule(intention['braking'] & speed['high'], gear_adj['down']),
        ctrl.Rule(intention['coast'] & rpm['low'] & gear_in['high'], gear_adj['down']),
        ctrl.Rule(intention['coast'] & rpm['mid'], gear_adj['keep']),
        ctrl.Rule(intention['accel'] & speed['high'] & gear_in['low'], gear_adj['up']),
    ]

    system = ctrl.ControlSystem(rules)
    self.gear_ctrl = ctrl.ControlSystemSimulation(system)

def gear_controller(self, sensors):
    """
    Calcula sugestão de marcha com base em self._last_intention, rpm e speed.
    Atualiza self.gear e retorna a gear sugerida (int).
    """
    rpm = float(sensors.get('rpm', 0.0))
    speed = float(sensors.get('speedX', 0.0))
    intention = float(getattr(self, '_last_intention', 0.0))

    try:
        self.gear_ctrl.input['intention'] = np.clip(intention, -1.0, 1.0)
        self.gear_ctrl.input['gear_in'] = np.clip(self.gear, 1.0, 6.0)
        self.gear_ctrl.input['g_speed'] = np.clip(speed, 0.0, 350.0)
        self.gear_ctrl.input['g_rpm'] = np.clip(rpm, 0.0, 10000.0)

        self.gear_ctrl.compute()
        gear_adj = float(self.gear_ctrl.output['gear_adj'])
    except Exception as e:
        logger.warning(f"Erro no gear fuzzy: {e}")
        gear_adj = 0.0

    UP_THRESHOLD = 0.4
    DOWN_THRESHOLD = -0.4

    suggested = int(self.gear)
    if gear_adj > UP_THRESHOLD and self.gear < 6:
        suggested = self.gear + 1
        self.gear = suggested
        logger.debug(f"Gear up -> {self.gear} (gear_adj={gear_adj:.2f})")
    elif gear_adj < DOWN_THRESHOLD and self.gear > 1:
        suggested = self.gear - 1
        self.gear = suggested
        logger.debug(f"Gear down -> {self.gear} (gear_adj={gear_adj:.2f})")
    else:
        # manter
        suggested = self.gear

    return suggested
