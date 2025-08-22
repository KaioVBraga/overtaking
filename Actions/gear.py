# gear.py
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from log import logger

def build_gear_model(self):
    """
    Modelo fuzzy de marcha com:
    - Todas as regras principais cobertas.
    - Decisão de marcha baixa em curvas fechadas DENTRO do sistema fuzzy.
    - Prioridades explícitas para segurança e eficiência.
    """
    # === Antecedentes ===
    intention = ctrl.Antecedent(np.linspace(-1, 1, 201), 'intention')
    rpm = ctrl.Antecedent(np.linspace(0, 10000, 101), 'rpm')
    speed = ctrl.Antecedent(np.linspace(0, 350, 351), 'speed')
    gear_in = ctrl.Antecedent(np.arange(1, 7), 'gear_in')
    severity = ctrl.Antecedent(np.linspace(0, 1, 101), 'severity')
    gear_adj = ctrl.Consequent(np.linspace(-1, 1, 201), 'gear_adj')

    # --- Funções de Pertinência ---

    # Intention
    intention['braking'] = fuzz.trimf(intention.universe, [-1, -1, -0.2])
    intention['coast']   = fuzz.trimf(intention.universe, [-0.3, 0, 0.3])
    intention['accel']   = fuzz.trimf(intention.universe, [0.1, 1, 1])

    # RPM
    rpm['very_low'] = fuzz.trimf(rpm.universe, [0, 0, 2000])
    rpm['low']      = fuzz.trimf(rpm.universe, [1000, 3000, 4000])
    rpm['mid']      = fuzz.trimf(rpm.universe, [3000, 5000, 7000])
    rpm['high']     = fuzz.trimf(rpm.universe, [6000, 8000, 10000])
    rpm['very_high']= fuzz.trimf(rpm.universe, [8000, 10000, 10000])

    # Speed
    speed['low']  = fuzz.trimf(speed.universe, [0, 0, 60])
    speed['mid']  = fuzz.trimf(speed.universe, [40, 120, 200])
    speed['high'] = fuzz.trimf(speed.universe, [150, 250, 350])

    # Gear In
    gear_in['low']  = fuzz.zmf(gear_in.universe, 1, 3)      # 1-2
    gear_in['mid']  = fuzz.pimf(gear_in.universe, 2, 3, 4, 5) # 3-4
    gear_in['high'] = fuzz.smf(gear_in.universe, 4, 6)      # 5-6

    # Severity
    severity['low']   = fuzz.trimf(severity.universe, [0.0, 0.0, 0.3])
    severity['medium'] = fuzz.trimf(severity.universe, [0.2, 0.5, 0.8])
    severity['high']  = fuzz.trimf(severity.universe, [0.6, 1.0, 1.0])

    # Gear Adjustment
    gear_adj['down'] = fuzz.trimf(gear_adj.universe, [-1, -1, -0.4])
    gear_adj['keep'] = fuzz.trimf(gear_adj.universe, [-0.3, 0, 0.3])
    gear_adj['up']   = fuzz.trimf(gear_adj.universe, [0.4, 1, 1])

    # === REGRAS: Cobertura Lógica e Completa ===
    rules = []

    # --- 1. Prioridade Máxima: Curvas Fechadas → Forçar Marcha Baixa ---
    # Mesmo acelerando, se a curva é fechada e velocidade baixa, reduzir
    rules += [
        ctrl.Rule(
            severity['high'] & speed['low'],
            gear_adj['down']
        ),
        ctrl.Rule(
            severity['high'] & speed['mid'] & intention['braking'],
            gear_adj['down']
        ),
        ctrl.Rule(
            severity['high'] & speed['mid'] & intention['coast'],
            gear_adj['down']
        ),
    ]

    # --- 2. Frenagem → Sempre Reduzir ---
    rules += [
        ctrl.Rule(
            intention['braking'],
            gear_adj['down']
        ),
    ]

    # --- 3. Aceleração ---
    rules += [
        # Overrev: subir mesmo em baixa intenção
        ctrl.Rule(rpm['very_high'], gear_adj['up']),
        ctrl.Rule(rpm['high'] & ~gear_in['high'], gear_adj['up']),

        # Acelerando com RPM médio → manter ou subir se em marcha baixa
        ctrl.Rule(intention['accel'] & rpm['mid'], gear_adj['keep']),
        ctrl.Rule(intention['accel'] & rpm['high'], gear_adj['up']),
        # ctrl.Rule(intention['accel'] & rpm['mid'] & gear_in['mid'], gear_adj['keep']),
        # ctrl.Rule(intention['accel'] & rpm['mid'] & gear_in['high'], gear_adj['keep']),

        # Acelerando com RPM baixo → reduzir para recuperar torque
        ctrl.Rule(intention['accel'] & rpm['low'] & gear_in['mid'], gear_adj['down']),
        ctrl.Rule(intention['accel'] & rpm['low'] & gear_in['high'], gear_adj['down']),
        ctrl.Rule(intention['accel'] & rpm['very_low'], gear_adj['down']),
    ]

    # --- 4. Coasting (neutro) ---
    rules += [
        ctrl.Rule(intention['coast'] & rpm['mid'], gear_adj['keep']),
        ctrl.Rule(intention['coast'] & rpm['high'] & ~gear_in['high'], gear_adj['up']),
        ctrl.Rule(intention['coast'] & rpm['low'] & gear_in['high'], gear_adj['down']),
    ]

    # --- 5. Alta Velocidade + Alta Severidade → Reduzir (antecipação) ---
    rules += [
        ctrl.Rule(severity['high'] & speed['high'] & intention['accel'], gear_adj['down']),
        ctrl.Rule(severity['medium'] & intention['braking'], gear_adj['down']),
    ]

    # --- 6. Manutenção (Keep) - Casos Estáveis ---
    rules += [
        ctrl.Rule(
            intention['accel'] & rpm['mid'] & speed['high'] & gear_in['high'],
            gear_adj['keep']
        ),
        ctrl.Rule(
            intention['coast'] & rpm['mid'] & speed['mid'],
            gear_adj['keep']
        ),
    ]

    # --- 7. Fallback Geral: Quando nada se aplica, manter marcha ---
    # Esta regra tem baixa prioridade, mas garante saída definida
    rules += [
        ctrl.Rule(
            ~intention['braking'] & ~intention['accel'] & 
            rpm['mid'] & 
            ~severity['high'],
            gear_adj['keep']
        ),
    ]

    # Criar sistema com todas as regras
    system = ctrl.ControlSystem(rules)
    self.gear_ctrl = ctrl.ControlSystemSimulation(system)


def gear_controller(self, sensors):
    """
    Controlador de marcha com todas as decisões dentro do fuzzy.
    Nenhuma regra crítica fora do sistema.
    """
    try:
        rpm = float(sensors.get('rpm', 0.0))
        speed = float(sensors.get('speedX', 0.0))
        intention = float(getattr(self, '_last_intention', 0.0))
        severity = float(getattr(self, '_last_severity', 0.0))

        # Entradas
        self.gear_ctrl.input['intention'] = np.clip(intention, -1.0, 1.0)
        self.gear_ctrl.input['rpm'] = np.clip(rpm, 0.0, 10000.0)
        self.gear_ctrl.input['speed'] = np.clip(speed, 0.0, 350.0)
        self.gear_ctrl.input['gear_in'] = float(self.gear)
        self.gear_ctrl.input['severity'] = np.clip(severity, 0.0, 1.0)

        # Computar
        self.gear_ctrl.compute()

        # Saída segura
        gear_adj = float(self.gear_ctrl.output.get('gear_adj', 0.0))

    except Exception as e:
        logger.warning(f"Erro no gear fuzzy: {e} — fallback keep")
        gear_adj = 0.0

    # Histerese temporal
    MIN_TICKS = 30
    last = getattr(self, '_last_gear_change_tick', -MIN_TICKS)
    if (self.tick - last) < MIN_TICKS:
        return self.gear  # bloqueia troca frequente

    # Aplicar decisão
    UP_THRESH = 0.4
    DOWN_THRESH = -0.4
    suggested = self.gear

    if gear_adj > UP_THRESH and self.gear < 6:
        suggested = self.gear + 1
        self.gear = suggested
        self._last_gear_change_tick = self.tick
        logger.debug(f"Gear up -> {self.gear} | adj={gear_adj:.2f}")
    elif gear_adj < DOWN_THRESH and self.gear > 1:
        suggested = self.gear - 1
        self.gear = suggested
        self._last_gear_change_tick = self.tick
        logger.debug(f"Gear down -> {self.gear} | adj={gear_adj:.2f}")

    return suggested