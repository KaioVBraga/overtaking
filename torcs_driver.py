# torcs_driver.py
import numpy as np

from log import logger

# Importando os módulos de Interpretation e Actions.
# Ajuste os caminhos de import caso você tenha os pacotes montados (por exemplo: Interpretation.track)
try:
    from Interpretation import track as track_mod
    from Interpretation import intention as intention_mod
    from Actions import accelaration as accel_mod
    from Actions import gear as gear_mod
    from Actions import steering as steering_mod
except Exception:
    # Fallback caso os módulos estejam no mesmo diretório (ou durante testes)
    import track as track_mod
    import intention as intention_mod
    import accelaration as accel_mod
    import gear as gear_mod
    import steering as steering_mod


class TorcsDriver:
    def __init__(self):
        self.name = "PilotoNebuloso"
        self.author = "Rafael | Kaio"
        self.version = "1.1"

        self.tick = 0

        # Controle
        self.gear = 1
        self.steering = 0.0
        self.accel = 0.0
        self.brake = 0.0
        self.last_steer = 0

        # Estado interpretado
        self._last_classification = None
        self._last_severity = 0.0 # Classificação da 'severidade' da curva
        self._last_intention = 0.0 # Classificação da 'intenção' aumentar ou diminuir a velocidade

        # Parâmetros de largada
        self.LAUNCH_DIST_THRESHOLD = 5.0
        self.LAUNCH_MAX_SPEED = 5.0
        self.LAUNCH_STEER_AGGRESSIVENESS = 0.25

        # Construir modelos fuzzy nos módulos
        # Cada módulo adiciona atributos ao objeto (ex.: self.turn_classifier, self.accel_brake_ctrl...)
        track_mod.turn_classifier_model(self)
        accel_mod.accel_brake_model(self)
        gear_mod.build_gear_model(self)
        # steering não precisa de modelo, é calculo direto

    def init(self):
        self.gear = 1
        self.accel = 0.0
        self.brake = 0.0
        self.steering = 0.0
        self.tick = 0

    def is_launch(self, sensors):
        try:
            dist = float(sensors.get('distRaced', 9999.0))
            if dist <= self.LAUNCH_DIST_THRESHOLD:
                return True
            speed = float(sensors.get('speedX', 0.0))
            rpm = float(sensors.get('rpm', 0.0))
            if speed <= self.LAUNCH_MAX_SPEED and rpm > 1500:
                return True
        except Exception as e:
            logger.debug(f"is_launch heuristics error: {e}")
        return False

    # Handlers que chamam os módulos Actions
    def steering_handler(self, sensors, aggressiveness=1.0):
        try:
            steer = steering_mod.steering_controller(self, sensors, aggressiveness)
            self.steering = float(np.clip(steer, -1.0, 1.0))
        except Exception as e:
            logger.warning(f"steering_handler error: {e}")
            # mantém ultimo valor
            self.steering = float(np.clip(self.steering, -1.0, 1.0))

    def accel_brake_handler(self, sensors):
        try:
            accel, brake = accel_mod.accel_brake_controller(self, sensors)
            self.accel = float(np.clip(accel, 0.0, 1.0))
            self.brake = float(np.clip(brake, 0.0, 1.0))
            logger.debug(f"[ACCEL] intention={self._last_intention:.2f} -> accel={self.accel:.2f} brake={self.brake:.2f}")
        except Exception as e:
            logger.warning(f"accel_brake_handler error: {e}")
            self.accel = 0.0
            self.brake = 0.0

    def gear_handler(self, sensors):
        try:
            # Se for largada, manter marcha 1
            if self.is_launch(sensors):
                if self.gear != 1:
                    self.gear = 1
                return
            suggested = gear_mod.gear_controller(self, sensors)
            # gear_controller já atualiza self.gear internamente, mas retornamos a sugestão
            if suggested is not None:
                self.gear = int(suggested)
        except Exception as e:
            logger.warning(f"gear_handler error: {e}")

    # Orquestração principal
    def drive(self, sensors):
        self.tick += 1

        # 1) interpretar pista
        try:
            cls, sev = track_mod.turn_classifier_controller(self, sensors)
            # armazenar
            self._last_classification = cls
            self._last_severity = float(sev)
        except Exception as e:
            logger.warning(f"track interpretation error: {e}")
            self._last_classification, self._last_severity = 'straight', 0.0

        # 2) interpretar intenção (baseado em classificação e severidade)
        try:
            intention_mod.intention_interpreter(self, sensors)
            # intention_interpreter guarda em self._last_intention
        except Exception as e:
            logger.warning(f"intention interpretation error: {e}")
            self._last_intention = 0.0

        # 3) actions: accel/brake, gear, steering
        is_launch = self.is_launch(sensors)
        aggress = self.LAUNCH_STEER_AGGRESSIVENESS if is_launch else 1.0

        self.accel_brake_handler(sensors)
        self.gear_handler(sensors)
        self.steering_handler(sensors, aggressiveness=aggress)
        
        actual_steer = self.last_steer * 0.5 + self.steering * 0.5

        control = {
            'accel': float(self.accel),
            'brake': float(self.brake),
            'gear': int(self.gear),
            'steer': float(actual_steer)
        }
        
        self.last_steer = actual_steer
        
        return control

    def on_shutdown(self):
        pass
