# intention.py
from log import logger
import numpy as np

def intention_interpreter(self, sensors):
    """
    A partir da classificação (self._last_classification) e severidade
    (self._last_severity) calcula uma intenção entre -1..+1:
      - valores negativos -> frear / reduzir
      - valores positivos -> acelerar
    O valor final é armazenado em self._last_intention.
    """
    cls = getattr(self, '_last_classification', 'straight')
    sev = float(getattr(self, '_last_severity', 0.0))
    speed = float(sensors.get('speedX', 0.0))

    # Base heurística simples (pode evoluir para fuzzy depois)
    if sev < 0.25:
        base = 0.9
    elif sev < 0.45:
        base = 0.4
    elif sev < 0.7:
        base = -0.3
    else:
        base = -0.9

    # adaptar pela velocidade: em alta velocidade, curvas exigem intenção mais de frear
    speed_factor = np.clip(speed / 200.0, 0.0, 1.0)  # 0..1
    if base < 0:
        # se base é negativo (frenagem), intensifica com a velocidade
        intention = base * (0.5 + 0.5 * speed_factor)
    else:
        # se base é positivo (aceleração), reduz um pouco quando a severidade é média
        intention = base * (1.0 - 0.6 * sev)

    # normaliza para -1..1
    intention = float(np.clip(intention, -1.0, 1.0))
    self._last_intention = intention
    logger.debug(f"Intention interp -> cls={cls} sev={sev:.2f} speed={speed:.1f} intention={intention:.2f}")
    return intention
