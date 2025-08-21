# steering.py
import numpy as np
from log import logger

def steering_controller(self, sensors, aggressiveness=1.0):
    """
    Calcula steer baseado no sensor 'track' (lista de distâncias).
    agressiveness: 0.0..1.0 (para largada podemos reduzir a agressividade)
    """
    track = sensors.get('track', None)
    if track is None or len(track) == 0:
        # mantém ultimo valor
        return float(np.clip(getattr(self, 'steering', 0.0), -1.0, 1.0))

    try:
        t = np.array(track, dtype=float)
        center_idx = len(t) // 2
        center_dist = float(t[center_idx])
        left = t[:center_idx]
        right = t[center_idx+1:]

        left_mean = float(np.mean(left)) if left.size > 0 else center_dist
        right_mean = float(np.mean(right)) if right.size > 0 else center_dist

        denom = (left_mean + right_mean)
        if denom <= 1e-6:
            return 0.0

        steer_raw = (left_mean - right_mean) / denom
        steer_raw = float(np.clip(steer_raw, -1.0, 1.0))

        steer = steer_raw * float(np.clip(aggressiveness, 0.0, 1.0))
        steer = float(np.clip(steer, -1.0, 1.0))
        return steer
    except Exception as e:
        logger.warning(f"steering_controller error: {e}")
        return float(np.clip(getattr(self, 'steering', 0.0), -1.0, 1.0))
