import pybullet as p
import pybullet_utils.bullet_client as bc
from typing import Optional
import warnings
import numpy as np

class KeyboardHandler:

    def __init__(self):
        self.keys_pressed = set()
        self.physics_client: Optional[bc.BulletClient] = None

    def update(self, controller):
        if self.physics_client is None:
            warnings.warn("No physics client set for KeyboardHandler")
            return
        keys = self.physics_client.getKeyboardEvents()
        new_keys_pressed = set()
        for k, v in keys.items():
            if v & p.KEY_IS_DOWN:
                self.keys_pressed.add(k)
                if v & p.KEY_WAS_TRIGGERED:
                    new_keys_pressed.add(k)
            elif v & p.KEY_WAS_RELEASED and k in self.keys_pressed:
                self.keys_pressed.remove(k)

        # Action on continuisly pressed keys
        # Translate
        directions = []
        if p.B3G_LEFT_ARROW in self.keys_pressed:
            directions.append(np.pi)
        if p.B3G_RIGHT_ARROW in self.keys_pressed:
            directions.append(0)
        if p.B3G_UP_ARROW in self.keys_pressed:
            directions.append(np.pi/2)
        if p.B3G_DOWN_ARROW in self.keys_pressed:
            directions.append(-np.pi/2)
        direction = 0
        speed = 0.
        if len(directions) > 0 and set(directions) != {0, np.pi}:
            direction = sum(directions) / len(directions)
            speed = 1.
        # Rotate
        rotation_speed = 0.
        if ord('n') in self.keys_pressed:
            rotation_speed += 1.
        if ord('m') in self.keys_pressed:
            rotation_speed += -1.
        controller.set_command(direction, speed, rotation_speed)
        