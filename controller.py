from models.hexapod import HexapodModel


class HexapodController:

    def __init__(self, hexapod: HexapodModel):
        self.hexapod = hexapod

    def set_angles(self, angles):
        self.hexapod.set_angles(angles)