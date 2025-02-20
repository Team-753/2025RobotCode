"""
from photonlibpy import PhotonCamera
from wpilib import SmartDashboard

class Vision:
    def __init__(self):
        self.camera = PhotonCamera("Apriltag_Camera")
        self._prev_locked_state = None  # We'll store the last known bool here


        SmartDashboard.setDefaultBoolean ("Locked(1)", False)
        SmartDashboard.setDefaultBoolean ("Locked(2)", False)

    def update(self):

        current_locked_state = self._isTargetLocked()
        
        if current_locked_state != self._prev_locked_state:
            if current_locked_state:

                SmartDashboard.putBoolean("Locked(1)", True)  # Initialize Locked state as False
                SmartDashboard.putBoolean("Locked(2)", True)  # Initialize Locked state as False
                

            else:
                SmartDashboard.putBoolean("Locked(1)", False)  # Set Locked to False when target is not locked
                SmartDashboard.putBoolean("Locked(2)", False)

            # Update our previous state
            self._prev_locked_state = current_locked_state

    def _isTargetLocked(self) -> bool:
    
        result = self.camera.getLatestResult()
        return result.hasTargets()  # Returns True if a target is detected
"""