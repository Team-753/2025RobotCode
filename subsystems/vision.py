from photonlibpy import PhotonCamera

class Vision:
    def __init__(self):
        self.camera = PhotonCamera("Apriltag_Camera")
        self._prev_locked_state = None  # We'll store the last known bool here

    def update(self):
        """
        Check if the camera sees a target and handle any changes. 
        All logic + prints happen here in vision.py.
        """

        # We'll consider 'locked' if hasTargets == True
        current_locked_state = self._isTargetLocked()
        
        # Only print if there's a change from False->True or True->False
        if current_locked_state != self._prev_locked_state:
            if current_locked_state:
                print("Target Locked = TRUE")
            else:
                print("Target Locked = FALSE")

            # Update our previous state
            self._prev_locked_state = current_locked_state

    def _isTargetLocked(self) -> bool:
        """
        Internal helper function to check the camera's current result.
        Returns True if we see at least one target, False otherwise.
        """
        result = self.camera.getLatestResult()
        return result.hasTargets()
