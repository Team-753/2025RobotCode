from photonlibpy import targeting, PhotonCamera, photonCamera

class Vision:
    def __init__(self):
        self.camera = PhotonCamera("AprilTag_Camera")

    def GetTarget(self):
        result = self.camera.getLatestResult()  # Always fetch latest result
        return result.hasTargets()  # Return up-to-date value
