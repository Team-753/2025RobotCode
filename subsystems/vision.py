from photonlibpy import PhotonCamera  # Import PhotonCamera

class Vision:
    def __init__(self):
        self.camera = PhotonCamera("AprilTag_Camera")  # Initialize camera

    def GetTargetData(self):
        result = self.camera.getLatestResult()  # Get latest vision result
        if not result.hasTargets():  # If no targets detected, return None
            return None

        # Extracting the best target (the one PhotonVision determines is best)
        best_target = result.getBestTarget()
        
        # Extracting useful data
        target_id = best_target.getFiducialId()  # AprilTag ID
        yaw = best_target.getYaw()  # Horizontal angle offset
        pitch = best_target.getPitch()  # Vertical angle offset
        distance = best_target.getBestCameraToTarget().getTranslation().getNorm()  # Distance in meters
        
        # Returning as a dictionary
        return {
            "id": target_id,
            "yaw": yaw,
            "pitch": pitch,
            "distance": distance
        }

    def PrintTargetData(self):
        """ Fetches target data and prints it in a readable format. """
        target_data = self.GetTargetData()

        if target_data:
            print(f"Target Detected! ID: {target_data['id']}, Distance: {target_data['distance']:.2f}m, "
                  f"Yaw: {target_data['yaw']:.2f}°, Pitch: {target_data['pitch']:.2f}°")
        else:
            print("No Target Detected")
