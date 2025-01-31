from photonlibpy import PhotonCamera  # Import PhotonCamera

class Vision:
    def __init__(self):
        self.camera = PhotonCamera("AprilTag_Camera")  # Initialize camera

    def GetTargetData(self):
        result = self.camera.getLatestResult()  # Get latest vision result
        if not result.hasTargets():  # If no targets detected, return None
            return None

        # Extracting useful data
        targets = result.getTargets()  # Gets all detected targets
        first_target = targets[0]  # Use the first target in the list
        
        target_id = first_target.getFiducialId()  # AprilTag ID
        yaw = first_target.getYaw()  # Horizontal angle offset
        pitch = first_target.getPitch()  # Vertical angle offset
        distance = first_target.getCameraToTarget().getTranslation().getNorm()  # Distance in meters

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
