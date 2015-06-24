class Tracker(object):
    def __init__(self, generate_configs, max_empty_frames):
        self.generate_configs = generate_configs 
        self.max_empty_frames = max_empty_frames
        self.empty_frames_count = 0
        self.configurations = []

    def createConfigurations(self, locations):
        self.configurations = self.generate_configs(locations)

    def update(self, locations):
        # Update the empty frames count and fail if too many frames dropped
        if len(locations) == 0:
            self.empty_frames_count += 1
            if self.empty_frames_count > self.max_empty_frames:
                self.resetTracking()
            return
        else:
            self.empty_frames_count = 0

        if len(self.configurations) == 0:
            self.configurations = self.createConfigurations(locations)
        else:
            # Update the potential configurations while removing any
            # configurations that do not fit the observations
            self.configurations = [config for config in self.configurations
                                   if config.update(locations)]

        if len(self.configurations) == 0:
            self.resetTracking()

    def resetTracking(self):
        self.empty_frames_count = 0
        self.configurations = []
