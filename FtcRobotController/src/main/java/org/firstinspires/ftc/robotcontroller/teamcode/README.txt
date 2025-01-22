This package has to be here in order to allow FtcRobotControllerActivity to access HardwareMechanismClassManager (and it's dependencies) without introducing a circular dependency between the two modules. Yes, it's really dumb and bad. No, I don't know how to fix it.

TODO: This could all be solved just by switching to Sinister (https://docs.dairy.foundation/Sinister). :/