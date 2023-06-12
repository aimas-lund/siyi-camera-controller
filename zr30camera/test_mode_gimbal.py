import siyi_sdk


def test():
    camera = siyi_sdk.SIYISDK()
    pitch = 90.0
    yaw = 90.0
    camera.setGimbalRotation(yaw, pitch)