def get_movement(error_x, error_y, threshold=50):

    horizontal = "HOLD"
    vertical = "HOLD"

    if error_x > threshold:
        horizontal = "RIGHT"

    elif error_x < -threshold:
        horizontal = "LEFT"

    if error_y > threshold:
        vertical = "DOWN"

    elif error_y < -threshold:
        vertical = "UP"

    return horizontal, vertical