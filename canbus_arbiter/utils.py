"""Utility types and functions."""


def clamp(value, thresh):
    """A value bounded by a minimum and a maximum."""
    if value < -thresh:
        return -thresh
    elif value > thresh:
        return thresh
    else:
        return value
