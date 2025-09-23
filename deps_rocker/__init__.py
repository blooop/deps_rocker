"""
deps_rocker package - A rocker plugin to help installing apt and pip dependencies
"""

# Import the rocker hooks to automatically patch extension discovery
try:
    from . import rocker_hooks
except ImportError:
    # If rocker is not available, that's fine
    pass

__version__ = "0.12.0"
