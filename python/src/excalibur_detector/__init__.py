from excalibur_detector._version import get_versions
from excalibur_detector.control import ExcaliburAdapter

__version__ = get_versions()["version"]
del get_versions

__all__ = ["ExcaliburAdapter", "__version__"]
