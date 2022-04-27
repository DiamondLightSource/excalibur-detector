from ._version import get_versions
from .adapter import ExcaliburAdapter

__version__ = get_versions()["version"]

__all__ = ["ExcaliburAdapter", "__version__"]
