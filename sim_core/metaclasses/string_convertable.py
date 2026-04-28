import json
import numpy as np
from typing import Any

class StringConvertible:
    """
    Mixin that provides a default string representation
    and JSON serialization for any inheriting class.
    """

    def __str__(self) -> str:
        attrs = ", ".join(
            f"{key}={value!r}"
            for key, value in self.__dict__.items()
        )
        return f"{self.__class__.__name__}({attrs})"

    def __repr__(self) -> str:
        return self.__str__()
    
    def to_dict(self) -> dict:
        """
        Recursively converts the object to a dictionary,
        handling nested objects, arrays, and numpy arrays.
        """
        def convert_value(value: Any) -> Any:
            # If it's a numpy array, convert to list
            if isinstance(value, np.ndarray):
                return value.tolist()
            # If it's a numpy scalar type
            elif isinstance(value, (np.integer, np.floating)):
                return value.item()
            # If it's a StringConvertible instance, recursively convert
            elif isinstance(value, StringConvertible):
                return value.to_dict()
            # If it's a list, convert each element
            elif isinstance(value, list):
                return [convert_value(item) for item in value]
            # If it's a dict, convert each value
            elif isinstance(value, dict):
                return {k: convert_value(v) for k, v in value.items()}
            # If it's a set or tuple, convert to list
            elif isinstance(value, (set, tuple)):
                return [convert_value(item) for item in value]
            # For basic types, return as-is
            else:
                return value
        
        return {
            key: convert_value(value)
            for key, value in self.__dict__.items()
        }
    
    def to_json(self, indent: int = 2) -> str:
        """
        Returns a JSON string representation of the object.
        
        Args:
            indent: Number of spaces for indentation (default: 2)
        
        Returns:
            JSON string
        """
        return json.dumps(self.to_dict(), indent=indent)
    
    def save_json(self, filename: str, indent: int = 2) -> None:
        """
        Saves the object as JSON to a file.
        
        Args:
            filename: Path to the output file
            indent: Number of spaces for indentation (default: 2)
        """
        with open(filename, 'w') as f:
            json.dump(self.to_dict(), f, indent=indent)