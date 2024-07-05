import os
from typing import Dict
import warnings

CACHE_FOLDER = f"{os.path.dirname(__file__)}/.cache"
os.makedirs(CACHE_FOLDER, exist_ok=True)
TEMP_URDF_LOCATION = f"{CACHE_FOLDER}/temp.urdf"


def parse_urdf_file(urdf_path: str, params: Dict[str, float]) -> str:
    with open(urdf_path, 'r') as file:
        urdf_content = file.read()

    # Replace all parameters in the URDF file
    items = [(k,v) for k, v in params.items()]
    for (key, value) in items:
        urdf_content = urdf_content.replace("{" + key + "}", str(value))
        del params[key]
    if len(params) > 0:
        warnings.warn(f"Unused parameters: {params}")
    
    missing_params = []
    while "{" in urdf_content:
        start = urdf_content.index("{") + 1
        end = urdf_content.index("}", start)
        missing_param = urdf_content[start:end]
        urdf_content = urdf_content.replace("{" + missing_param + "}", "MISSING")
        missing_params.append(missing_param)
    if len(missing_params) > 0:
        raise ValueError(f"Missing parameters: {missing_params}")


    # Evaluate all expressions in eval[...] tags
    while "eval[" in urdf_content:
        start = urdf_content.index("eval[") + 5
        end = urdf_content.index("]", start)
        expression = urdf_content[start:end]
        value = str(eval(expression))
        urdf_content = urdf_content[:start - 5] + value + urdf_content[end + 1:]

    # Write the modified URDF to a temporary file
    with open(TEMP_URDF_LOCATION, 'w') as file:
        file.write(urdf_content)
    return TEMP_URDF_LOCATION