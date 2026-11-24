import argparse
import os
import re
import shutil
from pathlib import Path
from tempfile import TemporaryDirectory

import casadi as ca

from .injection import injection_model
from .nominal import nominal_state_model
from .state import IMU, ErrorState, ProcessNoise, State
from .tangent import tangent_state_model

CODEGEN_OPTIONS = {
    "with_header": True,
    "casadi_real": "float",
    "casadi_int": "int",
}

INDEX_CLASSES = (State, ErrorState, IMU, ProcessNoise)


def generate_index(output_dir: Path, filename: str = "eskf_indices"):
    lines = [
            "#ifndef ULYSSES_TOOLS_CODEGEN_ESKF_INDICES",
            "#define ULYSSES_TOOLS_CODEGEN_ESKF_INDICES",
            "",
            "// Auto-generated. Do not edit.",
            "",
            "enum {",
        ]
    
    for cls in INDEX_CLASSES:
        prefix = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", cls.__name__).upper()

        for name, value in vars(cls).items():
            if name.startswith("_"):
                continue

            key = f"{prefix}_{name}"

            if isinstance(value, slice):
                if value.start is None or value.stop is None or value.step not in (None, 1):
                    raise ValueError(f"Invalid slice: {key}")

                lines.extend(
                    [
                        f"    {key}_OFFSET = {value.start},",
                        f"    {key}_SIZE = {value.stop - value.start},",
                    ]
                )
            elif isinstance(value, int):
                lines.append(f"    {key} = {value},")

        lines.append("")

    lines.extend(
        [
            "};",
            "",
            "#endif // ULYSSES_TOOLS_CODEGEN_ESKF_INDICES"
        ]
    )
    
    contents = "\n".join(lines) + "\n"
    
    filename_with_suffix = filename + ".h"
    
    output = output_dir / filename_with_suffix
    output.write_text(contents, encoding="utf-8")
    
    return output
    

def generate_model(output_dir: Path, filename: str = "eskf_model"):
    filename_with_suffix = filename + ".c"
    
    eskf = ca.CodeGenerator(filename_with_suffix, CODEGEN_OPTIONS)
    eskf.add(nominal_state_model())
    eskf.add(tangent_state_model())
    eskf.add(injection_model())
    source = Path(eskf.generate(str(output_dir) + os.sep))
    
    return source, source.with_suffix(".h")
    
    
# def generate_measurement(output_dir: Path, filename: str = "eskf_measurment") -> None:
#     filename_with_suffix = filename + ".c"
#     measurement = ca.CodeGenerator(filename_with_suffix, CODEGEN_OPTIONS)
#     measurement.generate(str(output_dir) + os.sep)
    


def main() -> None:
    parser = argparse.ArgumentParser(description="generate ESKF autogen C source and headers.")
    parser.add_argument("--header-dir", type=Path, required=True)
    parser.add_argument("--source-dir", type=Path, required=True)
    args = parser.parse_args()

    args.header_dir.mkdir(parents=True, exist_ok=True)
    args.source_dir.mkdir(parents=True, exist_ok=True)
    
    with TemporaryDirectory() as temp_dir:
        temp_dir_path = Path(temp_dir)
        
        index_header = generate_index(temp_dir_path)
        model_source, model_header = generate_model(temp_dir_path)
        # generate_measurement(temp_dir_path)
        
        headers = (index_header, model_header)
        sources = (model_source, )
    
        for header in headers:
            shutil.move(header, args.header_dir / header.name)
        
        for source in sources:
            shutil.move(source, args.source_dir / source.name)
        
        
if __name__ == "__main__":
    main()
