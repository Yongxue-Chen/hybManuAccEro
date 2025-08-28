import argparse
from typing import List


def parse_vector_from_third_line(input_path: str) -> List[int]:
    """Read the 3rd line from the input file and parse it as a comma-separated 0/1 vector."""
    with open(input_path, 'r', encoding='utf-8') as f:
        lines = f.read().splitlines()
    if len(lines) < 3:
        raise ValueError("Input file must have at least 3 lines.")
    third_line = lines[2].strip()
    if not third_line:
        return []
    tokens = [t.strip() for t in third_line.split(',')]
    try:
        vec = [int(t) for t in tokens if t != ""]
    except ValueError as e:
        raise ValueError("Third line must contain only 0/1 integers separated by commas.") from e
    for x in vec:
        if x not in (0, 1):
            raise ValueError("Third line must contain only 0/1 values.")
    return vec


def write_vector_mapping_txt(input_path: str, output_path: str) -> None:
    """
    - Read the 3rd line (comma-separated 0/1 vector) from input_path.
    - Write output_path with:
        Line 1: n+1
        Line 2..n+1: for each v[j] (j=0..n-1):
            if v[j]==1 -> write "j,n+2"
            else       -> write "n+2,n+3"
    """
    v = parse_vector_from_third_line(input_path)
    n = len(v)
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(f"{n+1}\n")
        for j, val in enumerate(v):
            if val == 1:
                f.write(f"{j},{n+2}\n")
            else:
                f.write(f"{n+2},{n+3}\n")


def main():
    input_txt = "TO_noNeedSupport.txt"
    output_txt = "TO_noNeedSupport_AccEroHMP.txt"
    write_vector_mapping_txt(input_txt, output_txt)


if __name__ == "__main__":
    main()
