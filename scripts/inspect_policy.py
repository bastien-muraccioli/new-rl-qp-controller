#!/usr/bin/env python3

import sys
import onnxruntime as ort


def shape_to_string(shape):
    return "[" + ", ".join(str(dim) for dim in shape) + "]"


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <model.onnx>")
        sys.exit(1)

    model_path = sys.argv[1]

    session = ort.InferenceSession(model_path)

    print(f"Model: {model_path}")
    print()

    print("Inputs:")
    for inp in session.get_inputs():
        print(f"  Name : {inp.name}")
        print(f"  Type : {inp.type}")
        print(f"  Shape: {shape_to_string(inp.shape)}")

        # Compute flattened size if all dimensions are known
        if all(isinstance(dim, int) for dim in inp.shape):
            size = 1
            for dim in inp.shape:
                size *= dim
            print(f"  Total elements: {size}")
        else:
            print("  Total elements: dynamic")
        print()

    print("Outputs:")
    for out in session.get_outputs():
        print(f"  Name : {out.name}")
        print(f"  Type : {out.type}")
        print(f"  Shape: {shape_to_string(out.shape)}")

        if all(isinstance(dim, int) for dim in out.shape):
            size = 1
            for dim in out.shape:
                size *= dim
            print(f"  Total elements: {size}")
        else:
            print("  Total elements: dynamic")
        print()


if __name__ == "__main__":
    main()
