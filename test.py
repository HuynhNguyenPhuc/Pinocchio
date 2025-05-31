import os
import argparse
from trimesh.exchange import obj as obj_export
import trimesh

def process(file_path: str, output_dir: str, verbose: bool = False):
    os.makedirs(output_dir, exist_ok=True)
    if verbose:
        print(f"Creating output directory: {output_dir}")

    mesh = trimesh.load(file_path, force='mesh')
    if verbose:
        print(f"Loaded mesh from {file_path}.")

    output_path = os.path.join(output_dir, 'model.obj')

    mesh.export(output_path, include_normals=True)
    print(f"Exported OBJ to: {output_path}")

    obj_text = obj_export.export_obj(
        mesh,
        include_normals=False,
        include_texture=False,
        include_color=False,
        write_texture=False
    )

    geom_output_path = os.path.join(output_dir, 'model_geom.obj')

    with open(geom_output_path, 'w') as f:
        f.write(obj_text)

    print(f"Exported geometry to: {geom_output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f", "--file", type=str, required=True, help="Path to the input GLB file"
    )
    parser.add_argument(
        "-o", "--output", type=str, required=True, help="Directory to save the output OBJ file"
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Enable verbose output"
    )

    args = parser.parse_args()
    process(args.file, args.output, args.verbose)