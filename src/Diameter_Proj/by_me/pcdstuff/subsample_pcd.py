#!/usr/bin/env python3
import sys, random

def usage():
    print("Usage: subsample_pcd.py INPUT_PCD OUTPUT_PCD [N=1000]")

def main():
    if len(sys.argv) < 3:
        usage()
        sys.exit(1)
    infile = sys.argv[1]
    outfile = sys.argv[2]
    N = int(sys.argv[3]) if len(sys.argv) > 3 else 1000

    # Read header and data lines
    header_lines = []
    data_lines = []
    in_data = False
    with open(infile, 'r') as f:
        for line in f:
            if not in_data:
                header_lines.append(line.rstrip('\n'))
                if line.strip().upper().startswith('DATA'):
                    in_data = True
            else:
                if line.strip() == '':
                    continue
                data_lines.append(line.rstrip('\n'))

    total = len(data_lines)
    if total == 0:
        print('No data lines found in', infile)
        sys.exit(1)

    if total <= N:
        chosen = list(range(total))
    else:
        random.seed(0)
        chosen = sorted(random.sample(range(total), N))

    points = []
    for idx in chosen:
        toks = data_lines[idx].split()
        if len(toks) < 3:
            # skip malformed
            continue
        points.append((toks[0], toks[1], toks[2]))

    # Extract some header values to copy
    version = '0.7'
    viewpoint = '0 0 0 1 0 0 0'
    for h in header_lines:
        if h.startswith('VERSION'):
            parts = h.split()
            if len(parts) > 1:
                version = parts[1]
        if h.startswith('VIEWPOINT'):
            parts = h.split()
            if len(parts) > 1:
                viewpoint = ' '.join(parts[1:])

    # Write minimal ASCII PCD with x y z only
    with open(outfile, 'w') as out:
        out.write(f"# .PCD v{version} - Point Cloud Data file format\n")
        out.write(f"VERSION {version}\n")
        out.write("FIELDS x y z\n")
        out.write("SIZE 4 4 4\n")
        out.write("TYPE F F F\n")
        out.write("COUNT 1 1 1\n")
        out.write(f"WIDTH {len(points)}\n")
        out.write("HEIGHT 1\n")
        out.write(f"VIEWPOINT {viewpoint}\n")
        out.write(f"POINTS {len(points)}\n")
        out.write("DATA ascii\n")
        for x,y,z in points:
            out.write(f"{x} {y} {z}\n")

    print(f"Wrote {len(points)} points to: {outfile}")

if __name__ == '__main__':
    main()
