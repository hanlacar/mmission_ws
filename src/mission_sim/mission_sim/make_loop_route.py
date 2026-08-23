"""8자 순환 경로 생성 (교차로를 여러 번 지남)."""
import sys, math, os


def generate(out_path, A=20.0, B=12.0, n=400):
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        for i in range(n + 1):
            t = 2 * math.pi * i / n
            x = A * math.sin(t)
            y = B * math.sin(t) * math.cos(t)
            f.write(f"{x:.3f},{y:.3f}\n")
    return n + 1


def main():
    out = None
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if args:
        out = args[0]
    if not out:
        out = os.path.expanduser("~/mission_ws/routes/loop.csv")
    n = generate(out)
    print(f"8자 순환 경로 생성: {out} ({n} points)")


if __name__ == "__main__":
    main()
