import argparse
from pathlib import Path
from .runner import run, analyze, report

def main():
    p=argparse.ArgumentParser(description="Run and analyze Rust swarm experiments")
    sub=p.add_subparsers(dest="command",required=True)
    r=sub.add_parser("run"); r.add_argument("manifest"); r.add_argument("--binary",required=True); r.add_argument("--output",required=True); r.add_argument("--timeout",type=float,default=60); r.add_argument("--trace-every",type=int,default=1); r.add_argument("--component-radius",type=float,default=1.0)
    a=sub.add_parser("analyze"); a.add_argument("directory"); a.add_argument("--component-radius",type=float)
    s=sub.add_parser("report"); s.add_argument("directory")
    args=p.parse_args()
    try:
        if args.command=="run": return run(args.manifest,args.binary,args.output,args.timeout,args.trace_every,args.component_radius)
        if args.command=="analyze":
            try:
                analyze(args.directory, component_radius=args.component_radius)
            finally:
                parent = Path(args.directory).resolve().parent
                if (parent / "manifest.json").exists(): report(parent)
        else: report(args.directory)
    except (ValueError, OSError) as exc:
        p.exit(1, f"Error: {exc}\n")
    return 0

if __name__=="__main__": raise SystemExit(main())
