import argparse
from .runner import run, analyze, report

def main():
    p=argparse.ArgumentParser(description="Run and analyze Rust swarm experiments")
    sub=p.add_subparsers(dest="command",required=True)
    r=sub.add_parser("run"); r.add_argument("manifest"); r.add_argument("--binary",required=True); r.add_argument("--output",required=True); r.add_argument("--timeout",type=float,default=60)
    a=sub.add_parser("analyze"); a.add_argument("directory")
    s=sub.add_parser("report"); s.add_argument("directory")
    args=p.parse_args()
    if args.command=="run": return run(args.manifest,args.binary,args.output,args.timeout)
    if args.command=="analyze": analyze(args.directory)
    else: report(args.directory)
    return 0

if __name__=="__main__": raise SystemExit(main())
